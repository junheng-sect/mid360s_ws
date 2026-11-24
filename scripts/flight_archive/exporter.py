"""Persist closed Orange Pi recordings and export their matching PX4 ULog."""
import hashlib
import json
from pathlib import Path
import struct
import shutil
import sys
import threading
import time

from .pairing import bag_window, describe, run_identity, start_delta, verify_pair, write_json
from .transport import LogTransfer

DATA = Path("/home/orangepi/test_data")
SPOOL = DATA / "ulog_exports"
BOARD_SCRIPTS = Path("/home/orangepi/mid360s_ws/scripts")
MAGIC = b"ULog\x01\x12\x35"


def associated_files(bag):
    files = [bag]
    if bag.parent != DATA / "bags":
        for path in sorted(bag.parent.iterdir()):
            if path.is_symlink() or not path.is_file():
                continue
            if path.suffix == ".log" or (
                path.suffix == ".json" and
                path.name.startswith(("goal_", "flight_", "drone_reloc_", "report"))
            ):
                files.append(path)
    if "_rc_" in bag.name:
        raw = bag.with_name(bag.name.replace("_rc_", "_raw_", 1))
        if raw.is_file():
            files.append(raw)
    return files


class Exporter:
    def __init__(self, ros):
        self.ros = ros
        self.lock = threading.Lock()
        self.bag_open = False
        self.state_at = 0.0
        self.safe = False
        self.armed = False
        self.rc_at = 0.0
        self.rc_high = False
        self.queue = SPOOL / "pending"
        self.queue.mkdir(parents=True, exist_ok=True)
        self.states = {}

    def state(self, msg):
        self.safe = msg.connected and not msg.armed
        self.armed = msg.connected and msg.armed
        self.state_at = time.monotonic()

    def rc(self, msg):
        if len(msg.channels) >= 6 and 900 <= msg.channels[5] <= 2100:
            self.rc_high = msg.channels[5] >= 1650
            self.rc_at = time.monotonic()

    def event(self, msg):
        action, _, path = msg.data.partition(" ")
        if action == "OPEN":
            self.bag_open = True
            return
        if action not in ("CLOSED", "CLOSE_FAILED"):
            return
        self.bag_open = False
        if action != "CLOSED":
            return
        bag = Path(path).resolve()
        if DATA not in bag.parents or bag.suffix != ".bag" or not bag.is_file():
            return
        key = hashlib.sha256(str(bag).encode()).hexdigest()[:24]
        with self.lock:
            pending = self.queue / (key + ".json")
            if not pending.exists():
                write_json(pending, {"bag": str(bag), "closed_at": time.time(),
                                     "status": "pending"})

    def check(self):
        now = time.monotonic()
        if self.ros.is_shutdown() or not self.safe or now - self.state_at > 2.8:
            raise RuntimeError("等待已连接并上锁的飞控")
        if self.bag_open or (self.rc_high and now - self.rc_at < 3):
            raise RuntimeError("等待录包开关归位并结束录包")

    def collect(self, window, folder):
        # Reuse the existing quiet GCS bridge without transmitting a GCS heartbeat.
        from pymavlink import mavutil
        from fc_set_time import keep_local_port_open

        self.check()
        link = mavutil.mavlink_connection("udpout:127.0.0.1:14555",
                                         source_system=255, source_component=190)
        try:
            keep_local_port_open(link)
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                self.check()
                heartbeat = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
                if (heartbeat is not None and heartbeat.get_srcSystem() == 1
                        and heartbeat.get_srcComponent() == 1
                        and heartbeat.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4):
                    if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                        raise RuntimeError("飞控已解锁")
                    break
            else:
                raise TimeoutError("MAVROS 本地口未收到 PX4 心跳")
            link.target_system, link.target_component = 1, 1
            matches = []
            part_paths = []
            try:
                with LogTransfer(link, self.check) as transfer:
                    entries = transfer.list_logs()
                    for entry in reversed(entries):
                        self.check()
                        if entry.size < 16:
                            continue
                        header = transfer.read(entry, 0, 16)
                        if header[:7] != MAGIC:
                            continue
                        boot_us = struct.unpack_from("<Q", header, 8)[0]
                        if abs(start_delta(window, boot_us)) > 1.5:
                            continue
                        part = folder / ("candidate_{}.ulg.part".format(entry.id))
                        part_paths.append(part)
                        transfer.fetch(entry, part)
                        evidence = verify_pair(window, part)
                        if evidence["matches"]:
                            matches.append((entry, part, evidence))
                    if len(matches) != 1:
                        return ("ambiguous" if matches else "missing_ulog",
                                {"candidate_ids": [entry.id for entry, _, _ in matches]}, None)
                    entry, part, evidence = matches[0]
                    evidence.update({"px4_log_id": entry.id, "px4_utc_s": entry.time_utc})
                    self.check()
                    final = folder / ("px4_log_{}_{}.ulg".format(
                        evidence["ulog_start_boot_us"], entry.size
                    ))
                    if final.exists():
                        if describe(final)["sha256"] != describe(part)["sha256"]:
                            raise ValueError("已导出的 ULog 内容冲突")
                    else:
                        part.replace(final)
                    return "paired", evidence, final
            finally:
                for path in part_paths:
                    if path.exists():
                        path.unlink()
        finally:
            link.close()

    def process(self, pending):
        job = json.loads(pending.read_text())
        if job["status"] == "paired" or time.time() - job["closed_at"] < 3:
            return
        if time.monotonic() < self.states.get(pending, 0):
            return
        if self.bag_open or self.armed:
            return
        self.states[pending] = time.monotonic() + (120 if self.safe else 8)
        bag = Path(job["bag"])
        window = bag_window(bag, native=True)
        identity = run_identity(bag, window)
        folder = SPOOL / "runs" / identity
        folder.mkdir(parents=True, exist_ok=True)
        manifest = folder / "manifest.json"
        record = None
        if manifest.exists():
            record = json.loads(manifest.read_text())
            if record["status"] == "paired":
                job["status"] = "paired"
                write_json(pending, job)
                return
        files = list(record["files"]) if record else [describe(bag)]
        status, evidence, ulg = "missing_timesync", {}, None
        if window["offset_ns"] is not None:
            try:
                self.check()
                status, evidence, ulg = self.collect(window, folder)
            except Exception as error:
                status, evidence = "pending", {"reason": str(error)}
        if ulg is not None:
            snapshots = folder / "sidecars"
            snapshots.mkdir(exist_ok=True)
            for original in associated_files(bag)[1:]:
                if original.suffix == ".bag":
                    files.append(describe(original))
                    continue
                snapshot = snapshots / original.name
                if not snapshot.exists():
                    temporary = snapshot.with_suffix(snapshot.suffix + ".tmp")
                    shutil.copy2(original, temporary)
                    temporary.replace(snapshot)
                item = describe(snapshot)
                item["original_source"] = str(original)
                files.append(item)
            files.append(describe(ulg))
        write_json(manifest, {"run_id": identity, "status": status,
                              "window": window, "pairing": evidence, "files": files})
        job["status"] = status
        write_json(pending, job)
        print("{} {}".format(identity, status), flush=True)

    def serve(self):
        while not self.ros.is_shutdown():
            for pending in sorted(self.queue.glob("*.json")):
                try:
                    self.process(pending)
                except Exception as error:
                    print("{}: {}".format(pending.name, error), flush=True)
                if self.ros.is_shutdown():
                    break
            time.sleep(2)


def main():
    sys.path.insert(0, str(BOARD_SCRIPTS))
    import rospy
    from mavros_msgs.msg import RCIn, State
    from std_msgs.msg import String

    rospy.init_node("mid360s_ulog_export")
    service = Exporter(rospy)
    rospy.Subscriber("/mavros/state", State, service.state, queue_size=1)
    rospy.Subscriber("/mavros/rc/in", RCIn, service.rc, queue_size=1)
    rospy.Subscriber("/rc_record/bag_path", String, service.event, queue_size=32)
    print("ULog export: waiting for a closed recording", flush=True)
    service.serve()


if __name__ == "__main__":
    main()
