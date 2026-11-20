"""Match ROS and PX4 recording windows using the bag's time synchronization."""
from datetime import datetime, timedelta, timezone
import hashlib
import json
import os
from pathlib import Path
from statistics import median

LOCAL_TZ = timezone(timedelta(hours=8))
TIMESYNC = "/mavros/timesync_status"


def digest(path):
    sha = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            sha.update(block)
    return sha.hexdigest()


def write_json(path, value):
    path = Path(path)
    part = path.with_name(path.name + ".tmp")
    with part.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    part.replace(path)


def bag_window(path, native=False):
    """Read the index and the first few seconds of TIMESYNC, not the point clouds."""
    if native:
        import rosbag
        import rospy

        with rosbag.Bag(str(path), "r") as bag:
            start = round(bag.get_start_time() * 1e9)
            end = round(bag.get_end_time() * 1e9)
            offsets = [
                msg.estimated_offset_ns
                for _, msg, _ in bag.read_messages(
                    topics=[TIMESYNC], end_time=rospy.Time.from_sec(start / 1e9 + 5)
                )
                if msg.remote_timestamp_ns > 0 and msg.estimated_offset_ns > 0
            ]
    else:
        from rosbags.highlevel import AnyReader
        from rosbags.typesys import Stores, get_typestore

        with AnyReader([Path(path)], default_typestore=get_typestore(Stores.ROS1_NOETIC)) as bag:
            start, end = bag.start_time, bag.end_time
            connections = [c for c in bag.connections if c.topic == TIMESYNC]
            offsets = []
            if connections:
                for c, _, raw in bag.messages(connections, stop=start + 5_000_000_000):
                    msg = bag.deserialize(raw, c.msgtype)
                    if msg.remote_timestamp_ns > 0 and msg.estimated_offset_ns > 0:
                        offsets.append(msg.estimated_offset_ns)
    offset = int(median(offsets)) if len(offsets) >= 3 else None
    if offset is not None and median(abs(x - offset) for x in offsets) > 50_000_000:
        offset = None
    return {"start_ns": start, "end_ns": end, "offset_ns": offset,
            "timesync_samples": len(offsets)}


def run_identity(bag, window):
    stamp = datetime.fromtimestamp(window["start_ns"] / 1e9, LOCAL_TZ)
    return stamp.strftime("%Y-%m-%d/%H%M%S_") + Path(bag).stem


def start_delta(window, boot_us):
    if window["offset_ns"] is None:
        return None
    return (boot_us * 1000 + window["offset_ns"] - window["start_ns"]) / 1e9


def verify_pair(window, ulg_path):
    from pyulog import ULog

    ulg = ULog(str(ulg_path))
    if ulg.file_corruption or not ulg.data_list:
        raise ValueError("ULog 文件损坏或没有数据")
    start = start_delta(window, ulg.start_timestamp)
    end = None if start is None else (
        ulg.last_timestamp * 1000 + window["offset_ns"] - window["end_ns"]
    ) / 1e9
    return {
        "method": "mavros_timesync_and_recording_window",
        "start_delta_s": None if start is None else round(start, 6),
        "end_delta_s": None if end is None else round(end, 6),
        "ulog_start_boot_us": ulg.start_timestamp,
        "ulog_end_boot_us": ulg.last_timestamp,
        "matches": start is not None and abs(start) <= 1.5 and abs(end) <= 2.5,
    }


def describe(path):
    path = Path(path)
    before = path.stat()
    sha = digest(path)
    after = path.stat()
    if (before.st_size, before.st_mtime_ns) != (after.st_size, after.st_mtime_ns):
        raise ValueError("文件仍在变化：{}".format(path))
    return {"name": path.name, "source": str(path), "size": after.st_size, "sha256": sha}
