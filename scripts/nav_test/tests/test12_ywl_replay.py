#!/usr/bin/env python3
"""Replay raw outdoor sensors with FAST-LIO2 and the live YWL startup sequence."""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import time
from xmlrpc.client import ServerProxy

import numpy as np
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

EV_TOPIC = "/mavros/vision_pose/pose_cov"
RAW_TOPICS = ("/livox/lidar", "/livox/imu")


class Replay:
    def __init__(self, args):
        self.args = args
        self.children = []
        self.samples = {name: [] for name in (
            "raw", "fastlio", "ev", "bootstrap", "ywl", "init", "status", "reloc", "resources")}
        self.start = time.monotonic()

    def spawn(self, log, *argv):
        with (self.args.out / f"{log}.log").open("wb") as output:
            child = subprocess.Popen(argv, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append(child)
        return child

    def wait(self, condition, seconds, error):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if any(child.poll() is not None for child in self.children):
                raise RuntimeError("replay process exited early")
            try:
                if condition():
                    return
            except (OSError, rosgraph.MasterException):
                pass
            time.sleep(.05)
        raise RuntimeError(error)

    def record(self, key, stamp, *values):
        self.samples[key].append((stamp, time.monotonic() - self.start, *values))

    def status(self, msg):
        for status in msg.status:
            if status.name == "ywl_localization":
                self.record("status", msg.header.stamp.to_nsec(), status.message,
                            json.dumps({item.key: item.value for item in status.values}, ensure_ascii=False))

    def raw(self, msg):
        _, sec, nsec = struct.unpack_from("<III", msg._buff)
        self.record("raw", sec * 10**9 + nsec)

    def odom(self, msg, key):
        pos, rot = msg.pose.pose.position, msg.pose.pose.orientation
        self.record(key, msg.header.stamp.to_nsec(), *(pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w))

    def collect(self):
        args = self.args
        self.spawn("master", "roscore", "-p", str(args.port))
        master = rosgraph.Master("/test12_ywl_replay")
        self.wait(lambda: master.getPid() > 0, 15, "private ROS master not ready")
        master.setParam("/use_sim_time", True)
        rospy.init_node("test12_ywl_replay", disable_signals=True)
        self.subscribers = [
            rospy.Subscriber("/livox/lidar", rospy.AnyMsg, self.raw, buff_size=2**23),
            rospy.Subscriber("/fastlio/odom_world", Odometry, self.odom, callback_args="fastlio", tcp_nodelay=True),
            rospy.Subscriber("/ywl/bootstrap_odom", Odometry, self.odom, callback_args="bootstrap", tcp_nodelay=True),
            rospy.Subscriber("/ywl/odom", Odometry, self.odom, callback_args="ywl", tcp_nodelay=True),
            rospy.Subscriber(EV_TOPIC, PoseWithCovarianceStamped, self.odom, callback_args="ev", tcp_nodelay=True),
            rospy.Subscriber("/ywl/status", DiagnosticArray, self.status, tcp_nodelay=True),
            rospy.Subscriber("/ywl/initial_transform", TransformStamped,
                             lambda msg: self.record("init", msg.header.stamp.to_nsec())),
            rospy.Subscriber("/drone_reloc/result", String, lambda msg: self.record("reloc", 0, msg.data)),
        ]
        self.spawn("fastlio", "roslaunch", "fast_lio", "mapping_mid360s.launch")
        self.spawn("bridge", "roslaunch", "fastlio_gravity_bridge", "gravity_bridge.launch", "send_to_mavros:=true")
        self.spawn("ywl", "roslaunch", "ywl_localization", "ywl.launch",
                   f"map_dir:={args.prior}", f"legacy_map_dir:={args.source}", f"reloc_config:={args.config}")

        def input_subscribers_ready():
            subscribers = dict(master.getSystemState()[1])
            publishers = dict(master.getSystemState()[0])
            return (all({"/laserMapping", "/ywl_localization"} <= set(subscribers.get(topic, ()))
                        for topic in RAW_TOPICS) and publishers.get("/ywl/initial_transform") == ["/ywl_init_adapter"])

        self.wait(input_subscribers_ready, 90, "FAST-LIO2 or YWL input subscriptions not ready")
        if args.preload_relocalizer:
            self.start_relocalizer()
            self.wait(lambda: "/drone_reloc" in dict(master.getSystemState()[1]).get("/Odometry", ()),
                      30, "relocalizer map preload failed")
        durations = [f"--duration={args.seconds}"] if args.seconds else []
        player = self.spawn("bag", "rosbag", "play", "--clock", "--delay=2", "--quiet", *durations,
                            str(args.bag), "--topics", *RAW_TOPICS)
        self.wait(lambda: self.samples["fastlio"] and self.samples["bootstrap"], 30, "startup odometry missing")
        if not args.preload_relocalizer:
            self.start_relocalizer()
        pids = {name: ServerProxy(master.lookupNode(node)).getPid("/test12_ywl_replay")[2]
                for name, node in (("fastlio", "/laserMapping"), ("ywl", "/ywl_localization"))}
        while player.poll() is None:
            for name, pid in pids.items():
                stat = (Path("/proc") / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()
                cpu = (int(stat[11]) + int(stat[12])) / os.sysconf("SC_CLK_TCK")
                rss_mib = int(stat[21]) * os.sysconf("SC_PAGE_SIZE") / 2**20
                self.record("resources", 0, name, cpu, rss_mib)
            time.sleep(.5)
        if player.returncode != 0:
            raise RuntimeError("rosbag play failed")
        time.sleep(.8)
        self.publishers = dict(master.getSystemState()[0])
        if self.publishers.get(EV_TOPIC) != ["/gravity_bridge"]:
            raise RuntimeError("unexpected EV topic publishers")
        if set(self.publishers.get("/tf", [])) != {"/laserMapping", "/drone_reloc"}:
            raise RuntimeError("unexpected TF publishers")

    def save(self):
        for name, values in self.samples.items():
            with (self.args.out / f"{name}.csv").open("w") as file:
                csv.writer(file).writerows(values)

    def start_relocalizer(self):
        args = self.args
        self.spawn("reloc", "rosrun", "drone_relocalization", "relocalize",
                   "--map", str(args.source), "--config", str(args.config), "--save", str(args.out))

    def close(self):
        if rospy.core.is_initialized():
            rospy.signal_shutdown("isolated replay done")
        for child in reversed(self.children):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGTERM)


def summarize(samples, publishers, airborne):
    def summary(rows):
        stamps = np.array([x[0] for x in rows], dtype=np.int64)
        stamp_gap = np.diff(stamps) / 1e6
        wall_gap = np.diff([x[1] for x in rows]) * 1000
        metrics = {"frames": len(rows)}
        if len(stamp_gap):
            metrics.update(hz=float(1000 / stamp_gap.mean()),
                           stamp_gap_p95_ms=float(np.percentile(stamp_gap, 95)),
                           stamp_gap_max_ms=float(stamp_gap.max()),
                           wall_gap_p95_ms=float(np.percentile(wall_gap, 95)),
                           wall_gap_max_ms=float(wall_gap.max()))
        return metrics

    output = {key: summary(samples[key]) for key in ("raw", "fastlio", "ev", "ywl")}
    poses = {key: {row[0] for row in samples[key]} for key in ("fastlio", "ev", "ywl")}
    tracking = [json.loads(row[3]) for row in samples["status"] if row[2] == "TRACKING"]
    output["publishers"] = publishers
    output["same_stamp_ev"] = poses["fastlio"] == poses["ev"]
    output["inits"] = len(samples["init"])
    output["states"] = list(dict.fromkeys(row[2] for row in samples["status"]))
    origin = samples["raw"][0][0]
    if tracking:
        output["first_ywl_s"] = (samples["ywl"][0][0] - origin) / 1e9
        output["last_ywl_s"] = (samples["ywl"][-1][0] - origin) / 1e9
        if airborne:
            common = poses["fastlio"] & poses["ywl"]
            begin, end = airborne
            output["airborne_interval_s"] = list(airborne)
            output["airborne_common_frames"] = sum(begin <= (stamp - origin) / 1e9 < end for stamp in common)
        output["ywl_processing_p99_ms"] = float(np.percentile([float(s["processing_ms"]) for s in tracking], 99))
        output["ywl_max_queue_depth"] = max(int(s["queue_depth"]) for s in tracking)
    for name in ("fastlio", "ywl"):
        resources = [row for row in samples["resources"] if row[2] == name]
        if len(resources) > 1:
            output[name]["peak_rss_mib"] = max(row[4] for row in resources)
            output[name]["cpu_one_core_pct"] = 100 * (
                resources[-1][3] - resources[0][3]) / (resources[-1][1] - resources[0][1])
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("bag", "source", "prior", "config", "out"):
        parser.add_argument(f"--{name}", type=Path, required=True)
    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--seconds", type=float)
    parser.add_argument("--airborne", type=float, nargs=2, metavar=("START_S", "END_S"),
                        help="airborne interval relative to the first LiDAR timestamp")
    parser.add_argument("--preload-relocalizer", action="store_true",
                        help="load the reference map before replay when the recorded start stillness is short")
    args = parser.parse_args()
    if args.port == 11311:
        parser.error("choose an isolated master port")
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", args.port))
    args.out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_IP="127.0.0.1",
                      ROS_HOSTNAME="127.0.0.1", ROS_HOME=str(args.out / "ros_home"))
    replay = Replay(args)
    try:
        replay.collect()
        summary = summarize(replay.samples, replay.publishers, args.airborne)
        summary["preloaded_relocalizer"] = args.preload_relocalizer
        (args.out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
        print(json.dumps(summary, indent=2))
        if summary["inits"] != 1 or summary["ywl"]["frames"] == 0 or not summary["same_stamp_ev"]:
            raise RuntimeError("YWL initialization or FAST-LIO2 external vision failed")
    finally:
        replay.close()
        replay.save()


if __name__ == "__main__":
    main()
