#!/usr/bin/env python3
"""Read-only ROS readiness probes for test12."""
import argparse
import json
import math
import sys
import time

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import ExtendedState, State
from nav_msgs.msg import Odometry
import rosgraph
import rospy
from std_msgs.msg import String


def fresh(message, seconds=2.5):
    if message is None or message.header.stamp.is_zero():
        return False
    return -0.05 <= (rospy.Time.now() - message.header.stamp).to_sec() <= seconds


def valid_pose(pose):
    p, q = pose.position, pose.orientation
    values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
    return all(map(math.isfinite, values)) and abs(sum(x*x for x in values[3:]) - 1) < 0.02


def decode(message):
    try:
        result = json.loads(message.data) if message else {}
        return result if isinstance(result, dict) else {}
    except ValueError:
        return {}


def check(stage, map_dir):
    if stage == "recorder":
        master = rosgraph.Master("/test12_recorder_probe")
        return any(name.startswith("/outdoor_dll_ywl_record_")
                   for name in dict(master.getSystemState()[1]).get("/mavros/rc/in", []))

    rospy.init_node("test12_state", anonymous=True, disable_signals=True)
    samples = {}

    def receive(message, topic):
        samples[topic] = message

    topics = {
        "ground": [("/mavros/state", State), ("/mavros/extended_state", ExtendedState)],
        "fastlio": [("/fastlio/odom_world", Odometry)],
        "reloc": [("/drone_reloc/result", String), ("/drone_reloc/status", String),
                  ("/drone_reloc/pose", PoseStamped)],
    }[stage]
    subscribers = [rospy.Subscriber(name, kind, receive, callback_args=name, queue_size=1)
                   for name, kind in topics]

    def ready():
        if stage == "ground":
            state = samples.get("/mavros/state")
            ext = samples.get("/mavros/extended_state")
            return (fresh(state) and state.connected and not state.armed and state.mode != "OFFBOARD"
                    and fresh(ext) and ext.landed_state == ExtendedState.LANDED_STATE_ON_GROUND)
        if stage == "fastlio":
            odom = samples.get("/fastlio/odom_world")
            return (fresh(odom, 0.5) and odom.header.frame_id == "drone_odom"
                    and odom.child_frame_id == "drone_base" and valid_pose(odom.pose.pose))
        result = decode(samples.get("/drone_reloc/result"))
        status = decode(samples.get("/drone_reloc/status"))
        pose = samples.get("/drone_reloc/pose")
        stamp = status.get("stamp")
        return (result.get("confirmed") is True and result.get("map") == map_dir
                and status.get("valid") is True and status.get("state") == "LOCALIZED"
                and isinstance(stamp, (int, float)) and math.isfinite(stamp)
                and -0.05 <= rospy.Time.now().to_sec() - stamp <= 2.5
                and fresh(pose, 0.5) and pose.header.frame_id == "drone_map" and valid_pose(pose.pose))

    try:
        deadline = time.monotonic() + (4 if stage == "ground" else 2)
        while time.monotonic() < deadline:
            if ready():
                return True
            time.sleep(0.05)
        return False
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("stage", choices=("ground", "fastlio", "reloc", "recorder"))
    parser.add_argument("map_dir", nargs="?")
    args = parser.parse_args()
    if not check(args.stage, args.map_dir):
        if args.stage == "ground":
            print("飞控尚未确认已连接、已着地且已上锁", file=sys.stderr)
        raise SystemExit(1)
