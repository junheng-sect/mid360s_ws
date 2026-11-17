#!/usr/bin/env python3
"""Exercise faults against the built ROS node on its own master."""
import argparse
from copy import deepcopy
import csv
import importlib.machinery
import importlib.util
import json
from pathlib import Path
import shutil
import time

import rosbag
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg
from sensor_msgs.msg import Imu

PACKAGE = Path(__file__).resolve().parents[1]
loader = importlib.machinery.SourceFileLoader("ywl_replay", str(PACKAGE / "scripts/ywl_replay"))
spec = importlib.util.spec_from_loader(loader.name, loader)
runner = importlib.util.module_from_spec(spec)
loader.exec_module(runner)


def duplicate_and_no_match(args, replay):
    anchor = json.loads(args.seed.read_text())["anchor_ns"]
    path = args.out / "no_match.bag"
    shifted = False
    with rosbag.Bag(str(args.bag)) as original, rosbag.Bag(str(path), "w") as fault:
        until = rospy.Time.from_sec(original.get_start_time() + 9)
        for topic, msg, stamp in original.read_messages(topics=runner.RAW, end_time=until):
            if topic == runner.RAW[0] and not shifted and msg.header.stamp.to_nsec() > anchor + 1_600_000_000:
                for point in msg.points:
                    point.x += 1000
                shifted = True
            fault.write(topic, msg, stamp)
    if not shifted:
        raise RuntimeError("no LiDAR scan available for no-match fault")

    diagnostics, initial = [], []
    status_sub = rospy.Subscriber("/ywl/status", DiagnosticArray,
                                  lambda m: diagnostics.extend((m.header.stamp.to_nsec(), x)
                                                               for x in m.status), queue_size=40)
    init_sub = rospy.Subscriber("/ywl/initial_transform", TransformStamped, initial.append, queue_size=2)
    duplicate = rospy.Publisher("/ywl/initial_transform", TransformStamped, queue_size=1)
    replay.spawn("tracking_fault", "roslaunch", "ywl_localization", "ywl.launch",
                          f"map_dir:={args.map}", f"seed_file:={args.seed}")
    replay.wait_for(lambda: duplicate.get_num_connections() > 1 and
                    any(x.message == "WAITING_INIT" for _, x in diagnostics), 10, "tracking test not ready")
    replay.spawn("fault_bag", "rosbag", "play", "--quiet", "--delay=1", str(path))
    replay.wait_for(lambda: initial and any(x.message == "TRACKING" for _, x in diagnostics),
                    15, "tracking test did not initialize")
    last_tracking = max(t for t, x in diagnostics if x.message == "TRACKING")
    msg = deepcopy(initial[0])
    msg.transform.translation.x += 10
    msg.header.stamp = rospy.Time(last_tracking // 10**9, last_tracking % 10**9)
    duplicate.publish(msg)
    # Keep polling after the player ends so a delayed diagnostic is still observed.
    deadline = time.monotonic() + 8
    while not any(x.message == "FAILED" for _, x in diagnostics) and time.monotonic() < deadline:
        time.sleep(0.04)
    later = [t for t, x in diagnostics if x.message == "TRACKING" and t > last_tracking]
    if len(set(later)) < 5:
        raise AssertionError("tracking stopped too early after a duplicate alignment")
    failures = [x for _, x in diagnostics if x.message == "FAILED"]
    if not failures or not any("no valid point-to-plane match" in v.value for v in failures[0].values):
        raise AssertionError("unmatchable scan did not invalidate localization")
    status_sub.unregister()
    init_sub.unregister()
    duplicate.unregister()
    return {"duplicate_initial_transform": "tracking continued after a conflicting duplicate",
            "no_valid_match": "unmatchable LiDAR scan triggered FAILED"}


def run_faults(args):
    import os
    args.out.mkdir(parents=True)
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_HOSTNAME="127.0.0.1",
                      ROS_IP="127.0.0.1", ROS_HOME=str(args.out / "ros_home"))
    replay = runner.Replay(args)
    results = {}
    try:
        replay.spawn("master", "roscore", "-p", str(args.port))
        master = rosgraph.Master("/ywl_fault_test")
        replay.wait_for(lambda: master.getPid() > 0, 10, "master unavailable")
        rospy.init_node("ywl_fault_test", disable_signals=True)

        def run_case(name, map_path, inject, expected):
            events = []
            sub = rospy.Subscriber("/ywl/status", DiagnosticArray,
                                   lambda m: events.extend(m.status), queue_size=10)
            master.setParam("/ywl_fault_node", {
                "map_dir": str(map_path), "extrinsic_T": [-0.011, -0.02329, 0.04412],
                "extrinsic_R": [1., 0., 0., 0., 1., 0., 0., 0., 1.],
                "time_offset_lidar_to_imu": 0.,
            })
            executable = replay.spawn(name, "rosrun", "ywl_localization", "ywl_localization_node",
                                      "__name:=ywl_fault_node")
            try:
                if inject:
                    replay.wait_for(lambda: any(e.message == "WAITING_INIT" for e in events),
                                    10, "node did not load test map")
                    inject(master, replay)
                replay.wait_for(lambda: any(e.message == "FAILED" for e in events), 10,
                                f"{name} did not become FAILED")
                failure = next(e for e in events if e.message == "FAILED")
                reason = next(v.value for v in failure.values if v.key == "reason")
                if expected not in reason:
                    raise AssertionError(f"{name}: unexpected reason {reason}")
                results[name] = reason
            finally:
                sub.unregister()
                executable.terminate()
                executable.wait(timeout=10)
                replay.launches.remove(executable)

        def reverse(_, replay):
            pub = rospy.Publisher("/livox/lidar", CustomMsg, queue_size=3)
            imu = rospy.Publisher("/livox/imu", Imu, queue_size=3)
            replay.wait_for(lambda: pub.get_num_connections() > 0 and imu.get_num_connections() > 0,
                            5, "raw input subscriptions not ready")
            preroll = Imu()
            preroll.header.stamp = rospy.Time(98)
            preroll.linear_acceleration.z = 9.81
            imu.publish(preroll)
            rospy.sleep(0.04)
            for sec in (100, 99):
                msg = CustomMsg()
                msg.header.stamp = rospy.Time(sec)
                pub.publish(msg)
                rospy.sleep(0.04)
            pub.unregister()
            imu.unregister()

        bad = args.out / "changed_map"
        bad.mkdir()
        for name in ("meta.json", "map_world.pcd"):
            shutil.copyfile(args.map / name, bad / name)
        run_case("time_reversal", args.map, reverse, "timestamp repeated or reversed")
        run_case("extrinsic_edit", args.map,
                 lambda master, _: master.setParam("/ywl_fault_node/extrinsic_T", [0., 0., 0.]),
                 "calibration parameter changed")
        run_case("map_edit", bad,
                 lambda *_: (bad / "meta.json").write_text("{}"), "prior map files changed")
        run_case("invalid_map", bad, None, "points_world")

        seed = json.loads(args.seed.read_text())
        seed["map_sha256"]["map_world.pcd"] = "0" * 64
        wrong = args.out / "wrong_seed.json"
        wrong.write_text(json.dumps(seed))
        adapter = replay.spawn("wrong_seed", "rosrun", "ywl_localization", "ywl_init_adapter",
                               f"_map_dir:={args.map}", f"_seed_file:={wrong}")
        if adapter.wait(timeout=10) != 2 or "map identity mismatch" not in (args.out / "wrong_seed.log").read_text():
            raise AssertionError("adapter did not reject the mismatched seed")
        replay.launches.remove(adapter)
        results["wrong_seed"] = "seed map digest mismatch rejected"
        results.update(duplicate_and_no_match(args, replay))
    finally:
        replay.close()
    return results


def analyze_replay(args):
    """Checks from the full run, using recorded publisher graph and messages."""
    def rows(name):
        with (args.ywl / name).open() as stream:
            return list(csv.DictReader(stream))
    pose, velocity, initial = rows("pose.csv"), rows("velocity.csv"), rows("initial_transform.csv")
    assert len(initial) == 1 and len(pose) > 0, "expected exactly one handoff and valid poses"
    assert [r["stamp_ns"] for r in pose] == [r["stamp_ns"] for r in velocity], "pose/velocity stamps"
    messages = [json.loads(line) for line in (args.ywl / "diagnostics.jsonl").read_text().splitlines()]
    tracks = [m for m in messages if m["state"] == "TRACKING"]
    assert len({m["map_points"] for m in tracks}) == 1, "prior point count changed"
    terminal = next(m for m in messages if m["state"] in ("STALE", "FAILED"))
    assert terminal["state"] == "STALE" and float(pose[-1]["wall_s"]) < terminal["wall_s"], "watchdog"
    run = json.loads((args.ywl / "run.json").read_text())
    assert run["invalid_pose_covariances"] == 0, "invalid YWL covariance"
    return {"valid_covariance_count": len(pose), "fixed_map_points": int(tracks[0]["map_points"]),
            "velocity_stamps_match": True, "initial_transforms": len(initial),
            "post_input_state": terminal["state"], "output_stops_on_timeout": True}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--ywl", type=Path, required=True)
    parser.add_argument("--seed", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--port", type=int, required=True)
    args = parser.parse_args()
    results = analyze_replay(args)
    results.update(run_faults(args))
    (args.out / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    print(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
