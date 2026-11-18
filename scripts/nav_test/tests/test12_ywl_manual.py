#!/usr/bin/env python3
"""Exercise the manual shell supervisor and native Channel 6 recorder on a private master."""
import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time

import rosbag
import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, TransformStamped, Vector3Stamped
from mavros_msgs.msg import ExtendedState, RCIn, State
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, String


def fixture(args):
    fixtures = args.out / "fixture"
    workspace = fixtures / "workspace"
    fixtures.mkdir(parents=True)
    workspace.mkdir()
    (workspace / "devel").symlink_to(args.devel)
    scripts = workspace / "scripts"
    scripts.mkdir()
    repo = args.source.resolve()
    for relative in ("nav_test12.sh", "nav_test/common.sh", "nav_test/test12_state.py",
                     "rc_record/record_outdoor_dll_ywl", "rc_record/record_outdoor_rc.py",
                     "rc_record/rc_record_policy.py"):
        destination = scripts / relative
        destination.parent.mkdir(exist_ok=True)
        shutil.copy2(repo / "scripts" / relative, destination)
    # Keep fixture locks, HOME, and ROS transport independent of live flight processes.
    supervisor = scripts / "nav_test12.sh"
    data = supervisor.read_text().replace("/tmp/mid360s-nav-test-", str(fixtures / "nav-lock-"))
    data = data.replace("/tmp/mid360s_fusion.lock", str(fixtures / "fusion.lock"))
    supervisor.write_text(data)
    (scripts / "ros_net.sh").write_text('export PATH="$FIXTURE/bin:$PATH"\n')
    home = fixtures / "home"
    map_root = home / "test_data/mapping"
    source_map = map_root / "20261105_114605_outdoor"
    for directory in (source_map, source_map.with_name(source_map.name + "_screenshot_roi")):
        directory.mkdir(parents=True)
        for filename in ("map_world.pcd", "meta.json", "manifest.json"):
            (directory / filename).write_text("test fixture only\n")
    config = workspace / "src/drone_relocalization/config"
    config.mkdir(parents=True)
    (config / "outdoor.yaml").write_text("{}\n")
    subprocess.run(["git", "init", "-q", str(workspace)], check=True)
    subprocess.run(["git", "-C", str(workspace), "-c", "user.name=Test Fixture",
                    "-c", "user.email=fixture@example.com", "commit", "--allow-empty",
                    "-qm", "Fixture"], check=True)
    bins = fixtures / "bin"
    bins.mkdir()
    worker = """#!/usr/bin/env python3
import os
from pathlib import Path
import signal
import sys
import time

for sig in (signal.SIGINT, signal.SIGTERM):
    signal.signal(sig, lambda *_: sys.exit(0))
if '--help' in sys.argv:
    sys.exit(0)
name = Path(sys.argv[0]).name
Path(os.environ["FIXTURE"] + "/" + name + ".ready").touch()
while True:
    time.sleep(.1)
"""
    for path in (bins / "roslaunch", bins / "rosrun", scripts / "fusion.sh"):
        path.write_text(worker)
        path.chmod(0o755)
    return supervisor, fixtures, home, bins, source_map


class Telemetry:
    def __init__(self, fixtures, source_map, mode):
        self.fixtures, self.source_map = fixtures, source_map
        self.mode = mode
        self.armed = False
        self.rc_high = False
        self.ywl_failed = False
        self.pubs = {topic: rospy.Publisher(topic, kind, queue_size=2) for topic, kind in (
            ("/mavros/state", State), ("/mavros/extended_state", ExtendedState), ("/mavros/rc/in", RCIn),
            ("/fastlio/odom_world", Odometry), ("/drone_reloc/result", String), ("/drone_reloc/status", String),
            ("/drone_reloc/pose", PoseStamped), ("/ywl/bootstrap_odom", Odometry), ("/ywl/odom", Odometry),
            ("/ywl/velocity", Vector3Stamped), ("/ywl/status", DiagnosticArray),
            ("/dll_localization/pose", PoseStamped), ("/dll_localization/correction", TransformStamped),
            ("/dll_localization/status", DiagnosticArray))}
        self.timer = rospy.Timer(rospy.Duration(.05), self.publish)

    def publish(self, _):
        stamp = rospy.Time.now()
        header = Header(stamp=stamp)
        send = lambda topic, msg: self.pubs[topic].publish(msg)
        send("/mavros/state", State(header=header, connected=True, armed=self.armed, mode="POSCTL"))
        send("/mavros/extended_state", ExtendedState(header=header, landed_state=2 if self.armed else 1))
        send("/mavros/rc/in", RCIn(header=header, channels=[1500] * 5 + [1800 if self.rc_high else 1300]))
        if (self.fixtures / "fusion.sh.ready").exists():
            odom = Odometry(header=Header(stamp=stamp, frame_id="drone_odom"), child_frame_id="drone_base")
            odom.pose.pose.orientation.w = 1
            send("/fastlio/odom_world", odom)
        observer_started = (self.fixtures / "roslaunch.ready").exists()
        ready = (self.fixtures / "rosrun.ready").exists()
        mapped = Odometry(header=Header(stamp=stamp, frame_id="drone_map"))
        mapped.pose.pose.orientation.w = 1
        if self.mode == "ywl" and observer_started:
            frame = "drone_map" if ready else "ywl_odom"
            ywl = Odometry(header=Header(stamp=stamp, frame_id=frame), child_frame_id="ywl_imu", pose=mapped.pose)
            state = "FAILED" if self.ywl_failed else "TRACKING" if ready else "WAITING_INIT"
            if not self.ywl_failed:
                send("/ywl/odom" if ready else "/ywl/bootstrap_odom", ywl)
                if ready:
                    send("/ywl/velocity", Vector3Stamped(header=ywl.header))
            send("/ywl/status", DiagnosticArray(header=header, status=[
                DiagnosticStatus(name="ywl_localization", message=state,
                                 level={"TRACKING": 0, "WAITING_INIT": 1, "FAILED": 2}[state])]))
        if ready:
            send("/drone_reloc/result", String(data='{"confirmed":true,"map":"%s"}' % self.source_map))
            send("/drone_reloc/status", String(data='{"valid":true,"state":"LOCALIZED","stamp":%s}' % stamp.to_sec()))
            pose = PoseStamped(header=mapped.header, pose=mapped.pose.pose)
            send("/drone_reloc/pose", pose)
            if self.mode == "dll" and observer_started:
                send("/dll_localization/pose", pose)
                send("/dll_localization/correction", TransformStamped(header=mapped.header, child_frame_id="drone_odom"))
                send("/dll_localization/status", DiagnosticArray(header=header, status=[
                    DiagnosticStatus(name="fastlio_dll", message="TRACKING", level=0, values=[
                        KeyValue(key="accepted_stamp_ns", value=str(stamp.to_nsec())),
                        KeyValue(key="local_input", value="fresh")])]))


def wait(condition, description, seconds=25):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        if condition():
            return
        time.sleep(.1)
    raise RuntimeError(description)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--devel", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--port", type=int, required=True, choices=range(11600, 11700), metavar="116xx")
    parser.add_argument("--mode", choices=("ywl", "dll", "original"), default="ywl")
    args = parser.parse_args()
    supervisor, fixtures, home, bins, source_map = fixture(args)
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{args.port}", ROS_HOSTNAME="127.0.0.1",
                      ROS_IP="127.0.0.1", ROS_HOME=str(args.out / "ros_home"))
    master = subprocess.Popen(["roscore", "-p", str(args.port)], start_new_session=True,
                              stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
    runner = None
    output = args.out / "supervisor.log"
    try:
        def ready():
            try:
                return master.poll() is None and rosgraph.Master("/fixture").getPid() > 0
            except OSError:
                return False
        wait(ready, "isolated master did not start")
        rospy.init_node("manual_flight_fixture", disable_signals=True)
        rospy.set_param("/ywl_localization/map_dir", str(source_map))
        rospy.set_param("/fastlio_dll/map_dir", str(source_map))
        telemetry = Telemetry(fixtures, source_map, args.mode)
        env = dict(os.environ, HOME=str(home), FIXTURE=str(fixtures), PATH=f"{bins}:{os.environ['PATH']}",
                   WAREHOUSE_TASK_ID="1" * 32, TMUX="fixture")
        with output.open("w") as stream:
            flags = {"ywl": ["--ywl"], "dll": [], "original": ["--no-dll"]}[args.mode]
            runner = subprocess.Popen([str(supervisor), *flags], env=env, start_new_session=True,
                                      stdout=stream, stderr=subprocess.STDOUT)
        contains = lambda text: text in output.read_text()
        wait(lambda: contains("静止录够 20 秒"), "test12 supervisor did not become ready")
        telemetry.rc_high = True
        wait(lambda: contains("Channel 6 录包中"), "Channel 6 did not open bag")
        # A native .bag.active starts before TCPROS connects its subscribers.
        time.sleep(2)
        if args.mode == "ywl":
            telemetry.ywl_failed = True
            probe = supervisor.parent / "nav_test/test12_state.py"
            result = subprocess.run(["python3", "-B", str(probe), "ywl"], timeout=8)
            assert result.returncode == 1 and runner.poll() is None and list(home.rglob("*.bag.active"))
        telemetry.armed = True
        time.sleep(.35)
        runner.send_signal(signal.SIGTERM)
        wait(lambda: contains("收尾被拒"), "airborne stop did not refuse")
        assert runner.poll() is None and list(home.rglob("*.bag.active"))
        telemetry.armed = telemetry.rc_high = False
        wait(lambda: contains("已封包"), "Channel 6 did not seal bag")
        runner.send_signal(signal.SIGTERM)
        assert runner.wait(timeout=35) == 0 and contains("网页任务已收尾")
        assert not list(home.rglob("*.bag.active"))
        bag_path, = home.rglob("*.bag")
        with rosbag.Bag(str(bag_path)) as bag:
            topics = set(bag.get_type_and_topic_info().topics)
            expected = {"/mavros/state", "/fastlio/odom_world"}
            if args.mode == "ywl":
                expected.update(("/ywl/odom", "/ywl/velocity", "/ywl/status"))
            if args.mode == "dll":
                expected.update(("/dll_localization/pose", "/dll_localization/correction", "/dll_localization/status"))
            assert expected <= topics, f"missing recorded topics: {sorted(expected - topics)}"
        print(f"PASS: {args.mode} readiness, Channel 6, airborne refusal, grounded shutdown. {bag_path}")
    finally:
        if runner is not None and runner.poll() is None:
            telemetry.armed = telemetry.rc_high = False
            time.sleep(.2)
            runner.send_signal(signal.SIGTERM)
            try:
                runner.wait(timeout=35)
            except subprocess.TimeoutExpired:
                runner.kill()
        os.killpg(master.pid, signal.SIGINT)
        master.wait(timeout=15)


if __name__ == "__main__":
    main()
