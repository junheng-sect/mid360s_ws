#!/usr/bin/env python3
"""Isolated CH6 -> light + real C++ raw bag test, no MAVROS or hardware.
Run on the board after sourcing ROS Noetic and mid360s_ws/devel/setup.bash.
"""
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import tempfile
import time


def main():
    root = Path(tempfile.mkdtemp(prefix='ch6_raw_e2e_'))
    bags = root / 'bags with spaces'
    run = root / 'run'
    bags.mkdir()
    run.mkdir()
    here = Path(__file__).resolve().parent
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % port
    os.environ['ROS_IP'] = '127.0.0.1'
    os.environ.pop('ROS_HOSTNAME', None)
    os.environ['ROS_HOME'] = str(root / 'ros_home')
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    processes = []
    logs = []

    def launch(command, name):
        log = (root / (name + '.log')).open('w')
        logs.append(log)
        child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        processes.append(child)
        return child

    def until(predicate, seconds=10):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.05)
        raise AssertionError('timed out: %s' % predicate)

    try:
        core = launch(['roscore', '-p', str(port)], 'roscore')
        def online():
            try:
                with socket.create_connection(('127.0.0.1', port), timeout=.1):
                    return True
            except OSError:
                return False
        until(online)
        import rospy
        import rosbag
        import rosgraph
        from mavros_msgs.msg import RCIn
        from livox_ros_driver2.msg import CustomMsg, CustomPoint
        from geometry_msgs.msg import Vector3Stamped, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu
        from tf2_msgs.msg import TFMessage
        rospy.init_node('ch6_raw_e2e', anonymous=True, disable_signals=True)
        rospy.set_param('/rc_bag_recorder', {
            'bag_directory': str(bags), 'bag_prefix': 'test_rc',
            'rc_topic': '/test/rc', 'rc_channel': 6,
            'confirm_samples': 3, 'release_tail_s': 1.0,
            'record_raw_cloud': False, 'topics': ['/Odometry', '/livox/imu'],
            'latched_topics': [], 'status_rate': 2.0})
        light = launch(['python3', '-u', str(here / 'bag_recorder_node.py')], 'light')
        raw = launch(['python3', '-u', str(here / 'raw_bag_recorder.py'),
                      '--run-dir', str(run), '--bag-dir', str(bags),
                      '--prefix', 'test'], 'raw')
        pubs = {
            'rc': rospy.Publisher('/test/rc', RCIn, queue_size=10),
            'cloud': rospy.Publisher('/livox/lidar', CustomMsg, queue_size=10),
            'imu': rospy.Publisher('/livox/imu', Imu, queue_size=10),
            'odom': rospy.Publisher('/Odometry', Odometry, queue_size=10),
            'gravity': rospy.Publisher('/fastlio/gravity_init', Vector3Stamped,
                                       queue_size=1, latch=True),
            'tf': rospy.Publisher('/tf_static', TFMessage, queue_size=1, latch=True)}
        until(lambda: (run / 'raw.ready').exists() and pubs['rc'].get_num_connections() > 0)
        gravity = Vector3Stamped()
        gravity.header.frame_id = 'camera_init'
        gravity.vector.z = -9.81
        pubs['gravity'].publish(gravity)
        transform = TransformStamped()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'camera_init'
        transform.transform.rotation.w = 1
        pubs['tf'].publish(TFMessage(transforms=[transform]))

        def pump(pwm, duration):
            end = time.monotonic() + duration
            while time.monotonic() < end:
                stamp = rospy.Time.now()
                rc = RCIn()
                rc.channels = [1500] * 8
                rc.channels[5] = pwm
                pubs['rc'].publish(rc)
                cloud = CustomMsg()
                cloud.header.stamp = stamp
                cloud.header.frame_id = 'livox_frame'
                cloud.timebase = stamp.to_nsec()
                cloud.point_num = 1
                point = CustomPoint()
                point.x = 1.0
                cloud.points = [point]
                pubs['cloud'].publish(cloud)
                imu = Imu()
                imu.header.stamp = stamp
                imu.linear_acceleration.z = 9.81
                pubs['imu'].publish(imu)
                odom = Odometry()
                odom.header.stamp = stamp
                odom.pose.pose.orientation.w = 1
                pubs['odom'].publish(odom)
                time.sleep(.05)

        # Startup with CH6 already high must not create a partial recording.
        pump(2000, 1.0)
        assert not list(bags.glob('*.bag*')), list(bags.iterdir())
        pump(1000, .6)
        pump(2000, 2)
        assert len(list(bags.glob('test_raw_*.active'))) == 1
        subscribers = dict(rosgraph.Master('/ch6_raw_e2e').getSystemState()[1])
        raw_subs = subscribers.get('/livox/lidar', [])
        assert len(raw_subs) == 1 and raw_subs[0].startswith('/record_'), raw_subs
        assert '/rc_bag_recorder' not in raw_subs
        # Release then re-trigger inside the 1 s tail: retain the same pair.
        pump(1000, .3)
        pump(2000, 1)
        assert len(list(bags.glob('test_rc_*.bag'))) == 1
        pump(1000, 2)
        until(lambda: len(list(bags.glob('test_raw_*.bag'))) == 1)
        assert not list(bags.glob('*.active'))
        sizes = {p: p.stat().st_size for p in bags.glob('*.bag')}
        pump(1000, .6)
        assert sizes == {p: p.stat().st_size for p in bags.glob('*.bag')}
        print('[PASS] initial high blocked; CH6 start/tail/retrigger/stop; idle does not record', flush=True)
        pump(2000, 2)
        pump(1000, 2)
        until(lambda: len(list(bags.glob('test_raw_*.bag'))) == 2)
        # Terminate the controller while CH6 remains on: the C++ bag must close.
        pump(2000, 1.5)
        raw.send_signal(signal.SIGTERM)
        assert raw.wait(timeout=35) == 0, (root / 'raw.log').read_text()
        pump(1000, 1.6)
        light.send_signal(signal.SIGINT)
        assert light.wait(timeout=10) == 0
        summary = json.loads((run / 'raw.summary.json').read_text())
        assert not summary['errors'], summary
        assert len(summary['paths']) == 3, summary
        assert not list(bags.glob('*.active'))
        for index in range(1, 4):
            stem = '%06d.bag' % index
            with rosbag.Bag(str(bags / ('test_raw_' + stem))) as bag:
                counts = {name: value.message_count for name, value in
                          bag.get_type_and_topic_info().topics.items()}
                assert counts.get('/livox/lidar', 0) >= 5, counts
                assert counts.get('/livox/imu', 0) >= 5, counts
                assert counts.get('/Odometry', 0) >= 5, counts
                assert counts.get('/tf_static', 0) >= 1, counts
                assert counts.get('/fastlio/gravity_init', 0) >= 1, counts
            with rosbag.Bag(str(bags / ('test_rc_' + stem))) as bag:
                assert '/livox/lidar' not in bag.get_type_and_topic_info().topics
        print('[PASS] three paired indexed bags, all five raw topics and latched metadata; SIGTERM closes active bag', flush=True)
        print('EVIDENCE', root, flush=True)
    finally:
        for child in reversed(processes):
            if child.poll() is None:
                os.killpg(child.pid, signal.SIGINT)
                try:
                    child.wait(timeout=35)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait(timeout=5)
        for log in logs:
            log.close()
        print('test artifacts:', root, flush=True)


if __name__ == '__main__':
    main()
