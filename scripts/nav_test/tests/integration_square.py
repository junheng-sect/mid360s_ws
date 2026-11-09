#!/usr/bin/env python3
"""隔离 ROS 消息/action 回归；只用假状态，不起导航、不发布飞控设定值。

ROS_MASTER_URI 必须指向单独启动的 localhost:11435，绝不可用板端常驻 master。
用法：source 两个工作空间后，在隔离 master 下运行 integration_square.py [nav_square.py]
"""
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time

import actionlib
import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
from mavros_msgs.msg import State, EstimatorStatus
from traj_utils.srv import CheckTrajectory, CheckTrajectoryResponse
from drone_navigation.msg import ExecuteRouteAction, ExecuteRouteResult

assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11435', 'requires isolated master 11435'


class Fixture:
    def __init__(self):
        self.offboard = True
        self.prepared = True
        self.gw, self.reason, self.executor = 'READY', '', 'IDLE'
        self.position = np.array([.2, -.1, 1.])
        self.yaw, self.map_yaw = .37, .6
        self.r = np.array([[math.cos(.6), -math.sin(.6), 0.], [math.sin(.6), math.cos(.6), 0.], [0., 0., 1.]])
        self.translation = np.array([5., -3., .4])
        self.goals, self.prepares = [], []
        self.stop = False
        self.pubs = {}
        for name, typ in (('/drone_nav/odom', Odometry), ('/mavros/state', State), ('/mavros/estimator_status', EstimatorStatus)):
            self.pubs[name] = rospy.Publisher(name, typ, queue_size=3)
        for name in ('/drone_nav/input_status', '/drone_nav/gateway_status', '/drone_nav/executor_status', '/drone_reloc/status', '/drone_reloc/result'):
            self.pubs[name] = rospy.Publisher(name, String, queue_size=3, latch=name.endswith('result'))
        self.tf = tf2_ros.TransformBroadcaster()
        self.prepare_service = rospy.Service('/drone_nav/prepare', Trigger, self.prepare)
        self.check_service = rospy.Service('/drone_nav/check_trajectory', CheckTrajectory, lambda _: CheckTrajectoryResponse(valid=True, reason='synthetic'))
        self.server = actionlib.SimpleActionServer('/drone_nav/execute_route', ExecuteRouteAction, execute_cb=self.execute, auto_start=False)
        self.server.start()
        self.thread = threading.Thread(target=self.publish, daemon=True)
        self.thread.start()

    def prepare(self, _):
        self.prepares.append(time.monotonic())
        self.prepared = True
        self.gw, self.reason = ('READY' if self.offboard else 'PREPARED'), ''
        return TriggerResponse(success=True, message='prepared')

    def execute(self, goal):
        if not self.offboard or self.gw != 'READY':
            self.server.set_aborted(ExecuteRouteResult(success=False, reason='not ready'))
            return
        self.goals.append((time.monotonic(), goal))
        self.executor = 'RUNNING'
        until = time.monotonic() + .3
        while time.monotonic() < until:
            if self.server.is_preempt_requested() or not self.offboard:
                self.executor = 'IDLE'
                self.server.set_preempted(ExecuteRouteResult(success=False, reason='cancelled'))
                return
            time.sleep(.01)
        pose = goal.waypoints[0].pose
        self.position = self.r.T @ (np.array([pose.position.x, pose.position.y, pose.position.z]) - self.translation)
        self.yaw = math.atan2(2 * pose.orientation.z * pose.orientation.w, 1 - 2 * pose.orientation.z ** 2) - self.map_yaw
        self.gw, self.reason, self.executor = 'FAULT', 'task stopped; prepare before next route', 'IDLE'
        self.server.set_succeeded(ExecuteRouteResult(success=True, reason='all waypoints settled'))

    def mode(self, offboard):
        self.offboard = offboard
        if offboard and self.prepared:
            self.gw, self.reason = 'READY', ''
        elif not offboard:
            self.prepared = False
            self.gw, self.reason = 'FAULT', 'OFFBOARD exited; explicit prepare/start required'

    def publish(self):
        while not self.stop and not rospy.is_shutdown():
            stamp = rospy.Time.now()
            o = Odometry()
            o.header.stamp, o.header.frame_id = stamp, 'drone_odom'
            o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z = self.position
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
            self.pubs['/drone_nav/odom'].publish(o)
            s = State(connected=True, armed=True, mode='OFFBOARD' if self.offboard else 'POSCTL')
            s.header.stamp = stamp
            self.pubs['/mavros/state'].publish(s)
            e = EstimatorStatus()
            e.header.stamp = stamp
            for k in ('attitude_status_flag', 'velocity_horiz_status_flag', 'velocity_vert_status_flag', 'pos_horiz_rel_status_flag', 'pos_vert_abs_status_flag'):
                setattr(e, k, True)
            self.pubs['/mavros/estimator_status'].publish(e)
            for name, data in (
                ('/drone_nav/input_status', dict(state='READY')),
                ('/drone_nav/gateway_status', dict(state=self.gw, reason=self.reason, real_output=True)),
                ('/drone_nav/executor_status', dict(state=self.executor, reason='')),
                ('/drone_reloc/status', dict(state='LOCALIZED', valid=True)),
                ('/drone_reloc/result', dict(confirmed=True, map='synthetic'))):
                self.pubs[name].publish(String(json.dumps(data)))
            tr = TransformStamped()
            tr.header.stamp, tr.header.frame_id, tr.child_frame_id = stamp, 'drone_map', 'drone_odom'
            tr.transform.translation.x, tr.transform.translation.y, tr.transform.translation.z = self.translation
            tr.transform.rotation.z, tr.transform.rotation.w = math.sin(self.map_yaw / 2), math.cos(self.map_yaw / 2)
            self.tf.sendTransform(tr)
            time.sleep(.025)


def wait(predicate, seconds, label, proc):
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        if predicate():
            print('PASS', label, flush=True)
            return
        assert proc.poll() is None, 'nav_square exited: ' + str(proc.returncode)
        time.sleep(.05)
    raise AssertionError(label)


def main():
    rospy.init_node('square_isolated_fixture')
    rospy.set_param('/drone_nav/config', dict(map_id='synthetic', bounds_min=[-3., -3., .4], bounds_max=[3., 3., 2.5], max_vel=.5, max_acc=.5))
    f = Fixture()
    # 无参数时从部署位置 nav_test/tests 向上找主脚本。
    script = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), '..', 'nav_square.py')
    out = tempfile.mkdtemp(prefix='square-integration-')
    log_path = os.path.join(out, 'client.log')
    proc = None
    try:
        with open(log_path, 'w') as log:
            proc = subprocess.Popen([sys.executable, '-B', script, '--out', out, '--delay', '.2'], stdout=log, stderr=subprocess.STDOUT)
            time.sleep(2)
            assert proc.poll() is None
            assert not f.goals, 'starting OFFBOARD must not trigger'
            print('PASS startup OFFBOARD sends nothing', flush=True)
            f.mode(False)
            wait(lambda: f.prepared, 3, 'idle prepares while pilot in POSCTL', proc)
            f.mode(True)
            wait(lambda: len(f.goals) == 4, 27, 'four goals across real action callbacks', proc)
            wait(lambda: any(json.load(open(os.path.join(out, name))).get('end') == 'SUCCEEDED'
                             for name in os.listdir(out) if name.endswith('.json')), 8, 'fourth corner holds five seconds then lap succeeds', proc)
            for i in range(1, 4):
                assert f.goals[i][0] - f.goals[i - 1][0] >= 5.3
            np.testing.assert_allclose(f.position, [.2, -.1, 1.], atol=1e-10)
            print('PASS closed square, per-leg five-second holds and reprepare handshake', flush=True)
            time.sleep(.5)
            assert len(f.goals) == 4
            f.mode(False)
            wait(lambda: f.prepared, 3, 'next lap requires fresh prepare outside OFFBOARD', proc)
            f.mode(True)
            wait(lambda: len(f.goals) == 5 and f.executor == 'IDLE', 4, 'fresh OFFBOARD triggers next lap', proc)
            time.sleep(.2)
            f.pubs['/drone_nav/input_status'].publish(String(json.dumps(dict(state='FAULT', reason='transient fault'))))
            wait(lambda: any(json.load(open(os.path.join(out, name))).get('end') == 'ABORTED'
                             for name in os.listdir(out) if name.endswith('.json')), 2, 'transient input fault aborts holding lap', proc)
            time.sleep(1)
            assert len(f.goals) == 5, 'aborted lap must not send remaining edges'
            pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            assert not any(t == '/mavros/setpoint_raw/local' for t, _ in pubs)
            print('PASS no remaining goals after fault; no MAVROS setpoint publisher', flush=True)
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
            assert proc.returncode == 130, proc.returncode
            print('PASS Ctrl+C exits 130', flush=True)
    finally:
        f.stop = True
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            proc.wait(timeout=5)
        print('Evidence:', out, flush=True)
        if proc is not None and proc.returncode not in (None, 130):
            print(open(log_path).read(), flush=True)
        # SimpleActionServer 的执行线程需要 ROS shutdown 才能结束等待。
        rospy.signal_shutdown('isolated square test complete')


if __name__ == '__main__':
    main()
