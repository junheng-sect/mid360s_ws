#!/usr/bin/env python3
"""test11: four continuous paths, with the test10 takeoff and landing flow."""
import argparse
import math
from pathlib import Path
import sys
import time

import yaml

HERE = Path(__file__).resolve().parent
with (HERE / 'inspection_route.yaml').open() as stream:
    ROUTE = yaml.safe_load(stream)
SEGMENTS = ROUTE['segments']
FINAL_HOLD = 2.
ROUTE_TIMEOUT = 240.


def profile(config):
    expected = dict(takeoff_height=1., yaw_rate_deg=45., max_vel=1., max_acc=1.,
                    max_vel_z=.3, max_acc_z=.3)
    if any(config.get(key) != value for key, value in expected.items()):
        raise ValueError('test11 导航限速及起飞参数不匹配')
    if config['map_id'] != ROUTE['map_id']:
        raise ValueError('test11 需要地图 ' + ROUTE['map_id'])
    if config['bounds_min'] != [-2., -6., .4] or config['bounds_max'] != [9.2, 7., 3.6]:
        raise ValueError('test11 导航边界不匹配')


def arguments(argv):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--delay', type=float, default=3.)
    parser.add_argument('--out', default='')
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--nav-config', default=str(HERE / 'outdoor_nav_inspection.yaml'))
    args = parser.parse_args(argv)
    if not 1. <= args.delay <= 4.:
        parser.error('--delay 要在 1~4 s 之间')
    return args


def dry_run(args):
    with open(args.nav_config) as stream:
        profile(yaml.safe_load(stream)['config'])
    points = [ROUTE['start']] + [p for segment in SEGMENTS for p in segment['points']]
    length = sum(math.dist(a, b) for a, b in zip(points, points[1:]))
    print('test11｜%s｜航点折线 %.1f m｜drone_map XYZ' % (ROUTE['map_id'], length))
    print('人工解锁后等 %g s，复用 test10 自动起飞 1 m' % args.delay)
    for segment in SEGMENTS:
        print('%s：%s；停稳后保持 %g s' % (
            segment['name'], ' → '.join(str(tuple(p)) for p in segment['points']), segment['hold_s']))
    print('返回后原地恢复 0°，保持 2 s，复用 test10 慢降、着地确认和上锁')


if __name__ == '__main__' and any(flag in sys.argv for flag in ('--dry-run', '--help', '-h')):
    try:
        dry_run(arguments(sys.argv[1:]))
    except (OSError, ValueError, KeyError, yaml.YAMLError) as error:
        sys.exit(str(error))
    raise SystemExit(0)

import actionlib
from actionlib_msgs.msg import GoalStatus
import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from drone_navigation.core import check_goal_bounds, heading, wrap
from drone_navigation.msg import ExecutePathAction, ExecutePathGoal, ExecuteRouteAction
from nav_forward import say
from nav_outdoor_full import FullRunner, FullState, run_full
from nav_land import Fc
from nav_square import STOP_REASON, Stopped


class InspectionRunner(FullRunner):
    test_name = record_prefix = 'test11'
    route_name = '室外连续航段巡检'
    route = tuple(tuple(s['points'][-1]) + (0.,) for s in SEGMENTS)
    holds = tuple(s['hold_s'] for s in SEGMENTS[:-1]) + (FINAL_HOLD,)
    names = tuple(s['name'] for s in SEGMENTS[:-1]) + ('返回后恢复 0°',)
    leg_timeout = 90.

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.route_client = self.client
        self.path_client = actionlib.SimpleActionClient('/drone_nav/execute_path', ExecutePathAction)
        if not self.path_client.wait_for_server(rospy.Duration(10)):
            raise Stopped('10 s 没等到 execute_path')
        self.route_deadline = None

    def verify_profile(self):
        profile(self.cfg)

    def preflight_targets(self):
        r, t = self.reference
        points = np.array([p for segment in SEGMENTS for p in segment['points']])
        names = tuple('%s / 点%d' % (s['name'], i+1) for s in SEGMENTS for i in range(len(s['points'])))
        return (points-t) @ r, names

    def create_records(self):
        super().create_records()
        self.record['segments'] = SEGMENTS
        self.record['goal_yaw_map_deg'] = [None]*len(SEGMENTS)
        self.save()

    def guard(self, **options):
        super().guard(**options)
        if self.route_deadline is not None and time.monotonic() >= self.route_deadline:
            raise Stopped('航线总超时（240 s）')

    def trace_leg(self):
        # During the final stationary turn the test10 tracing is still applicable.
        if self.current == len(SEGMENTS)-1:
            super().trace_leg()

    def fly_path(self, index, segment):
        self.guard()
        self.client = self.path_client
        goal = ExecutePathGoal(map_id=self.cfg['map_id'], points=[Point(*p) for p in segment['points']])
        leg = dict(index=index+1, direction=segment['name'], points_map=segment['points'],
                   end='RUNNING', sent_ros=rospy.Time.now().to_sec(), sent_wall=time.time(),
                   events=[], trajectory_ids=[], course_error_deg=[])
        self.record['legs'].append(leg)
        self.save()
        latest = [None]
        self.deadline = time.monotonic() + self.leg_timeout
        self.active = True
        self.client.send_goal(goal, feedback_cb=lambda m: latest.__setitem__(0, m))
        say('%s：连续经过 %d 点后停车' % (segment['name'], len(segment['points'])))
        feedback_state, sample_at, stop_at = None, 0., None
        while True:
            self.guard(stopped=True)
            now = time.monotonic()
            state = self.client.get_state()
            if self.client.get_result() is not None or state == GoalStatus.LOST:
                break
            if self.s.get('gateway_status', 'reason') == STOP_REASON:
                stop_at = now if stop_at is None else stop_at
                if now - stop_at > .5:
                    raise Stopped('网关已停止，未收到航段成功结果')
            feedback = latest[0]
            event = None if feedback is None else (feedback.point_index, feedback.state)
            if event is not None and event != feedback_state:
                feedback_state = event
                leg['events'].append(dict(ros=rospy.Time.now().to_sec(), index=event[0], state=event[1]))
                self.save()
            if now >= sample_at:
                traj_id = self.s.get('executor_status', 'trajectory_id')
                ids = leg['trajectory_ids']
                if traj_id and (not ids or ids[-1] != traj_id):
                    ids.append(traj_id)
                angle = self.s.get('executor_status', 'course_error_deg')
                if isinstance(angle, (float, int)):
                    leg['course_error_deg'].append([rospy.Time.now().to_sec(), angle])
                sample_at = now + .1
            time.sleep(.05)

        result = self.client.get_result()
        state = self.client.get_state()
        self.active = False
        self.client = self.route_client
        self.deadline = None
        if state != GoalStatus.SUCCEEDED or result is None or not result.success:
            raise Stopped('%s：%s' % (segment['name'], result.reason if result else str(state)))
        p, yaw, _ = self.s.pose()
        leg.update(end='SUCCEEDED', end_ros=rospy.Time.now().to_sec(), end_odom=p.tolist(),
                   end_error_m=float(np.linalg.norm(p-self.goals[index])),
                   end_yaw_odom_deg=math.degrees(yaw), hold_start_wall=time.time())
        self.save()
        hold_start = time.monotonic()
        if segment['hold_s']:
            end = hold_start + segment['hold_s']
            while time.monotonic() < end:
                self.guard(stopped=True)
                actual, actual_yaw, speed = self.s.pose()
                if (np.linalg.norm(actual-self.goals[index]) > .15 or speed > .1 or
                        abs(wrap(actual_yaw-yaw)) > math.radians(5)):
                    raise Stopped('%s 停留期间失去稳定' % segment['name'])
                time.sleep(min(.05, max(0., end-time.monotonic())))
        leg['hold_elapsed_s'] = time.monotonic()-hold_start
        self.save()

    def route_finished(self):
        return (len(self.record['legs']) == len(SEGMENTS)+1 and
                all(leg['end'] == 'SUCCEEDED' for leg in self.record['legs']) and
                self.record['legs'][-1].get('kind') == 'return_heading')

    def fly_route(self):
        self.phase('ROUTE')
        self.route_deadline = time.monotonic() + ROUTE_TIMEOUT
        try:
            for index, segment in enumerate(SEGMENTS):
                if index:
                    self.reprepare()
                self.fly_path(index, segment)
            self.reprepare()
            i = len(SEGMENTS)-1
            self.send_leg(i, self.goals[i], self.goals_map[i], self.yaw_map[i])
            self.record['legs'][-1].update(kind='return_heading', index=len(SEGMENTS)+1)
            self.check_return()
            self.record.update(end='SUCCEEDED', end_wall=time.time())
            self.save()
        finally:
            self.route_deadline = None

    def preview_route(self):
        self.verify_profile()
        self.s.begin(False, self.cfg['map_id'])
        self.create_records()
        try:
            end = time.monotonic() + 20.
            while self.inputs_problem():
                if self.s.get('gateway_status', 'real_output') is True:
                    raise Stopped('预览要求 real_output=false')
                if rospy.is_shutdown() or time.monotonic() >= end:
                    raise Stopped(self.inputs_problem())
                time.sleep(.1)
            self.reference = self.transform()
            r, t = self.reference
            if np.linalg.norm(r @ self.s.pose()[0] + t - ROUTE['start']) > .3:
                raise Stopped('预览从地图原点上方 1 m（误差不超过 0.3 m）开始')
            self.goals_map = np.array([p[:3] for p in self.route])
            self.goals = (self.goals_map-t) @ r
            self.yaw_map = np.zeros(len(self.route))
            self.yaws = np.array([heading(r.T, 0.)]*len(self.route))
            points, names = self.preflight_targets()
            check_goal_bounds(points, self.cfg, names)
            # Preview startup uses the ordinary preparation service only.
            response = rospy.ServiceProxy('/drone_nav/prepare', Trigger)()
            if not response.success:
                raise Stopped(response.message)
            until = time.monotonic() + 3.
            while self.s.get('gateway_status') != 'READY':
                if time.monotonic() >= until or rospy.is_shutdown():
                    raise Stopped('预览网关未 READY')
                time.sleep(.05)
            self.fly_route()
            self.flight['end'] = 'SUCCEEDED'
            return 0
        except BaseException as error:
            if self.active:
                self.client.cancel_goal()
            self.record.update(end='ABORTED', reason=str(error))
            self.flight.update(end='ABORTED', reason=str(error))
            raise
        finally:
            self.s.end()
            self.save()


def preview(args):
    rospy.init_node('nav_test11_preview', anonymous=True, disable_signals=True)
    rospy.wait_for_service('/drone_nav/prepare', timeout=10)
    state = FullState()
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
    if not client.wait_for_server(rospy.Duration(10)):
        raise Stopped('10 s 没等到 execute_route')
    runner = InspectionRunner(args, rospy.get_param('/drone_nav/config'), state, buf,
                              rospy.ServiceProxy('/drone_nav/prepare', Trigger),
                              rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory), client, Fc(state))
    return runner.preview_route()


if __name__ == '__main__':
    try:
        args = arguments(rospy.myargv()[1:])
        raise SystemExit(preview(args) if args.preview else run_full(args, InspectionRunner, 'test11'))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except Exception as error:
        say('test11 终止：' + str(error))
        raise SystemExit(1)
