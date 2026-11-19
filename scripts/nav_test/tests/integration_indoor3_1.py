#!/usr/bin/env python3
"""Exercise the four-stop survey on a private master with synthetic flight and camera."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import actionlib
import cv2
import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from traj_utils.srv import CheckTrajectory
from ov13855_apriltag_vision.msg import TagObservation, TagSurveyFrame
from drone_navigation.core import rz
from drone_navigation.msg import ExecuteRouteAction
from drone_navigation.ros_common import path_of

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration_sequence import Fixture
from indoor3_runner import Indoor3Runner
from nav_indoor3 import load_route
from nav_indoor3_1 import FRAME, FAMILY, TOPIC, DualSurveyRunner, SurveyRunner, coverage, waypoint_rows
from nav_square import SquareState


def tag(i):
    return TagObservation(id=i, pose=Pose(Point(.1, .2, .8), Quaternion(0., 0., 0., 1.)),
                          reprojection_rms_px=.25)


def test_rows():
    stamps = [1700000000123456789 + n for n in range(3)]
    frames = {t: TagSurveyFrame(ids=ids, tags=[tag(i) for i in ids if i != 42])
              for t, ids in zip(stamps, ((2, 2, 42), (2,), (0, 2)))}
    nav = SimpleNamespace(position=np.array([1., 0., .4]), quaternion=[0., 0., 0., 1.])
    lookup = lambda t: nav if t >= stamps[1] else None
    ref = rz(math.pi / 2), np.array([1., 2., 3.])
    rows = waypoint_rows(frames, 1, '左下', dict(file='first.png', stamp_ns=stamps[0]), lookup, ref)
    data = {r['id']: r for r in rows}
    assert data[2]['seen_frames'] == 3 and data[2]['representative_stamp_ns'] == stamps[1]
    assert data[2]['first_stamp_ns'] == stamps[0] and data[2]['last_stamp_ns'] == stamps[2]
    assert data[0]['seen_frames'] == 1 and data[42]['pose_status'] == 'no_synchronized_odometry'
    assert data[42]['tag_pose_status'] == 'unavailable' and 'reprojection_rms_px' not in data[42]
    assert 'drone_map_x_m' not in data[42] and data[42]['representative_stamp_ns'] == stamps[0]
    np.testing.assert_allclose([data[2]['drone_map_' + k] for k in ('x_m','y_m','z_m')], [1.,3.,3.4])
    np.testing.assert_allclose([data[2]['drone_map_' + k] for k in ('qx','qy','qz','qw')],
                               [0.,0.,2**-.5,2**-.5], atol=1e-8)
    result = coverage(rows)
    assert result['seen'] == [0,2] and result['missing'] == [1,3,4,5] and result['extra'] == [42]
    print('PASS duplicate ID, one-frame hit, earliest aligned frame, missing odometry, map pose and partial coverage')


class Camera:
    def __init__(self, fixture, points, mode):
        self.fixture, self.points, self.mode = fixture, np.asarray([p['xyz'] for p in points]), mode
        self.tags = rospy.Publisher(TOPIC, TagSurveyFrame, queue_size=5)
        self.usb = None
        if mode.startswith('dual'):
            topic, self.usb_frame = DualSurveyRunner.cameras['ov9281']
            self.usb = rospy.Publisher(topic, TagSurveyFrame, queue_size=5)
        self.images = rospy.Publisher('/ov13855_full/image_raw', Image, queue_size=1)
        self.pixels = bytes(4224 * 3136)
        self.tag_timer = rospy.Timer(rospy.Duration(.08), self.publish_tags)
        self.image_timer = rospy.Timer(rospy.Duration(.2), self.publish_image)
        self.interrupted = False
        self.runner = None

    def publish_tags(self, _):
        distance = np.linalg.norm(self.points - self.fixture.p, axis=1)
        nearest = int(np.argmin(distance))
        ids = ((5, 1, 0), (2, 1), (), (3, 4, 7))[nearest] if distance[nearest] < .04 else (99,)
        if nearest == 1 and distance[nearest] < .04:
            if self.mode == 'dropout':
                return
            if self.mode == 'stop' and self.runner and self.runner.record and not self.interrupted:
                self.runner.s.stop('synthetic pilot interruption')
                self.interrupted = True
        if self.usb and not (self.mode == 'dual_dropout' and nearest == 1 and distance[nearest] < .04):
            # Same ID on both cameras, USB-only ID, plus a live window with no hits.
            usb_ids = ((1, 8), (), (4,), (0,))[nearest] if distance[nearest] < .04 else (99,)
            self.usb.publish(TagSurveyFrame(header=Header(frame_id=self.usb_frame, stamp=rospy.Time.now()),
                                            family=FAMILY, ids=usb_ids, tags=[tag(i) for i in usb_ids]))
        self.tags.publish(TagSurveyFrame(header=Header(frame_id=FRAME, stamp=rospy.Time.now()),
                                              family=FAMILY, ids=ids, tags=[tag(i) for i in ids]))

    def publish_image(self, _):
        if self.images.get_num_connections():
            self.images.publish(Image(header=Header(frame_id=FRAME, stamp=rospy.Time.now()),
                                      width=4224, height=3136, step=4224, encoding='mono8', data=self.pixels))

    def close(self):
        self.tag_timer.shutdown()
        self.image_timer.shutdown()


def verify(out, record, mode):
    with (out / 'apriltag_observations.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == len(record['observations'])
    assert not any(r['id'] == '99' for r in rows)
    assert len(set((r['waypoint'], r['id']) for r in rows)) == len(rows)
    complete = [l for l in record['legs'] if l.get('observation', {}).get('status') == 'complete']
    for leg in complete:
        obs, photo = leg['observation'], leg['photo']
        assert obs['start_ns'] <= photo['stamp_ns'] < obs['end_ns']
        assert cv2.imread(str(out / photo['file']), cv2.IMREAD_GRAYSCALE).shape == (3136, 4224)
        for r in [row for row in rows if row['photo_file'] == photo['file']]:
            assert r['photo_stamp_ns'] == str(photo['stamp_ns']) and r['pose_status'] == 'aligned'
            assert obs['start_ns'] <= int(r['first_stamp_ns']) <= int(r['representative_stamp_ns']) <= int(r['last_stamp_ns']) < obs['end_ns']
            assert int(r['seen_frames']) <= int(r['window_frames']) == obs['frames']
    if mode == 'full':
        assert record['end'] == 'SUCCEEDED' and len(complete) == 4 and len(rows) == 8
        assert record['coverage']['count'] == 6 and record['coverage']['extra'] == [7]
        assert record['coverage']['missing'] == [] and not complete[2]['observation']['ids']
        assert sum(r['id'] == '1' for r in rows) == 2 and len(list(out.glob('*.png'))) == 4
    else:
        assert record['end'] == 'ABORTED' and len(complete) == 1 and len(rows) == 3
        assert record['coverage']['missing'] == [2,3,4]
        if mode == 'dropout':
            obs = record['legs'][-1]['observation']
            assert obs['status'] == 'stream_fault' and obs['frames'] == 0
        else:
            assert 'synthetic pilot interruption' in record['reason']
    print('PASS', mode, record['end'], 'rows', len(rows), record['coverage'], flush=True)


def verify_dual(out, record, mode):
    with (out / 'apriltag_observations.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    assert record['test'] == 'test_indoor3_2' and len(rows) == len(record['observations'])
    assert len(rows) == len({(r['waypoint'], r['camera'], r['id']) for r in rows})
    upper, lower = 'ov13855', 'ov9281'
    for i, leg in enumerate(record['legs'], 1):
        obs = leg['observation']
        stats = obs['cameras']
        assert obs['end_ns'] - obs['start_ns'] == 2_000_000_000
        for r in (r for r in rows if int(r['waypoint']) == i):
            item = stats[r['camera']]
            assert int(r['window_frames']) == item['frames'] and 0 < int(r['seen_frames']) <= item['frames']
            assert obs['start_ns'] <= int(r['first_stamp_ns']) <= int(r['last_stamp_ns']) < obs['end_ns']
            assert int(r['id']) in item['ids'] and r['pose_status'] == 'aligned'
        assert (out / leg['photo']['file']).is_file()
    assert record['legs'][0]['observation']['cameras'][upper]['ids'] == [0, 1, 5]
    assert record['legs'][0]['observation']['cameras'][lower]['ids'] == [1, 8]
    if mode == 'dual':
        assert record['end'] == 'SUCCEEDED' and len(record['legs']) == 4 and len(rows) == 12
        assert record['coverage'][upper]['count'] == 6 and record['coverage'][upper]['extra'] == [7]
        assert record['coverage'][lower]['seen'] == [0, 1, 4] and record['coverage'][lower]['extra'] == [8]
        empty = record['legs'][1]['observation']['cameras'][lower]
        assert empty['frames'] > 0 and empty['ids'] == []
    else:
        assert record['end'] == 'ABORTED' and len(record['legs']) == 2
        assert obs['status'] == 'stream_fault' and stats[lower]['frames'] == 0 and stats[upper]['frames'] > 0
        assert lower in record['reason']
    print('PASS', mode, record['end'], 'rows', len(rows), record['coverage'], flush=True)


def main():
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11447'
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--mode', choices=('full', 'dropout', 'stop', 'photo', 'dual', 'dual_dropout'), default='full')
    args = parser.parse_args()
    test_rows()
    rospy.init_node('indoor3_1_isolated')
    route = load_route()
    route['map_id'] = 'synthetic'
    flight = Fixture(args.out)
    flight.p = np.array([.45, .34, .50])
    flight.a = math.radians(route['yaw_deg'])
    camera = Camera(flight, route['points'], args.mode)
    try:
        state, buf = SquareState(), tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buf)
        rospy.wait_for_service('/drone_nav/prepare', timeout=20)
        prepare = rospy.ServiceProxy('/drone_nav/prepare', Trigger)
        check = rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory)
        client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
        assert client.wait_for_server(rospy.Duration(15))
        run_args = SimpleNamespace(preview=True, points=[(*p['xyz'], route['yaw_deg']) for p in route['points']],
                                   max_start_offset=math.inf, check_segments=True, hold=3., delay=3.,
                                   out=str(args.out), route=route)
        runner_class = Indoor3Runner if args.mode == 'photo' else SurveyRunner
        if args.mode.startswith('dual'):
            runner_class = DualSurveyRunner
        runner = runner_class(run_args, rospy.get_param('/drone_nav/config'), state, buf, prepare, check, client)
        camera.runner = runner
        deadline = time.monotonic() + 30
        while runner.inputs_problem() or not prepare().success or not check(path_of([flight.p, route['points'][0]['xyz']])).valid:
            assert time.monotonic() < deadline, 'inputs/observed free grid not ready'
            time.sleep(.1)
        ok = runner.preview()
        assert ok == (args.mode in ('full', 'photo', 'dual')), runner.record
        record = json.loads(Path(runner.record_path).read_text())
        if args.mode == 'photo':
            assert record['test'] == 'test_indoor3' and record['end'] == 'SUCCEEDED'
            assert len(record['legs']) == 4 and not (args.out / 'apriltag_observations.csv').exists()
            for leg in record['legs']:
                assert leg['photo']['control_xyz_map'] and (args.out / leg['photo']['file']).is_file()
            print('PASS original indoor test3 saved four full-resolution photos and poses')
        elif args.mode.startswith('dual'):
            verify_dual(args.out, record, args.mode)
        else:
            verify(args.out, record, args.mode)
        assert not any(topic == '/mavros/setpoint_raw/local'
                       for topic, _ in rosgraph.Master(rospy.get_name()).getSystemState()[0])
    finally:
        camera.close()
        flight.close()
        rospy.signal_shutdown('test complete')


if __name__ == '__main__':
    main()
