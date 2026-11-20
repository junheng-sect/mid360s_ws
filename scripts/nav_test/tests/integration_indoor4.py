#!/usr/bin/env python3
"""Synthetic observations with real test4 navigation on the isolated master."""
import argparse
import itertools
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import actionlib
import cv2
import numpy as np
import rosgraph
import rospy
import tf2_ros
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_matrix
from traj_utils.srv import CheckTrajectory
from drone_navigation.core import Spline, rz
from drone_navigation.msg import ExecuteRouteAction, ExecuteRouteActionGoal, ExecuteRouteGoal
from drone_navigation.ros_common import path_of
from barcode_reader_camera.msg import Barcode
from barcode_reader_camera.srv import DecodeBarcodes, DecodeBarcodesResponse
from ov13855_apriltag_vision.msg import TagObservation, TagObservationArray

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from integration_sequence import Fixture
from indoor4_logic import shortest_order
from nav_indoor3 import load_route
from nav_indoor4 import (CAMERA_IMAGE, CAMERA_INFO, CANCEL_DECODE, DECODE, DETECTIONS, FRAME,
                         SIZE, Indoor4Runner, load_calibration)
from nav_square import SquareState, Stopped


def wait_for(test, seconds, reason):
    deadline = time.monotonic() + seconds
    while not test():
        assert time.monotonic() < deadline, reason
        time.sleep(.05)


def ordering():
    start, finish = np.zeros(3), np.array([2., 2., 1.])
    points = dict(enumerate(np.random.RandomState(42).rand(10, 3)))
    for n in (0, 1, 6, 10):
        limited = dict(list(points.items())[:n])
        order, distance = shortest_order(start, limited, finish)
        assert sorted(order) == sorted(limited)
        if n <= 6:
            def cost(ids):
                line = [start, *[limited[k] for k in ids], finish]
                return sum(np.linalg.norm(b-a) for a, b in zip(line, line[1:]))
            best = min(cost(ids) for ids in itertools.permutations(limited))
            assert abs(distance - best) < 1e-10 and abs(cost(order) - best) < 1e-10
    print('PASS exact order vs permutations, empty batch and ten tags', flush=True)


class Camera:
    def __init__(self, flight, cfg, route):
        self.flight = flight
        self.route = route
        self.poses = None
        self.ids = []
        self.block_decode = False
        self.waiting_decode = threading.Event()
        self.cancelled = threading.Event()
        self.translation, self.rotation, self.layout, _ = load_calibration()
        self.offset = self.translation - np.asarray(cfg['lever_body'])
        self.coords = {
            1: np.array([.61, -1.15, .44]),
            2: np.array([.74, -1.15, .45]),
            3: np.array([.62, -1.15, 1.16]),
            4: np.array([-.29, -1.15, .44]),
        }
        upright = np.array([[-1.,0,0],[0,0,1.],[0,1.,0]])
        self.tag_axes = {key: rz(math.radians(deg)) @ upright
                         for key, deg in zip(self.coords, (8., -9., 0., 5.))}
        self.k = [2700., 0., SIZE[0]/2, 0., 2700., SIZE[1]/2, 0., 0., 1.]
        self.image = rospy.Publisher(CAMERA_IMAGE, Image, queue_size=1)
        self.info = rospy.Publisher(CAMERA_INFO, CameraInfo, queue_size=1, latch=True)
        self.tags = rospy.Publisher(DETECTIONS, TagObservationArray, queue_size=3)
        self.target = rospy.Subscriber('/indoor4/target_tag_pose', PoseStamped, self.on_target, queue_size=4)
        self.decoder = rospy.Service(DECODE, DecodeBarcodes, self.decode)
        self.cancel = rospy.Service(CANCEL_DECODE, Trigger, self.cancel_decode)
        self.timer = rospy.Timer(rospy.Duration(.067), self.publish)
        self.image_timer = rospy.Timer(rospy.Duration(.3), self.publish_image)
        self.image_data = bytes(SIZE[0] * SIZE[1])

    def on_target(self, msg):
        self.poses = msg

    def publish(self, _):
        p = rz(self.flight.map_yaw) @ self.flight.p.copy()
        yaw = self.flight.a + self.flight.map_yaw
        body = rz(yaw)
        camera = p + body @ self.offset
        view = body @ self.rotation
        header = Header(stamp=rospy.Time.now(), frame_id=FRAME)
        near = min(range(4), key=lambda i: np.linalg.norm(np.asarray(self.route[i]['xyz']) - p))
        at_observation = np.linalg.norm(np.asarray(self.route[near]['xyz']) - p) < .04
        ids = ((1,2), (1,3), (), (2,4))[near] if at_observation else tuple(self.coords)
        tags = []
        for key in ids:
            pos = view.T @ (self.coords[key] - camera)
            if pos[2] > .4:
                matrix = np.eye(4)
                matrix[:3,:3] = view.T @ self.tag_axes[key]
                quat = Quaternion(*quaternion_from_matrix(matrix))
                tags.append(TagObservation(id=key, pose=Pose(Point(*pos), quat), reprojection_rms_px=.3))
        self.source = camera, view
        self.tags.publish(TagObservationArray(header=header, family='tag36h11', tags=tags))
        self.info.publish(CameraInfo(header=header, width=SIZE[0], height=SIZE[1], K=self.k, D=[0.] * 5))

    def publish_image(self, _):
        if self.image.get_num_connections():
            self.image.publish(Image(header=Header(stamp=rospy.Time.now(), frame_id=FRAME),
                                     width=SIZE[0], height=SIZE[1], step=SIZE[0], encoding='mono8',
                                     data=self.image_data))

    def cancel_decode(self, _):
        self.cancelled.set()
        return TriggerResponse(success=True)

    def decode(self, _):
        if self.block_decode:
            self.waiting_decode.set()
            self.cancelled.wait(4.)
            return DecodeBarcodesResponse(error='cancelled')
        called = rospy.Time.now()
        wait_for(lambda: self.poses is not None and self.poses.header.stamp > called, 2., 'missing targeted pose')
        msg = self.poses
        pos, orientation = msg.pose.position, msg.pose.orientation
        p, view = self.source
        # Closest world position identifies which of the simulated tags the
        # real mission forwarded to the decoder.
        world = p + view @ np.asarray([pos.x, pos.y, pos.z])
        key = min(self.coords, key=lambda k: np.linalg.norm(self.coords[k] - world))
        self.ids.append(key)
        if key == 3:
            return DecodeBarcodesResponse(error='decode_fail', header=msg.header)
        matrix = view.T @ self.tag_axes[key]
        rvec = cv2.Rodrigues(matrix)[0]
        shape = np.array([[-.025,.035,0], [.025,.035,0], [.025,.085,0], [-.025,.085,0]])
        if key == 2:
            # Return a neighbouring label's pixels despite requesting tag 2.
            # The runner must reject this result even if the decoder says success.
            bad = view.T @ (self.coords[1] - self.coords[2])
        else:
            bad = np.zeros(3)
        pixels = cv2.projectPoints(shape, rvec, np.array([pos.x,pos.y,pos.z]) + bad,
                                   np.asarray(self.k).reshape(3,3), np.zeros(5))[0]
        code = Barcode(format='QRCode', code='label-%d' % key, corners=pixels.reshape(8).tolist())
        return DecodeBarcodesResponse(success=True, used_tag_roi=True, codes=[code], header=msg.header)

    def close(self):
        self.timer.shutdown()
        self.image_timer.shutdown()


def check_dispatched_goals(messages, record, route, camera, out):
    """Check the actual ROS action goals against the physical paper positions in the fixture."""
    yaw_deg = route['yaw_deg']
    assert yaw_deg == -89.5
    yaw_map = math.radians(yaw_deg)
    map_rotation = rz(camera.flight.map_yaw)
    np.testing.assert_allclose(record['transform_rotation'], map_rotation, atol=1e-7)
    offset_map = map_rotation @ rz(yaw_map - camera.flight.map_yaw) @ camera.offset
    label = camera.layout['label']
    paper_offset = np.array([label['tag_offset_x_meters'], label['tag_offset_y_meters'], 0.])

    expected = []
    for point, batch in zip(route['points'], record['batches']):
        expected.append((point['name'], np.array(point['xyz']), None))
        for key in batch['order']:
            axes = camera.tag_axes[key]
            paper = camera.coords[key] - axes @ paper_offset
            optical_center = paper + .80 * axes[:, 2]
            expected.append(('Tag %d 读码点' % key, optical_center - offset_map, optical_center))
    expected.append(('返回右下观察点', np.array(route['points'][-1]['xyz']), None))
    assert len(messages) == len(record['legs']) == len(expected) == 9
    evidence = []
    for message, leg, (name, position, optical_center) in zip(messages, record['legs'], expected):
        assert leg['direction'] == name and len(message.goal.waypoints) == 1
        pose = message.goal.waypoints[0]
        assert pose.header.frame_id == 'drone_map'
        p, q = pose.pose.position, pose.pose.orientation
        actual = np.array([p.x, p.y, p.z])
        measured_yaw = math.degrees(2 * math.atan2(q.z, q.w))
        np.testing.assert_allclose([q.x, q.y, q.z*q.z+q.w*q.w, measured_yaw], [0.,0.,1.,yaw_deg], atol=1e-7)
        np.testing.assert_allclose(actual, position, atol=.005)
        item = dict(leg=name, frame=pose.header.frame_id, yaw_deg=measured_yaw, goal_map=actual.tolist())
        if optical_center is not None:
            # Reconstruct the physical camera using the dispatched attitude and its calibrated lever arm.
            camera_map = actual + rz(math.radians(measured_yaw)) @ camera.offset
            np.testing.assert_allclose(camera_map, optical_center, atol=.005)
            item['optical_center_error_m'] = float(np.linalg.norm(camera_map-optical_center))
        evidence.append(item)
    (out/'dispatched_goals.json').write_text(json.dumps(evidence, ensure_ascii=False, indent=2)+'\n')
    print('PASS all 9 drone_map action goals have yaw %.1f deg with a %.1f deg rotated map; '
          'camera positions match independent paper normals' % (yaw_deg, math.degrees(camera.flight.map_yaw)), flush=True)


def short_goals(fixture, runner, spline_messages):
    for distance in (0., .01, .10, .19, .20):
        before = fixture.p.copy()
        goal = before + np.array([distance, 0., 0.])
        angle = fixture.map_yaw + fixture.a + (.23 if distance == 0. else 0.)
        runner.reprepare()
        start = rospy.Time.now()
        pose = PoseStamped(header=Header(frame_id='drone_map'),
                           pose=Pose(Point(*(rz(fixture.map_yaw) @ goal)), Quaternion(0.,0.,math.sin(angle/2),math.cos(angle/2))))
        runner.client.send_goal(ExecuteRouteGoal(map_id='synthetic', waypoints=[pose]))
        assert runner.client.wait_for_result(rospy.Duration(16.)), distance
        result = runner.client.get_result()
        assert result.success, (distance, result)
        messages = [m for m in spline_messages if m.start_time >= start]
        assert messages, distance
        if distance < .20:
            spline = Spline(3, messages[0].knots, [[p.x,p.y,p.z] for p in messages[0].pos_pts], 0., 1)
            np.testing.assert_allclose(spline.position(0.), before, atol=.002)
            np.testing.assert_allclose(spline.position(spline.duration), goal, atol=.002)
            for t in np.linspace(0., spline.duration, 101):
                _, v, a = spline.state(t)
                assert np.linalg.norm(v) <= .10001
                assert np.linalg.norm(a) <= .20001
            for t in (0., spline.duration):
                assert np.linalg.norm(spline.state(t)[1:]) < 1e-7
        assert np.linalg.norm(goal-fixture.p) < .04, distance
        print('PASS move %.2f m and settle' % distance, flush=True)


def reject_occupied(fixture, runner, splines):
    goal = fixture.p + np.array([.10, 0., 0.])
    obstacle = goal + np.array([.22, 0., 0.])
    fixture.points = np.vstack((fixture.points, np.repeat(obstacle[None], 800, axis=0)))
    wait_for(lambda: not runner.check(path_of([goal])).valid, 10., 'fixture obstacle not occupied')
    runner.reprepare()
    before = len(splines)
    pose = PoseStamped(header=Header(frame_id='drone_map'),
                       pose=Pose(Point(*(rz(fixture.map_yaw) @ goal)), Quaternion(0.,0.,0.,1.)))
    runner.client.send_goal(ExecuteRouteGoal(map_id='synthetic', waypoints=[pose]))
    assert runner.client.wait_for_result(rospy.Duration(5.))
    result = runner.client.get_result()
    assert not result.success and 'occupied' in result.reason and len(splines) == before, result
    print('PASS occupied short target rejected before a spline was sent', flush=True)


def decode_cancel(runner, camera):
    camera.block_decode = True

    def lose_localization():
        if camera.waiting_decode.wait(3.):
            runner.s.stop('synthetic localization loss')

    thread = threading.Thread(target=lose_localization, daemon=True)
    thread.start()
    try:
        runner.decode_once(('tag36h11', 1))
        raise AssertionError('decode survived navigation loss')
    except Stopped as error:
        assert 'synthetic localization loss' in str(error)
    assert camera.cancelled.wait(2.), 'decode service did not receive cancellation'
    assert runner.vision.target is None
    thread.join(timeout=1.)
    print('PASS navigation loss cancels active decode and clears target', flush=True)


def main():
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11446'
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    ordering()
    rospy.init_node('indoor4_isolated')
    from traj_utils.msg import Bspline
    splines = []
    sub = rospy.Subscriber('/drone_nav/bspline', Bspline, splines.append, queue_size=20)
    cfg = rospy.get_param('/drone_nav/config')
    route = load_route()
    route['map_id'] = 'synthetic'
    fixture = Fixture(args.out, map_yaw=math.radians(25.))
    map_rotation = rz(fixture.map_yaw)
    fixture.p = map_rotation.T @ np.array([.45, .34, .50])
    fixture.a = math.radians(route['yaw_deg']) - fixture.map_yaw
    sent_goals = []
    wire = rospy.Subscriber('/drone_nav/execute_route/goal', ExecuteRouteActionGoal,
                            sent_goals.append, queue_size=32)
    camera = Camera(fixture, cfg, route['points'])
    try:
        state, buffer = SquareState(), tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buffer)
        rospy.wait_for_service('/drone_nav/prepare', timeout=20)
        prepare = rospy.ServiceProxy('/drone_nav/prepare', Trigger)
        check = rospy.ServiceProxy('/drone_nav/check_trajectory', CheckTrajectory)
        client = actionlib.SimpleActionClient('/drone_nav/execute_route', ExecuteRouteAction)
        assert client.wait_for_server(rospy.Duration(15))
        run_args = SimpleNamespace(preview=True, points=[(*p['xyz'], route['yaw_deg']) for p in route['points']],
            max_start_offset=math.inf, check_segments=False, hold=3., delay=3., out=str(args.out), route=route)
        runner = Indoor4Runner(run_args, cfg, state, buffer, prepare, check, client)
        wait_for(lambda: not runner.inputs_problem() and prepare().success, 20, 'input not ready')
        wait_for(lambda: check(path_of([fixture.p, map_rotation.T @ route['points'][0]['xyz']])).valid, 20, 'free grid not ready')
        wait_for(lambda: wire.get_num_connections() > 0, 5, 'action goal monitor not connected')
        assert runner.preview(), runner.record.get('reason', 'no mission record')
        record = runner.record
        assert record['end'] == 'SUCCEEDED'
        assert len(record['batches']) == 4
        assert all(batch['photo'].get('file') for batch in record['batches'])
        assert len(camera.ids) == 4 and set(camera.ids) == {1,2,3,4}, camera.ids
        assert len(record['scans']) == 4
        assert {item['id']: item['status'] for item in record['scans']} == {
            1:'success', 2:'failed', 3:'failed', 4:'success'}, record['scans']
        assert not record['batches'][2]['order'], record['batches'][2]
        assert np.linalg.norm(map_rotation @ fixture.p - route['points'][3]['xyz']) < .05
        check_dispatched_goals(sent_goals, record, route, camera, args.out)
        for batch in record['batches']:
            for entry in batch['candidates']:
                if 'camera_map' in entry:
                    np.testing.assert_allclose(entry['tag_xyz_map'], camera.coords[entry['id']], atol=.005)
                    np.testing.assert_allclose(np.linalg.norm(np.asarray(entry['camera_map'])-entry['paper_map']), .8)
        print('PASS four batches, photo capture, unique decode, neighbour QR rejection, failure continuation and return', flush=True)
        runner.s.begin(False, cfg['map_id'])
        short_goals(fixture, runner, splines)
        reject_occupied(fixture, runner, splines)
        decode_cancel(runner, camera)
        assert not any(topic == '/mavros/setpoint_raw/local'
                       for topic, _ in rosgraph.Master(rospy.get_name()).getSystemState()[0])
        print('PASS no MAVROS setpoint output', flush=True)
    finally:
        camera.close()
        fixture.close()
        wire.unregister()
        sub.unregister()
        rospy.signal_shutdown('done')


if __name__ == '__main__':
    main()
