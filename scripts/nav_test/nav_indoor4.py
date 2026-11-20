#!/usr/bin/env python3
"""Discover shelf labels and execute a separate scan batch at each indoor3 stop."""
import argparse
from collections import Counter, defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import shutil
import subprocess
import threading
import time
from types import SimpleNamespace

import cv2
import numpy as np
import rospkg
import rospy
import yaml
from barcode_reader_camera.srv import DecodeBarcodes
from drone_navigation.core import check_goal_bounds, finite, rotation
from front_marker_control.localization import LocalPoseBuffer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ov13855_apriltag_vision.msg import TagObservationArray
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger

from indoor4_logic import STANDOFF, consolidate, map_tag, scan_target, shortest_order
from nav_forward import path_of, say
from nav_global import GlobalRunner, serve
from nav_indoor3 import load_route
from nav_square import Stopped

DETECTIONS = '/ov13855_apriltag_vision/detections'
CAMERA_IMAGE = '/ov13855_full/image_raw'
CAMERA_INFO = '/ov13855_full/camera_info'
DECODE = '/barcode_reader_camera/decode'
CANCEL_DECODE = '/barcode_reader_camera/cancel'
FRAME = 'ov13855_optical_frame'
SIZE = (4224, 3136)


def camera_frame(message):
    width, height = SIZE
    return ((message.width, message.height) == SIZE and message.encoding == 'mono8'
            and message.header.frame_id == FRAME and message.step >= width
            and len(message.data) == message.step * height
            and message.header.stamp > rospy.Time(0))


def valid_info(message):
    return ((message.width, message.height) == SIZE and message.header.frame_id == FRAME
            and message.K[0] > 0 and message.K[4] > 0 and np.isfinite((*message.K, *message.D)).all())


def wait_until(get, seconds, guard, after=None):
    end = time.monotonic() + seconds
    while not rospy.is_shutdown():
        guard()
        value = get()
        if value is not None and (after is None or
                (value.header.stamp >= after and -.02 <= (rospy.Time.now() - value.header.stamp).to_sec() <= .5)):
            return value
        remaining = end - time.monotonic()
        if remaining <= 0:
            return None
        time.sleep(min(.025, remaining))


class Vision:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.info = None
        self.frames = deque(maxlen=100)
        self.target = None
        self.since = rospy.Time(0)
        self.forwarded = deque(maxlen=150)
        self.pose_pub = rospy.Publisher('/indoor4/target_tag_pose', PoseStamped, queue_size=3)
        self.detections = rospy.Subscriber(DETECTIONS, TagObservationArray, self.on_detections, queue_size=4)
        self.camera = rospy.Subscriber(CAMERA_INFO, CameraInfo, self.on_info, queue_size=1)
        self.decode = rospy.ServiceProxy(DECODE, DecodeBarcodes)
        self.cancel = rospy.ServiceProxy(CANCEL_DECODE, Trigger)

    def on_info(self, msg):
        if valid_info(msg):
            self.info = msg

    def on_detections(self, msg):
        if (msg.header.frame_id != FRAME or msg.family != 'tag36h11'
                or not -.02 <= (rospy.Time.now() - msg.header.stamp).to_sec() <= .5
                or msg.header.stamp == rospy.Time(0)):
            return
        with self.lock:
            self.latest = msg
            self.frames.append(msg)
            if self.target is None or msg.header.stamp < self.since:
                return
            matches = [tag for tag in msg.tags if (msg.family, tag.id) == self.target
                       and math.isfinite(tag.reprojection_rms_px) and tag.reprojection_rms_px <= 2.]
            if len(matches) == 1:
                pose = PoseStamped(header=msg.header, pose=matches[0].pose)
                self.forwarded.append(pose)
                self.pose_pub.publish(pose)

    def window(self, start, end):
        with self.lock:
            return [m for m in self.frames if start <= m.header.stamp <= end]

    def select(self, target):
        with self.lock:
            self.target = target
            self.since = rospy.Time.now()
            self.forwarded.clear()

    def matched_pose(self, stamp):
        with self.lock:
            poses = list(self.forwarded)
        closest = min(poses, key=lambda p: abs((p.header.stamp - stamp).to_sec()), default=None)
        if closest and abs((closest.header.stamp - stamp).to_sec()) <= .010:
            return closest
        return None

    def take_photo(self, after, guard):
        frame = None

        def on_image(msg):
            nonlocal frame
            if camera_frame(msg):
                frame = msg

        subscription = rospy.Subscriber(CAMERA_IMAGE, Image, on_image, queue_size=1,
                                        buff_size=2 * SIZE[0] * SIZE[1])
        try:
            return wait_until(lambda: frame, 2., guard, after)
        finally:
            subscription.unregister()


def load_calibration():
    packages = rospkg.RosPack()
    tf_path = Path(packages.get_path('front_marker_control')) / 'config/ov13855_camera_tf.yaml'
    reader = Path(packages.get_path('barcode_reader_camera')) / 'config'
    layout_path = reader / 'layouts/qr_top_tag_bottom.yaml'
    tf = yaml.safe_load(tf_path.read_text())
    layout = yaml.safe_load(layout_path.read_text())
    if tf['base_frame'] != 'drone_base' or tf['camera_frame'] != FRAME or layout['label']['tag_position'] != 'custom':
        raise ValueError('test4 camera calibration or label geometry is incompatible')
    return (finite(tf['translation']), rotation(tf['rotation_quaternion']), layout,
            (tf_path, layout_path, reader / 'barcode_reader_camera.yaml'))


class Indoor4Runner(GlobalRunner):
    test_name = record_prefix = 'test_indoor4'
    route_name = '室内 test4 四点分批扫码'
    side_m = None
    repeat_laps = False

    def __init__(self, args, cfg, state, buf, prepare, check, client):
        super().__init__(args, cfg, state, buf, prepare, check, client)
        self.route = args.route
        self.out = Path(args.out)
        self.cam_translation, self.cam_rotation, self.layout, self.config_paths = load_calibration()
        self.lever = finite(cfg['lever_body'])
        self.vision = Vision()
        self.pose_lock = threading.Lock()
        self.poses = LocalPoseBuffer(400)
        self.odom = rospy.Subscriber('/drone_nav/odom', Odometry, self.on_odom, queue_size=25)
        self.attempted = set()
        self.discovered = set()
        self.counts = Counter()

    def on_odom(self, msg):
        with self.pose_lock:
            if msg.header.frame_id != 'drone_odom' or msg.child_frame_id != 'drone_nav_base':
                self.poses.clear()
                return
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            self.poses.accept(msg.header.stamp.to_sec(), msg.header.frame_id,
                              (p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def pose_at(self, stamp):
        with self.pose_lock:
            return self.poses.lookup(stamp.to_sec(), .20)

    def make_route_map(self):
        goals, yaws, _ = super().make_route_map()
        if (Path(self.cfg['map_id']).name != self.route['map_id']
                or self.cfg.get('allow_unknown_above') is not False
                or self.cfg.get('allow_short_goals') is not True):
            raise Stopped('test4 需要飞扫图、拒绝未知区域和短目标支持')
        recent = wait_until(lambda: self.vision.latest, 1.5, self.guard, rospy.Time.now())
        if recent is None:
            raise Stopped('AprilTag 图像或标定未就绪')
        return goals, yaws, tuple(point['name'] for point in self.route['points'])

    def freeze(self):
        goals = super().freeze()
        metadata = {}
        for workspace in (Path(__file__).resolve().parents[2], Path.home() / 'orange'):
            def git(*args):
                return subprocess.check_output(['git', '-C', str(workspace), *args], text=True).strip()
            metadata[workspace.name] = dict(commit=git('rev-parse', 'HEAD'), status=git('status', '--short'))
        self.record.update(
            map_sha256=self.route['map_sha256'], versions=metadata,
            configuration={p.name: dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
                                         yaml=p.read_text()) for p in self.config_paths},
            navigation_config=self.cfg, fixed_route=self.route, batches=[], scans=[])
        self.save()
        return goals

    def guard(self, **kwargs):
        super().guard(**kwargs)
        if self.reference is not None:
            r, t = self.reference
            if (r @ self.s.pose()[0] + t)[2] > self.route['max_control_z']:
                raise Stopped('超过 test4 地图高度上限 %.2f m' % self.route['max_control_z'])

    def stopped(self):
        self.guard(stopped=True)

    def skip_leg(self, i, goal, yaw):
        return False

    def hold_for(self, i):
        return self.hold_seconds if i < len(self.route['points']) else 1.

    def send_leg(self, i, goal, goal_map, yaw_map):
        # The optimizer can detour for regular goals. A short spline traces the
        # direct segment, so check the full segment against the same live grid.
        self.check_segments = np.linalg.norm(self.s.pose()[0] - goal) < .20
        return super().send_leg(i, goal, goal_map, yaw_map)

    def capture(self, frame, index):
        if frame is None:
            return dict(error='no_fresh_full_resolution_frame')
        path = self.out / ('%02d_%s.png' % (index + 1, self.route['points'][index]['name']))
        temp = path.with_suffix('.tmp.png')
        try:
            img = np.frombuffer(frame.data, np.uint8).reshape(SIZE[1], frame.step)[:, :SIZE[0]]
            if path.exists() or not cv2.imwrite(str(temp), img, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
                raise OSError('cannot save observation photograph: ' + path.name)
            self.stopped()
            temp.replace(path)
        finally:
            temp.unlink(missing_ok=True)
        photo = dict(file=path.name, stamp_ns=frame.header.stamp.to_nsec(), frame_id=FRAME)
        nav = self.pose_at(frame.header.stamp)
        if nav is not None:
            r, t = self.reference
            control = r @ nav.position + t
            camera = control + r @ rotation(nav.quaternion) @ (self.cam_translation - self.lever)
            photo.update(control_xyz_map=control.tolist(), camera_xyz_map=camera.tolist(),
                         body_quat_odom_xyzw=list(nav.quaternion))
        else:
            photo['pose_error'] = 'no time-aligned odometry for photo'
        return photo

    def observe(self, frames, history):
        samples = defaultdict(dict)
        for msg in frames:
            stamp = msg.header.stamp.to_nsec()
            nav = self.pose_at(msg.header.stamp)
            ids = Counter(tag.id for tag in msg.tags)
            for tag in msg.tags:
                key = (msg.family, tag.id)
                self.discovered.add(key)
                rms = float(tag.reprojection_rms_px)
                sample = dict(id=tag.id, family=msg.family, stamp_ns=stamp,
                              reprojection_rms_px=rms if math.isfinite(rms) else None)
                history.append(sample)
                try:
                    if (ids[tag.id] != 1 or nav is None or not math.isfinite(tag.reprojection_rms_px)
                            or tag.reprojection_rms_px > 2.):
                        raise ValueError('duplicate ID, missing aligned odometry or reprojection RMS >2 px')
                    point, orientation, camera = map_tag(nav, tag.pose, self.reference,
                                                         self.cam_translation, self.cam_rotation, self.lever)
                except ValueError as error:
                    sample['error'] = str(error)
                    continue
                sample.update(xyz_map=point.tolist(), rotation_map=orientation.tolist(), camera_map=camera.tolist())
                samples[key][stamp] = point, orientation
        return samples

    def target(self, samples):
        point, orientation = consolidate(list(samples.values()))
        g, odom, angle, camera, paper = scan_target(point, orientation, self.layout['label'],
                                                    self.reference, self.cam_translation - self.lever,
                                                    math.radians(self.route['yaw_deg']))
        details = dict(tag_xyz_map=point.tolist(), tag_rotation_map=orientation.tolist(),
                       goal_map=g.tolist(), goal_odom=odom.tolist(), yaw_map_deg=math.degrees(angle),
                       camera_map=camera.tolist(), paper_map=paper.tolist(), standoff_m=STANDOFF)
        return g, odom, angle, details

    def precheck(self, g, odom, current=None):
        self.stopped()
        if g[2] > self.route['max_control_z']:
            raise ValueError('above test4 height limit')
        check_goal_bounds([odom], self.cfg, ['scan goal'])
        points = [current, odom] if current is not None and np.linalg.norm(current - odom) < .20 else [odom]
        response = self.check(path_of(points))
        if not response.valid:
            raise ValueError(response.reason)
        self.stopped()

    def report(self, index, state, key=None, route=None):
        message = dict(observation=self.route['points'][index]['name'], phase=state,
                       tagId=key[1] if key else None, discovered=len(self.discovered),
                       success=self.counts['ok'], failed=self.counts['failed'])
        if route is not None:
            message['route'] = route
        say('INDOOR4 ' + json.dumps(message, ensure_ascii=False, separators=(',', ':')))

    def hold_at_point(self, i, leg, seconds):
        if i >= len(self.route['points']):
            self.pause(seconds, stopped=True)
            return
        self.report(i, '识别标签')
        self.pause(1., stopped=True)
        start = rospy.Time.now()
        end = start + rospy.Duration(2.)
        photo = self.vision.take_photo(start, self.stopped)
        self.pause(max(0., (end - rospy.Time.now()).to_sec()) + .20, stopped=True)
        batch = dict(observation=i + 1, name=self.route['points'][i]['name'],
                     photo=self.capture(photo, i), observations=[], candidates=[], order=[])
        self.record['batches'].append(batch)
        frames = self.vision.window(start, end)
        batch['frames'] = len(frames)
        if len({frame.header.stamp.to_nsec() for frame in frames}) < 3:
            self.save()
            raise Stopped('观察窗口内 AprilTag 相机数据断流')
        samples = self.observe(frames, batch['observations'])
        observed = {(item['family'], item['id']) for item in batch['observations']}
        targets = {}
        for key in sorted(observed):
            entry = dict(family=key[0], id=key[1])
            batch['candidates'].append(entry)
            if key in self.attempted:
                entry['status'] = 'already_attempted'
                continue
            try:
                goal, odom, angle, details = self.target(samples.get(key, {}))
                entry.update(details)
                self.precheck(goal, odom)
            except ValueError as error:
                entry.update(status='skipped', reason=str(error))
                continue
            entry['status'] = 'planned'
            targets[key] = goal, odom, angle, entry
        r, t = self.reference
        next_index = min(i + 1, len(self.route['points']) - 1)
        finish = np.asarray(self.route['points'][next_index]['xyz'])
        start_map = r @ self.s.pose()[0] + t
        order, distance = shortest_order(start_map, {k: v[0] for k, v in targets.items()}, finish)
        batch.update(order=[k[1] for k in order], distance_m=distance,
                     start_map=start_map.tolist(), finish_map=finish.tolist())
        displayed = [start_map.tolist(), *[targets[k][0].tolist() for k in order], finish.tolist()]
        self.save()
        self.report(i, '扫描规划完成' if order else '无可扫描的新标签', route=dict(points=displayed, ids=[key[1] for key in order]))
        moved = False
        for key in order:
            g, odom, angle, entry = targets[key]
            try:
                self.precheck(g, odom, self.s.pose()[0])
            except ValueError as error:
                entry.update(status='skipped', reason=str(error))
                self.save()
                continue
            self.report(i, '前往读码点', key)
            self.fly_extra('Tag %d 读码点' % key[1], g, odom, angle)
            moved = True
            self.attempted.add(key)
            self.report(i, '读取 QR', key)
            result = dict(family=key[0], id=key[1], observation=i + 1,
                          goal_map=g.tolist(), started_ns=rospy.Time.now().to_nsec(), status='running')
            self.record['scans'].append(result)
            self.save()
            try:
                result.update(self.decode_once(key))
            except BaseException as error:
                result.update(status='aborted', reason=str(error))
                raise
            finally:
                self.save()
            self.counts['ok' if result['status'] == 'success' else 'failed'] += 1
            self.report(i, '读码成功' if result['status'] == 'success' else '读码失败', key)
        if next_index == i and moved:
            self.report(i, '返回右下悬停')
            self.fly_extra('返回右下观察点', finish, r.T @ (finish - t), math.radians(self.route['yaw_deg']))

    def fly_extra(self, name, g, odom, angle):
        i = len(self.names)
        self.names += (name,)
        self.reprepare()
        self.send_leg(i, odom, g, angle)

    def decode_once(self, key):
        done = threading.Event()
        result = []

        def decode():
            try:
                result.append(self.vision.decode())
            except rospy.ServiceException as error:
                result.append(error)
            finally:
                done.set()

        self.vision.select(key)
        worker = None
        try:
            # Give the downscaled stream one frame to populate the target pose.
            self.pause(.25, stopped=True)
            start = rospy.Time.now()
            worker = threading.Thread(target=decode, daemon=True)
            worker.start()
            deadline = time.monotonic() + 6.
            while not done.wait(.025):
                self.stopped()
                if time.monotonic() >= deadline:
                    raise Stopped('读码服务超时')
            self.stopped()
            if not result or isinstance(result[0], Exception):
                return dict(status='failed', reason='decode_service_error: ' + str(result[0] if result else 'no response'))
            response = result[0]
            record = dict(frame_ns=response.header.stamp.to_nsec(), elapsed_s=response.elapsed_s,
                          decoder_report=response.report, session_dir=response.session_dir)
            if response.success and response.used_tag_roi and response.header.frame_id == FRAME and response.header.stamp >= start:
                pose = self.vision.matched_pose(response.header.stamp)
                if len(response.codes) == 1 and pose and self.code_on_label(response.codes[0], pose):
                    code = response.codes[0]
                    return dict(record, status='success', qr=code.code, raw_hex=code.raw_hex,
                                decode_strategy=code.source)
            return dict(record, status='failed', reason=response.error or 'QR not associated with requested tag')
        finally:
            self.vision.select(None)
            if worker is not None and not done.is_set():
                cancel = threading.Thread(target=self.cancel_decode, daemon=True)
                cancel.start()
                cancel.join(timeout=.3)

    def cancel_decode(self):
        try:
            self.vision.cancel()
        except rospy.ServiceException as error:
            say('读码取消服务异常：' + str(error))

    def code_on_label(self, code, pose):
        info = self.vision.info
        if code.format != 'QRCode' or info is None:
            return False
        p, q = pose.pose.position, pose.pose.orientation
        rvec = cv2.Rodrigues(rotation([q.x, q.y, q.z, q.w]))[0]
        tvec = np.array([p.x, p.y, p.z])
        label = self.layout['label']
        w = label['width_meters'] / 2.
        h = label['height_meters'] / 2.
        x, y = label['tag_offset_x_meters'], label['tag_offset_y_meters']
        # Check the code against the physical paper above the tag, not the
        # larger padded ROI the decoder uses for detection.
        bottom, top = self.layout['target_marker_size_m'] / 2., h - y
        corners = np.array([[-w-x, bottom, 0], [w-x, bottom, 0], [w-x, top, 0], [-w-x, top, 0]])
        area, _ = cv2.projectPoints(corners, rvec, tvec, np.asarray(info.K).reshape(3, 3), np.asarray(info.D))
        pixels = np.asarray(code.corners).reshape(4, 2)
        return (np.isfinite(pixels).all() and
                all(cv2.pointPolygonTest(area.astype(np.float32), tuple(p), True) >= -3. for p in pixels)
                and abs(cv2.contourArea(pixels.astype(np.float32))) > 64.)


def probe(out):
    rospy.init_node('indoor4_camera_probe', anonymous=True)
    vision = Vision()
    rospy.wait_for_service(DECODE, timeout=10.)
    rospy.wait_for_service(CANCEL_DECODE, timeout=3.)
    now = rospy.Time.now()
    if (wait_until(lambda: vision.latest, 6., lambda: None, now) is None
            or wait_until(lambda: vision.info, 2., lambda: None, now) is None
            or vision.take_photo(now, lambda: None) is None):
        raise Stopped('OV13855 双路图像或标定不可用')
    if shutil.disk_usage(out).free < 1024 ** 3:
        raise Stopped('test4 剩余空间不足以保存相片与扫码记录')
    say('test4 相机就绪：1280×960 AprilTag、4224×3136 QR')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--probe-camera', action='store_true')
    args = parser.parse_args()
    if args.probe_camera:
        return probe(args.out)
    route = load_route()
    return serve('nav_indoor4', SimpleNamespace(
        route=route, points=[(*p['xyz'], route['yaw_deg']) for p in route['points']],
        hold=route['hold_s'], check_segments=False, delay=3., max_start_offset=math.inf,
        out=str(args.out.resolve()), preview=args.preview), Indoor4Runner)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (Stopped, rospy.ROSException) as error:
        say(str(error))
        raise SystemExit(1)
