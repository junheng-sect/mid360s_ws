#!/usr/bin/env python3
"""Observe shelf AprilTags at the four indoor photo stops."""
import argparse
import csv
from contextlib import ExitStack
from collections import defaultdict, deque
from pathlib import Path
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from ov13855_apriltag_vision.msg import TagSurveyFrame
from tf.transformations import quaternion_from_matrix, quaternion_multiply
from front_marker_control.localization import LocalPoseBuffer

from indoor3_runner import Frame, Indoor3Runner, nearest_photo, probe_photo, save_photo, serve_route, wait_for_frame
from nav_forward import say
from nav_indoor3 import load_route
from nav_square import Stopped


TOPIC = '/ov13855_apriltag_vision/survey'
FRAME = 'ov13855_optical_frame'
FAMILY = 'tag36h11'
EXPECTED = set(range(6))
WINDOW_S = 2.
# The configured stream is 15 Hz. Count empty arrays as live frames.
MAX_FRAME_GAP_NS = 500_000_000
FIELDS = ('waypoint', 'waypoint_name', 'family', 'id', 'first_stamp_ns', 'last_stamp_ns',
          'seen_frames', 'window_frames', 'representative_stamp_ns', 'pose_status', 'tag_pose_status',
          'reprojection_rms_px', 'photo_file', 'photo_stamp_ns')
COMPONENTS = ('x_m', 'y_m', 'z_m', 'qx', 'qy', 'qz', 'qw')
POSE_FIELDS = tuple(prefix + '_' + field for prefix in ('tag_camera', 'drone_odom', 'drone_map')
                    for field in COMPONENTS)


def frame_problem(msg, frame_id):
    if msg.header.frame_id != frame_id:
        return 'wrong_frame'
    if msg.family != FAMILY:
        return 'wrong_family'
    if msg.header.stamp.to_nsec() <= 0:
        return 'zero_stamp'
    age = (rospy.Time.now() - msg.header.stamp).to_sec()
    if age < -.02:
        return 'future_stamp'
    if age > .5:
        return 'stale_stamp'
    return None


def valid_frame(msg, frame_id=FRAME):
    return frame_problem(msg, frame_id) is None


def pose_values(pose):
    p, q = pose.position, pose.orientation
    return (p.x, p.y, p.z, q.x, q.y, q.z, q.w)


def waypoint_rows(frames, waypoint, name, photo, pose_at, reference):
    samples = defaultdict(dict)
    for stamp, msg in sorted(frames.items()):
        poses = {tag.id: tag for tag in msg.tags}
        for tag_id in set(msg.ids):
            samples[tag_id][stamp] = poses.get(tag_id)
    rotation, translation = reference
    transform = np.eye(4)
    transform[:3, :3] = rotation
    map_quat = quaternion_from_matrix(transform)
    rows = []
    for tag_id, observations in sorted(samples.items()):
        first = min(observations)
        row = dict(waypoint=waypoint, waypoint_name=name, family=FAMILY, id=tag_id,
                   first_stamp_ns=first, last_stamp_ns=max(observations),
                   seen_frames=len(observations), window_frames=len(frames),
                   photo_file=photo['file'], photo_stamp_ns=photo['stamp_ns'],
                   pose_status='no_synchronized_odometry')
        stamp = first
        for candidate in observations:
            nav = pose_at(candidate)
            if nav is not None:
                stamp = candidate
                xyz = rotation @ nav.position + translation
                q = quaternion_multiply(map_quat, nav.quaternion)
                for prefix, values in (('drone_odom', (*nav.position, *nav.quaternion)),
                                       ('drone_map', (*xyz, *q))):
                    row.update(zip((prefix + '_' + f for f in COMPONENTS), map(float, values)))
                row['pose_status'] = 'aligned'
                break
        tag = observations[stamp]
        row.update(representative_stamp_ns=stamp, tag_pose_status='unavailable' if tag is None else 'solved')
        if tag is not None:
            row['reprojection_rms_px'] = float(tag.reprojection_rms_px)
            row.update(zip(('tag_camera_' + f for f in COMPONENTS), pose_values(tag.pose)))
        rows.append(row)
    return rows


def coverage(rows):
    ids = {row['id'] for row in rows}
    seen = sorted(EXPECTED & ids)
    return dict(seen=seen, missing=sorted(EXPECTED - ids), extra=sorted(ids - EXPECTED),
                count=len(seen), expected=sorted(EXPECTED))


class SurveyRunner(Indoor3Runner):
    test_name = record_prefix = 'test_indoor3_1'
    route_name = '室内 test3_1 四点 AprilTag 识别'

    cameras = {'ov13855': (TOPIC, FRAME)}
    csv_fields = FIELDS + POSE_FIELDS

    def __init__(self, *args):
        super().__init__(*args)
        self.lock = threading.Lock()
        self.frames = {name: deque(maxlen=150) for name in self.cameras}
        self.poses = LocalPoseBuffer(500)
        self.detections = [
            rospy.Subscriber(topic, TagSurveyFrame, self.on_frame, callback_args=name, queue_size=64)
            for name, (topic, _) in self.cameras.items()]
        self.odometry = rospy.Subscriber('/drone_nav/odom', Odometry, self.on_odom, queue_size=50)
        self.csv_path = Path(self.a.out) / 'apriltag_observations.csv'

    def on_frame(self, msg, name):
        if valid_frame(msg, self.cameras[name][1]):
            with self.lock:
                self.frames[name].append(msg)

    def on_odom(self, msg):
        with self.lock:
            if msg.header.frame_id != 'drone_odom' or msg.child_frame_id != 'drone_nav_base':
                self.poses.clear()
                return
            p = pose_values(msg.pose.pose)
            self.poses.accept(msg.header.stamp.to_sec(), msg.header.frame_id, p[:3], p[3:])

    def freeze(self):
        if self.csv_path.exists():
            raise Stopped('任务目录中已有 ' + self.csv_path.name)
        goals = super().freeze()
        with self.lock:
            for name, (_, frame_id) in self.cameras.items():
                if not self.frames[name] or not valid_frame(self.frames[name][-1], frame_id):
                    raise Stopped(name + ' AprilTag 检测流尚未就绪')
        self.record.update(csv_file=self.csv_path.name,
                           map_sha256=self.a.route['map_sha256'], observations=[])
        self.write_results()
        return goals

    def total_coverage(self, rows):
        return coverage(rows)

    def store_window(self, observation, stats):
        observation.update(stats['ov13855'])

    def progress_text(self, stats):
        result = self.record['coverage']
        return '%s｜已见 %d/6 %s｜缺失 %s｜额外 %s' % (
            stats['ov13855']['ids'], result['count'], result['seen'], result['missing'], result['extra'])

    def write_results(self):
        rows = self.record['observations']
        self.record['coverage'] = self.total_coverage(rows)
        self.save()
        temp = self.csv_path.with_suffix('.tmp.csv')
        try:
            with temp.open('w', newline='', encoding='utf-8') as stream:
                writer = csv.DictWriter(stream, fieldnames=self.csv_fields)
                writer.writeheader()
                writer.writerows(rows)
            temp.replace(self.csv_path)
        finally:
            temp.unlink(missing_ok=True)

    def hold_at_point(self, i, leg, seconds):
        name = self.a.route['points'][i]['name']
        path = self.csv_path.parent / ('%02d_%s.png' % (i + 1, name))
        paths = {'ov13855': path}
        if 'ov9281' in self.cameras:
            paths['ov9281'] = path.with_name(path.stem + '_ov9281.png')
        observation = leg['observation'] = dict(status='settling')
        try:
            for camera, target in paths.items():
                if target.exists():
                    raise Stopped('%s 照片已存在：%s' % (camera.upper(), target.name))
            with ExitStack() as subscriptions:
                streams = {}
                for camera in paths:
                    streams[camera] = Frame(camera)
                    subscriptions.callback(streams[camera].close)
                self.pause(1., stopped=True)
                start = rospy.Time.now()
                end = start + rospy.Duration(WINDOW_S)
                observation.update(status='sampling', start_ns=start.to_nsec(), end_ns=end.to_nsec())
                deadline = time.monotonic() + WINDOW_S
                upper = wait_for_frame(streams['ov13855'], start, deadline, self.hold_guard)
                streams['ov13855'].close()
                if upper.header.stamp >= end:
                    raise Stopped('观察窗口内没有 OV13855 全分辨率新帧')
                images = {'ov13855': upper}
                if 'ov9281' in streams:
                    images['ov9281'] = nearest_photo(streams['ov9281'], upper.header.stamp,
                                                     start, end, deadline, self.hold_guard)
            # Allow in-flight detections and bracketing odometry to arrive, but
            # only accept source stamps inside the original two-second window.
            self.pause(max(0., deadline - time.monotonic()) + .5, stopped=True)
            photos = {camera: dict(file=paths[camera].name, stamp_ns=image.header.stamp.to_nsec(),
                                  frame_id=image.header.frame_id) for camera, image in images.items()}
            stats = {}
            rows = []
            with self.lock:
                for camera, messages in self.frames.items():
                    frames = {m.header.stamp.to_nsec(): m for m in messages if start <= m.header.stamp < end}
                    hits = waypoint_rows(frames, i + 1, name, photos[camera],
                                         lambda ns: self.poses.lookup(ns / 1e9, .20), self.reference)
                    if 'camera' in self.csv_fields:
                        for row in hits:
                            row['camera'] = camera
                    stamps = [start.to_nsec(), *sorted(frames), end.to_nsec()]
                    gap = max(b - a for a, b in zip(stamps, stamps[1:]))
                    stats[camera] = dict(frames=len(frames), max_gap_ns=gap, ids=[r['id'] for r in hits])
                    rows.extend(hits)
            saved = []
            try:
                for camera, image in images.items():
                    try:
                        save_photo(image, paths[camera], self.hold_guard)
                    except Stopped as error:
                        raise Stopped('%s 拍照失败：%s' % (camera.upper(), error)) from error
                    saved.append(paths[camera])
            except BaseException:
                for target in saved:
                    target.unlink(missing_ok=True)
                raise
            leg['photo'] = photos['ov13855']
            if 'ov9281' in photos:
                leg['photos'] = photos
                leg['photo_delta_ms'] = abs(photos['ov9281']['stamp_ns'] - photos['ov13855']['stamp_ns']) / 1e6
            self.store_window(observation, stats)
            faults = [camera for camera, item in stats.items() if item['max_gap_ns'] > MAX_FRAME_GAP_NS]
            observation['status'] = 'stream_fault' if faults else 'complete'
            self.record['observations'].extend(rows)
            self.write_results()
            if faults:
                raise Stopped('%s AprilTag 检测断流：%s' % (name, ', '.join(faults)))
            say('AprilTag 观察 %d/4 %s %s' % (i + 1, name, self.progress_text(stats)))
        except BaseException as error:
            if observation['status'] != 'stream_fault':
                observation['status'] = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'failed'
            observation['reason'] = '%s：%s' % (name, str(error) or type(error).__name__)
            if isinstance(error, Stopped):
                raise Stopped(observation['reason']) from error
            raise


class DualSurveyRunner(SurveyRunner):
    test_name = record_prefix = 'test_indoor3_2'
    route_name = '室内 test3_2 四点双相机 AprilTag 识别'
    cameras = dict(SurveyRunner.cameras, ov9281=('/ov9281_apriltag_vision/survey', 'usb_camera_optical_frame'))
    csv_fields = ('camera',) + SurveyRunner.csv_fields

    def total_coverage(self, rows):
        return {camera: coverage(row for row in rows if row['camera'] == camera) for camera in self.cameras}

    def store_window(self, observation, stats):
        observation['cameras'] = stats

    def progress_text(self, stats):
        return '｜'.join('%s %s (%d 帧)，累计 %d/6，缺 %s' % (
            camera.upper(), item['ids'], item['frames'], self.record['coverage'][camera]['count'],
            self.record['coverage'][camera]['missing']) for camera, item in stats.items())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--probe-camera', action='store_true')
    parser.add_argument('--dual-camera', action='store_true')
    args = parser.parse_args()
    runner = DualSurveyRunner if args.dual_camera else SurveyRunner
    if args.probe_camera:
        rospy.init_node(runner.test_name + '_camera_probe', anonymous=True)
        probe_photo(args.out, runner.cameras)
        for camera, (topic, frame_id) in runner.cameras.items():
            deadline = time.monotonic() + 6.
            while time.monotonic() < deadline:
                try:
                    msg = rospy.wait_for_message(topic, TagSurveyFrame, timeout=max(.01, deadline - time.monotonic()))
                except rospy.ROSException as error:
                    raise Stopped(camera + ' AprilTag 检测流尚未就绪') from error
                if valid_frame(msg, frame_id):
                    break
            else:
                raise Stopped(camera + ' AprilTag 检测流尚未就绪')
            say(camera + ' AprilTag 检测流就绪')
        return 0
    return serve_route('nav_' + runner.test_name[len('test_'):], args, load_route(), runner)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (Stopped, rospy.ROSException) as error:
        say(str(error))
        raise SystemExit(1)
