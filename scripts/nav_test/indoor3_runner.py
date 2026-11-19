"""Shared camera and fixed shelf route for indoor test3 and test3_1."""
from collections import deque
import math
import shutil
import time
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image

from nav_global import GlobalRunner, serve
from nav_square import Stopped
from nav_forward import say
from drone_navigation.core import rotation


width, height = 4224, 3136


# Raw mono images from the two mounted cameras. Only the lower stream needs
# a short history to bracket the selected full-resolution upper frame.
PHOTO_STREAMS = {
    'ov13855': ('/ov13855_full/image_raw', 'ov13855_optical_frame', width, height),
    'ov9281': ('/usb_cam/image_raw', 'usb_camera_optical_frame', 1280, 800),
}


class Frame:
    def __init__(self, camera='ov13855'):
        self.camera = camera
        topic, self.frame_id, self.width, self.height = PHOTO_STREAMS[camera]
        self.recent = deque(maxlen=16 if camera == 'ov9281' else 1)
        self.subscription = rospy.Subscriber(topic, Image, self.receive, queue_size=1,
                                             buff_size=2 * self.width * self.height)

    def receive(self, msg):
        if (msg.width == self.width and msg.height == self.height and msg.encoding == 'mono8'
                and msg.header.frame_id == self.frame_id and msg.step >= self.width
                and len(msg.data) == msg.step * msg.height and msg.header.stamp.to_nsec() > 0):
            self.recent.append(msg)

    def close(self):
        if self.subscription is not None:
            self.subscription.unregister()
            self.subscription = None


def wait_for_frame(frame, since, deadline, guard):
    while not rospy.is_shutdown():
        guard()
        msg = frame.recent[-1] if frame.recent else None
        if msg is not None and msg.header.stamp > since:
            age = (rospy.Time.now() - msg.header.stamp).to_sec()
            if -.02 <= age <= .25:
                return msg
        left = deadline - time.monotonic()
        if left <= 0:
            break
        time.sleep(min(.03, left))
    raise Stopped('未收到带有效时间戳的 %s 新帧' % frame.camera.upper())


def nearest_photo(frame, anchor, start, end, deadline, guard):
    # The single USB publisher stamps images in acquisition order. Once we
    # have crossed the anchor we have both neighbours (if both are in-window).
    best = None
    while not rospy.is_shutdown():
        guard()
        recent = frame.recent.copy()
        for msg in recent:
            stamp = msg.header.stamp
            if start <= stamp < end and (best is None or
                    abs((stamp - anchor).to_nsec()) < abs((best.header.stamp - anchor).to_nsec())):
                best = msg
        if recent and recent[-1].header.stamp >= anchor and best is not None:
            return best
        left = deadline - time.monotonic()
        if left <= 0:
            break
        time.sleep(min(.01, left))
    if best is None or rospy.is_shutdown():
        raise Stopped('观察窗口内没有有效的 %s 照片帧' % frame.camera.upper())
    return best


def save_photo(msg, path, guard):
    temp = path.with_suffix('.tmp.png')
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
    try:
        if not cv2.imwrite(str(temp), img, [cv2.IMWRITE_PNG_COMPRESSION, 1]):
            raise OSError('PNG 保存失败')
        guard()
        temp.replace(path)
    except (OSError, cv2.error) as error:
        raise Stopped('%s 写盘失败：%s' % (path.name, error)) from error
    finally:
        temp.unlink(missing_ok=True)


def probe_photo(out, cameras=('ov13855',)):
    required = 0
    for camera in cameras:
        frame = Frame(camera)
        try:
            wait_for_frame(frame, rospy.Time.now(), time.monotonic() + 12., lambda: None)
        finally:
            frame.close()
        required += 4 * (frame.width * frame.height + 1024 * 1024)
        say('%s 照片流已就绪：%d×%d mono8' % (camera.upper(), frame.width, frame.height))
    if shutil.disk_usage(out).free < required:
        raise Stopped('磁盘空间不足以保存四个点位的 PNG')
    return 0


class Indoor3Runner(GlobalRunner):
    test_name = record_prefix = 'test_indoor3'
    route_name = '室内 test3 四点拍照'
    side_m = None
    repeat_laps = False

    def make_route_map(self):
        goals, yaws, _ = super().make_route_map()
        if Path(self.cfg['map_id']).name != self.a.route['map_id']:
            raise Stopped('重定位地图不匹配 test3')
        if self.cfg.get('allow_unknown_above') is not False:
            raise Stopped('test3 要求室内未知区域拒绝策略')
        frame = Frame()
        try:
            wait_for_frame(frame, rospy.Time.now(), time.monotonic() + 1.5, self.guard)
        finally:
            frame.close()
        return goals, yaws, tuple(point['name'] for point in self.a.route['points'])

    def guard(self, **kwargs):
        super().guard(**kwargs)
        if self.reference is not None:
            r, t = self.reference
            p = self.s.pose()
            if p is None or (r @ p[0] + t)[2] > self.a.route['max_control_z']:
                raise Stopped('test3 控制点超过地图高度上限 %.2f m' % self.a.route['max_control_z'])

    def skip_leg(self, i, goal, yaw):
        pose = self.s.pose()
        if pose is None or np.linalg.norm(pose[0] - goal) < .20:
            raise Stopped('第 %d 点无法启动独立航段（里程计无效或距离 <0.20 m）' % (i + 1))
        return False

    def hold_at_point(self, i, leg, seconds):
        path = Path(self.a.out) / ('%02d_%s.png' % (i + 1, self.a.route['points'][i]['name']))
        if path.exists():
            raise Stopped('照片文件已存在：' + path.name)
        frame = Frame()
        started = time.monotonic()
        guard = self.hold_guard
        try:
            self.pause(1., stopped=True)
            msg = wait_for_frame(frame, rospy.Time.now(), started + 2.5, guard)
            odom = self.s.odom
            p = odom.pose.pose.position
            q = odom.pose.pose.orientation
            if abs((msg.header.stamp - odom.header.stamp).to_sec()) > .25:
                raise Stopped('拍摄帧与导航位姿时间差超过 0.25 s')
            r, t = self.reference
            nav_odom = np.array([p.x, p.y, p.z])
            nav_map = r @ nav_odom + t
            cam_map = nav_map + r @ rotation([q.x, q.y, q.z, q.w]) @ (
                np.array([.18, 0., 0.]) - np.asarray(self.cfg['lever_body']))
            save_photo(msg, path, guard)
            leg['photo'] = dict(file=path.name, frame_id=msg.header.frame_id,
                                stamp_ns=msg.header.stamp.to_nsec(), pose_stamp_ns=odom.header.stamp.to_nsec(),
                                control_xyz_odom=nav_odom.tolist(), control_xyz_map=nav_map.tolist(),
                                camera_xyz_map=cam_map.tolist(), body_quat_odom_xyzw=[q.x, q.y, q.z, q.w])
            self.save()
            say('已保存 %s' % path.name)
            self.pause(max(0., started + seconds - time.monotonic()), stopped=True)
        finally:
            frame.close()

    def hold_guard(self):
        self.guard(stopped=True)


def serve_route(node, args, route, runner_class):
    return serve(node, SimpleNamespace(
        route=route, points=[(*p['xyz'], route['yaw_deg']) for p in route['points']],
        hold=route['hold_s'], check_segments=True, delay=3., max_start_offset=math.inf,
        out=str(args.out.resolve()), preview=args.preview), runner_class)
