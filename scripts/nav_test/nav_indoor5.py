#!/usr/bin/env python3
"""Indoor5: batch QR scans with position-only vision and an upright, map-fixed label."""
from collections import Counter, defaultdict, deque
import math

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from ov13855_apriltag_vision.msg import TagCornerArray
from sensor_msgs.msg import CameraInfo
from tf.transformations import quaternion_from_matrix

from indoor4_logic import STANDOFF, scan_target
from indoor5_geometry import fit_observation, paper_axes
from nav_indoor3 import load_route
from nav_indoor4 import FRAME, Indoor4Runner, Vision, run_cli
from nav_square import Stopped


def load_indoor5_route():
    route = load_route()
    route['max_control_z'] = 1.70
    return route


class CornerVision(Vision):
    detection_topic = '/ov13855_apriltag_vision/corners'
    detection_type = TagCornerArray
    target_topic = '/indoor5/target_tag_pose'

    def __init__(self):
        self.small_info = None
        self.pending = deque(maxlen=10)
        super().__init__()
        self.small_sub = rospy.Subscriber('/ov13855/camera_info', CameraInfo, self.on_small_info, queue_size=1)

    def on_small_info(self, info):
        valid = ((info.width, info.height) == (1280,960) and info.header.frame_id == FRAME
                 and info.K[0] > 0 and info.K[4] > 0 and np.isfinite((*info.K,*info.D)).all()
                 and info.distortion_model in ('plumb_bob','rational_polynomial')
                 and len(info.D) in (0,4,5,8,12,14))
        self.small_info = info if valid else None

    def ready_info(self):
        return self.info if self.small_info is not None else None

    def on_detections(self, msg):
        if (msg.width, msg.height) == (1280,960):
            super().on_detections(msg)

    def select(self, target):
        with self.lock:
            self.target = target
            self.since = rospy.Time.now()
            self.forwarded.clear()
            self.pending.clear()

    def on_target_frame(self, msg):
        # Called with self.lock held. Camera frames often arrive before the bracketing odometry.
        if self.target is not None and msg.header.stamp >= self.since:
            matches = [tag for tag in msg.tags if (msg.family,tag.id) == self.target]
            if len(matches) == 1:
                self.pending.append((self.since, msg.header, matches[0]))


class Indoor5Runner(Indoor4Runner):
    test_name = record_prefix = 'test_indoor5'
    route_name = '室内 test5 固定纸面分批扫码'
    test_label = 'test5'
    event_prefix = 'INDOOR5'
    vision_type = CornerVision

    def __init__(self, *args):
        super().__init__(*args)
        self.axes = paper_axes(math.radians(self.route['yaw_deg']))
        self.target_timer = rospy.Timer(rospy.Duration(.01), self.forward_pending)

    def measure(self, tag, pose):
        info = self.vision.small_info
        if info is None or pose is None or self.reference is None:
            raise ValueError('missing calibration or aligned odometry')
        return fit_observation([(p.x,p.y) for p in tag.corners], pose, self.reference,
                               self.cam_rotation, self.cam_translation - self.lever, self.axes,
                               self.layout['target_marker_size_m'], np.asarray(info.K).reshape(3,3), np.asarray(info.D))

    def forward_pending(self, _):
        vision = self.vision
        with vision.lock:
            if not vision.pending:
                return
            since, header, tag = vision.pending[0]
            stamp = header.stamp
            if (since != vision.since or vision.target is None or (rospy.Time.now()-stamp).to_sec() > .5):
                vision.pending.popleft()
                return
            pose = self.pose_at(stamp)
            if pose is None:
                # Do not let a frame older than the available odometry starve newer frames.
                with self.pose_lock:
                    newest = self.poses.newest_stamp
                if newest is not None and newest >= stamp.to_sec():
                    vision.pending.popleft()
                return
            vision.pending.popleft()
        try:
            center, orientation, _, _, rms = self.measure(tag, pose)
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            return
        if rms > 2.:
            return
        transform = np.eye(4)
        transform[:3,:3] = orientation
        msg = PoseStamped(header=header, pose=Pose(Point(*center), Quaternion(*quaternion_from_matrix(transform))))
        with vision.lock:
            if since == vision.since and vision.target == ('tag36h11',tag.id):
                vision.forwarded.append(msg)
                vision.pose_pub.publish(msg)

    def observe(self, frames, history):
        samples = defaultdict(dict)
        for frame in frames:
            pose = self.pose_at(frame.header.stamp)
            ids = Counter(tag.id for tag in frame.tags)
            for tag in frame.tags:
                key = (frame.family, tag.id)
                self.discovered.add(key)
                stamp = frame.header.stamp.to_nsec()
                pixels = [[p.x, p.y] for p in tag.corners]
                item = dict(family=frame.family, id=tag.id, stamp_ns=stamp,
                            corners_px=pixels if np.isfinite(pixels).all() else None)
                history.append(item)
                try:
                    if ids[tag.id] != 1:
                        raise ValueError('duplicate ID in frame')
                    center, _, point, camera, rms = self.measure(tag, pose)
                    item.update(center_camera=center.tolist(), xyz_map=point.tolist(), camera_map=camera.tolist(),
                                reprojection_rms_px=rms)
                    if rms > 2.:
                        raise ValueError('fixed-orientation reprojection RMS > 2 px')
                    samples[key][stamp] = point
                except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
                    item['error'] = str(error)
        return samples

    def target(self, samples):
        if len(samples) < 3:
            raise ValueError('fewer than 3 time-aligned observations')
        points = np.array(list(samples.values()))
        center = np.median(points,axis=0)
        if np.linalg.norm(points-center,axis=1).max() > .10:
            raise ValueError('tag positions vary by more than 0.10 m from the median')
        g, odom, angle, camera, paper = scan_target(center,self.axes,self.layout['label'],self.reference,
                                                   self.cam_translation-self.lever, math.radians(self.route['yaw_deg']))
        return g, odom, angle, dict(tag_xyz_map=center.tolist(), goal_map=g.tolist(), goal_odom=odom.tolist(),
                                    goal_height_map_m=float(g[2]), yaw_map_deg=math.degrees(angle),
                                    camera_map=camera.tolist(), paper_map=paper.tolist(), standoff_m=STANDOFF)

    def save(self):
        if self.record is not None:
            batches = self.record.get('batches', [])
            scans = self.record.get('scans', [])
            self.record['label_geometry'] = dict(model='fixed_upright_position_only', frame='drone_map',
                                                 axes=self.axes.tolist(), max_control_z=self.route['max_control_z'])
            passed = {c['id'] for b in batches for c in b['candidates'] if c.get('status') == 'planned'}
            settled = {leg['direction'] for leg in self.record.get('legs', []) if leg.get('end') == 'SUCCEEDED'}
            self.record['tag_counts'] = dict(discovered=len(self.discovered), passed_precheck=len(passed),
                arrived=sum('Tag %d 读码点' % key in settled for key in passed),
                read_ok=sum(s['status'] == 'success' for s in scans))
        super().save()


if __name__ == '__main__':
    try:
        raise SystemExit(run_cli(Indoor5Runner, load_indoor5_route, 'nav_indoor5'))
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (Stopped, rospy.ROSException) as error:
        rospy.logerr(str(error))
        raise SystemExit(1)
