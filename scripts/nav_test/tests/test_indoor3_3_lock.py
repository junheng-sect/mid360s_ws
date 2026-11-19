#!/usr/bin/env python3
"""Drive the production lock loop against deterministic source stamps, without an FCU."""
from collections import deque
from pathlib import Path
from types import SimpleNamespace
from unittest import TestCase, main, mock
import math
import sys
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from ov13855_apriltag_vision.msg import TagObservation, TagSurveyFrame
from std_msgs.msg import Header
from front_marker_control.localization import LocalPoseBuffer

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import nav_indoor3_3 as mission
from nav_square import Stopped


class LockLogTest(TestCase):
    def simulate(self, fault, *, moving=False):
        runner = object.__new__(mission.Runner)
        runner.lock = threading.Lock()
        runner.frames = deque(maxlen=160)
        runner.poses = LocalPoseBuffer(250)
        runner.reference = np.eye(3), np.zeros(3)
        runner.translation = runner.lever = np.zeros(3)
        # Optical +Z points forwards, optical -Y points up.
        runner.camera_rotation = np.array([[0,0,1], [-1,0,0], [0,-1,0]])
        runner.trial = dict(status='running')
        runner.s = SimpleNamespace(pose=lambda: (None, None, .07 if moving else .0))
        runner.lock_trace = None
        tick = 0

        def now():
            return rospy.Time(100, tick * 25_000_000)

        def guard():
            if fault == 'interrupt' and tick == 12:
                runner.trial['status'] = 'interrupted'
                raise Stopped('切出 OFFBOARD')

        def publish(_):
            nonlocal tick
            tick += 1
            stamp = now()
            if fault == 'drop':
                return
            tag = TagObservation(id=1, pose=Pose(Point(0., 0., .8), Quaternion(1.,0.,0.,0.)),
                                 reprojection_rms_px=.3)
            msg = TagSurveyFrame(header=Header(stamp=stamp, frame_id=mission.FRAME), family='tag36h11',
                                 ids=[1], tags=[tag])
            if fault == 'no_tag':
                msg.ids, msg.tags = [3], []
            elif fault == 'no_pose':
                msg.tags = []
            elif fault == 'duplicate_pose':
                msg.tags.append(tag)
            elif fault == 'rms':
                tag.reprojection_rms_px = 2.5
            elif fault == 'nonfinite':
                tag.reprojection_rms_px = float('nan')
            elif fault == 'depth':
                tag.pose.position.z = -.1
            elif fault == 'back':
                tag.pose.orientation = Quaternion(0., 0., 0., 1.)
            elif fault == 'invalid':
                tag.pose.orientation = Quaternion(0., 0., 0., 2.)
            elif fault == 'tilt':
                angle = math.radians(22)/2
                tag.pose.orientation = Quaternion(math.cos(angle), 0., 0., math.sin(angle))
            elif fault == 'up':
                tag.pose.orientation = Quaternion(0., 1., 0., 0.)
            elif fault == 'positions':
                tag.pose.position.x = .25 if tick % 2 else -.25
            elif fault == 'normals':
                angle = math.radians(16 if tick % 2 else -16)/2
                tag.pose.orientation = Quaternion(math.cos(angle), 0., math.sin(angle), 0.)
            elif fault == 'wrong_frame':
                msg.header.frame_id = 'another_camera'
            elif fault == 'sparse' and tick % 15:
                msg.tags = []
            if fault != 'odom' and (fault != 'late_odom' or tick >= 28):
                # Supply odometry for a previously evaluated source frame.
                odom_stamp = stamp.to_sec() - .05 if fault == 'late_odom' else stamp.to_sec()
                runner.poses.accept(odom_stamp, 'drone_odom', [0.,0.,0.], [0.,0.,0.,1.])
            runner.on_tags(msg)
            runner.on_tags(msg)  # Repeat delivery must not count as two source frames.

        runner.guard = guard
        clock = SimpleNamespace(monotonic=lambda: tick * .025, sleep=publish)
        with mock.patch.object(mission, 'time', clock), mock.patch.object(rospy.Time, 'now', now), \
                mock.patch.object(mission, 'say'):
            try:
                runner.search_tag()
            except Stopped:
                pass
        self.assertIsNone(runner.lock_trace)
        info = runner.trial['lock']
        self.assertEqual(info['survey_frames'], len({f['stamp_ns'] for f in info['frames']}))
        self.assertEqual(sum(info['frame_counts'].values()), info['survey_frames'])
        self.assertEqual(info['duplicate_messages'], info['survey_frames'])
        return info

    def test_visibility_and_hover_before_pose_checks(self):
        moving = self.simulate('ok', moving=True)
        self.assertGreater(moving['tag1_frames'], 100)
        self.assertEqual(moving['tag1_frames'], moving['pose_frames'])
        self.assertEqual(moving['windows'], [])
        self.assertEqual(moving['max_stable_s'], 0.)
        self.assertIn('连续低速', moving['reason'])
        hidden = self.simulate('no_tag')
        self.assertEqual(hidden['outcome'], 'timeout')
        self.assertEqual(hidden['tag1_frames'], 0)
        unsolved = self.simulate('no_pose')
        self.assertGreater(unsolved['tag1_frames'], 100)
        self.assertEqual(unsolved['pose_frames'], 0)
        self.assertGreater(unsolved['frame_counts']['no_pose'], 100)

    def test_per_frame_filters(self):
        cases = [('duplicate_pose','duplicate_pose'), ('rms','reprojection'), ('nonfinite','reprojection'),
                 ('odom','odom_sync'), ('depth','behind_camera'), ('back','back_facing'),
                 ('invalid','invalid_pose'), ('tilt','normal_tilt'), ('up','tag_up')]
        for fault, code in cases:
            with self.subTest(fault=fault):
                info = self.simulate(fault)
                self.assertEqual(info['outcome'], 'timeout')
                self.assertGreater(info['frame_counts'][code], 100)
                self.assertEqual(info['candidate_frames'], 0)

    def test_window_filters_and_lock(self):
        sparse = self.simulate('sparse')
        self.assertGreater(sparse['candidate_frames'], 3)
        self.assertLess(sparse['max_window_frames'], 3)
        self.assertIn('too_few', sparse['window_counts'])
        for fault, reason in [('positions','position_spread'), ('normals','normal_spread')]:
            with self.subTest(fault=fault):
                info = self.simulate(fault)
                self.assertGreater(info['max_window_frames'], 3)
                self.assertGreater(info['window_counts'][reason], 0)
                self.assertGreater(next(w['value'] for w in info['windows'] if w['result']==reason), 0)
        info = self.simulate('late_odom')
        self.assertEqual(info['outcome'], 'locked')
        self.assertGreaterEqual(info['max_window_frames'], 3)
        self.assertGreater(info['window_counts']['too_few'], 0)
        self.assertTrue(any(f.get('waited_for_odom') and f['result'] == 'candidate'
                            for f in info['frames']))

    def test_stream_and_interruption(self):
        self.assertEqual(self.simulate('drop')['outcome'], 'stream_fault')
        wrong = self.simulate('wrong_frame')
        self.assertEqual(wrong['outcome'], 'stream_fault')
        self.assertGreater(wrong['dropped_messages']['wrong_frame'], 0)
        stopped = self.simulate('interrupt')
        self.assertEqual(stopped['outcome'], 'interrupted')
        self.assertEqual(stopped['tag1_frames'], 12)
        self.assertIn('OFFBOARD', stopped['reason'])


if __name__ == '__main__':
    main()
