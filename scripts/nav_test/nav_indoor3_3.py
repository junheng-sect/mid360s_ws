#!/usr/bin/env python3
"""One Tag 1 approach and paired QR scan per OFFBOARD rising edge."""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time
from collections import deque
from types import SimpleNamespace

import numpy as np
import rospkg
import rospy
import yaml
from nav_msgs.msg import Odometry
from ov13855_apriltag_vision.msg import TagSurveyFrame
from drone_navigation.core import finite, heading, rotation
from front_marker_control.localization import LocalPoseBuffer

from indoor3_runner import probe_photo, PHOTO_STREAMS
from indoor3_3_geometry import consolidate, fixed_scan_goal, map_tag
from indoor3_3_qr import paired_scan
from nav_forward import say
from nav_global import serve
from nav_indoor3 import load_route
from nav_indoor3_1 import pose_values, valid_frame
from nav_square import SquareRunner, Stopped


CONFIG = Path(__file__).with_name('indoor3_3.yaml')
TOPIC = '/ov9281_apriltag_vision/survey'
FRAME = 'usb_camera_optical_frame'
YAW = math.radians(-89.5)


def settings():
    cfg = yaml.safe_load(CONFIG.read_text())
    if not isinstance(cfg['expected_text'], str) or not cfg['expected_text'] or type(cfg['text_confirmed']) is not bool:
        raise ValueError('检查 indoor3_3.yaml 的 expected_text 和 text_confirmed')
    tf_path = Path(rospkg.RosPack().get_path('front_marker_control')) / 'config/front_camera_tf.yaml'
    tf = yaml.safe_load(tf_path.read_text())
    if (tf['base_frame'], tf['camera_frame']) != ('drone_base', FRAME):
        raise ValueError('OV9281 mounting frame mismatch')
    cfg['camera_rotation_xyzw'] = tf['rotation_quaternion']
    finite(cfg['camera_translation'])
    rotation(cfg['camera_rotation_xyzw'])
    return cfg


def write_json(path, data):
    tmp = path.with_suffix('.tmp')
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    tmp.replace(path)


def write_summary(out):
    records = [json.loads(p.read_text()) for p in sorted(out.glob('round_*/round.json'))]
    fields = ['round','status','expected_text','text_confirmed','goal_yaw_map_deg','arrival_yaw_map_deg']
    score = {c:0 for c in PHOTO_STREAMS}
    for camera in score:
        fields += [camera+'_'+s for s in ('status','attempts','first_match_ms','texts','photo')]
    total = 0
    tmp = out/'rounds.tmp'
    with tmp.open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for r in records:
            row = {f:r.get(f) for f in fields[:6]}
            row.update(expected_text=r['config']['expected_text'],text_confirmed=r['config']['text_confirmed'])
            cameras = r.get('scan', {}).get('cameras', {})
            complete = r['status'] == 'complete'
            total += int(complete)
            for cam, s in cameras.items():
                score[cam] += int(complete and s['status'] == 'success')
                for k in ('status','attempts','first_match_ms'):
                    row[cam+'_'+k] = s.get(k)
                row[cam+'_texts'] = json.dumps(sorted({v for a in s['reads'] for v in a['texts']}), ensure_ascii=False)
                if s.get('photo'):
                    row[cam+'_photo'] = 'round_%03d/%s' % (r['round'], s['photo'])
            writer.writerow(row)
    tmp.replace(out/'rounds.csv')
    write_json(out/'success_rates.json', dict(completed=total, cameras={
        c:dict(successes=n, rate=n/total if total else None) for c,n in score.items()}))


class Runner(SquareRunner):
    test_name = record_prefix = 'test_indoor3_3'
    route_name = '室内 test3_3 Tag 1 双相机扫码'
    goals_in_map = True
    side_m = None
    hold_seconds = 2.
    repeat_laps = True

    def __init__(self, *args):
        super().__init__(*args)
        self.config = settings()
        self.out = Path(self.a.out)
        self.route = load_route()
        self.translation = finite(self.config['camera_translation'])
        self.camera_rotation = rotation(self.config['camera_rotation_xyzw'])
        self.lever = finite(self.cfg['lever_body'])
        self.lock = threading.Lock()
        self.frames = deque(maxlen=160)
        self.poses = LocalPoseBuffer(250)
        self.tag_sub = rospy.Subscriber(TOPIC, TagSurveyFrame, self.on_tags, queue_size=32)
        self.odom_sub = rospy.Subscriber('/drone_nav/odom', Odometry, self.on_odom, queue_size=32)

    def on_tags(self, msg):
        if valid_frame(msg, FRAME):
            with self.lock:
                self.frames.append(msg)

    def on_odom(self, msg):
        with self.lock:
            if (msg.header.frame_id, msg.child_frame_id) != ('drone_odom','drone_nav_base'):
                self.poses.clear()
                return
            values = pose_values(msg.pose.pose)
            self.poses.accept(msg.header.stamp.to_sec(), msg.header.frame_id, values[:3], values[3:])

    def guard(self, **kwargs):
        try:
            super().guard(**kwargs)
            if self.reference is not None:
                r,t = self.reference
                if (r @ self.s.pose()[0] + t)[2] > self.route['max_control_z']:
                    raise Stopped('控制点超过室内高度上限')
        except Stopped as error:
            self.trial['reason'] = str(error)
            if self.real and (not self.s.offboard() or '切出 OFFBOARD' in str(error)):
                self.trial['status'] = 'interrupted'
            raise

    def estimate(self, since):
        # Retry recent frames as odometry arrives: lookup requires bracketing samples.
        samples = {}
        now = rospy.Time.now()
        with self.lock:
            for msg in self.frames:
                stamp = msg.header.stamp
                if stamp < since or not 0 <= (now-stamp).to_sec() <= .5:
                    continue
                tags = [tag for tag in msg.tags if tag.id == 1]
                if len(tags) != 1 or not 0 <= tags[0].reprojection_rms_px <= 2.:
                    continue
                nav = self.poses.lookup(stamp.to_sec(), .20)
                if nav is None:
                    continue
                try:
                    p, axes, _ = map_tag(nav, tags[0].pose, self.reference, self.translation,
                                         self.camera_rotation, self.lever)
                    if np.isfinite(p).all() and np.isfinite(axes).all():
                        samples[stamp.to_nsec()] = (p,axes)
                except ValueError:
                    continue
        try:
            p, axes = consolidate(list(samples.values()))
            return p, axes, sorted(samples)
        except ValueError:
            return None

    def make_route_map(self):
        if (not self.cfg.get('allow_short_goals') or self.cfg.get('allow_unknown_above') is not False
                or (self.real and Path(self.cfg['map_id']).name != self.route['map_id'])):
            raise Stopped('test3_3 室内地图或导航配置不匹配')
        since = rospy.Time.now()
        entered = since
        deadline = time.monotonic() + 5.
        while time.monotonic() < deadline:
            self.guard()
            if self.s.pose()[2] > .05:
                since = rospy.Time.now()
            elif (rospy.Time.now()-since).to_sec() >= .5:
                observed = self.estimate(since)
                if observed:
                    break
            time.sleep(.025)
        else:
            with self.lock:
                latest = self.frames[-1] if self.frames else None
            live = latest is not None and latest.header.stamp >= entered and valid_frame(latest, FRAME)
            self.trial.update(status='unlocked' if live else 'stream_fault',
                              reason='5 秒内没有稳定锁定 Tag 1' if live else 'OV9281 Tag 数据流中断')
            raise Stopped(self.trial['reason'])
        tag, axes, stamps = observed
        goal, camera = fixed_scan_goal(tag, axes, self.reference, self.translation-self.lever, YAW)
        if not np.isfinite(goal).all() or goal[2] > self.route['max_control_z']:
            raise Stopped('扫码目标无效或超过室内高度上限')
        self.trial.update(tag_center_map=tag.tolist(), normal_map=axes[:,2].tolist(), tag_stamps_ns=stamps,
                          camera_goal_map=camera.tolist(), control_goal_map=goal.tolist(),
                          goal_yaw_map_deg=-89.5, standoff_m=.8)
        write_json(self.folder/'round.json', self.trial)
        return np.array([goal]), np.array([YAW]), ('Tag 1 前方 0.8 m',)

    def freeze(self):
        try:
            return super().freeze()
        except Stopped as error:
            self.trial.setdefault('reason', str(error))
            raise

    def send_leg(self, i, goal, goal_map, yaw_map):
        self.check_segments = np.linalg.norm(goal-self.s.pose()[0]) < .20
        super().send_leg(i, goal, goal_map, yaw_map)

    def hold_at_point(self, i, leg, seconds):
        self.guard(stopped=True)
        self.trial['arrival_yaw_map_deg'] = math.degrees(heading(self.reference[0], self.s.pose()[1]))
        self.trial['scan'] = {}
        paired_scan(self.folder, self.config['expected_text'], lambda: self.guard(stopped=True), self.trial['scan'])
        cameras = self.trial['scan']['cameras']
        self.guard(stopped=True)
        faults = [s['status'] for s in cameras.values() if s['status'] not in ('success','timeout')]
        self.trial['status'] = faults[0] if faults else 'complete'
        message = '，'.join(c.upper()+' '+s['status'] for c,s in cameras.items())
        say('Tag 1 QR：'+message)
        if faults:
            raise Stopped('双相机比较无效：'+message)

    def run_lap(self):
        number = max((int(p.name[6:]) for p in self.out.glob('round_*') if p.name[6:].isdigit()), default=0)+1
        self.folder = self.out/('round_%03d' % number)
        self.folder.mkdir()
        self.trial = dict(round=number,status='running',config=self.config,tag_id=1,
                          goal_yaw_map_deg=-89.5, start_ns=rospy.Time.now().to_nsec())
        write_json(self.folder/'round.json', self.trial)
        try:
            return super().run_lap()
        except KeyboardInterrupt:
            self.trial['status'] = 'interrupted'
            raise
        finally:
            if self.trial['status'] == 'running':
                self.trial['status'] = 'navigation_failed'
            if self.record:
                self.trial.update(nav_file=Path(self.record_path).name, nav_end=self.record['end'])
                if self.record.get('reason'):
                    self.trial['reason'] = self.record['reason']
            self.trial['end_ns'] = rospy.Time.now().to_nsec()
            write_json(self.folder/'round.json', self.trial)
            write_summary(self.out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',required=True,type=Path)
    parser.add_argument('--preview',action='store_true')
    parser.add_argument('--probe-camera',action='store_true')
    args = parser.parse_args()
    if args.probe_camera:
        rospy.init_node('indoor3_3_probe',anonymous=True)
        cfg = settings()
        probe_photo(args.out, PHOTO_STREAMS)
        until = time.monotonic()+6.
        while time.monotonic()<until:
            m = rospy.wait_for_message(TOPIC, TagSurveyFrame, timeout=max(.01,until-time.monotonic()))
            if valid_frame(m, FRAME):
                say('OV9281 Tag 流就绪，目标 QR '+cfg['expected_text'])
                return 0
        raise Stopped('OV9281 Tag 流未就绪')
    return serve('nav_indoor3_3', SimpleNamespace(out=str(args.out), preview=args.preview, delay=3.), Runner)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (Stopped,rospy.ROSException,ValueError) as error:
        say(str(error))
        raise SystemExit(1)
