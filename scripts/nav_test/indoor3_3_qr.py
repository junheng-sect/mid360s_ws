"""Two source-stamped, pose-guided QR readers sharing one arrival window."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import orange_qr
import rospkg
import rospy
from sensor_msgs.msg import CameraInfo
from ov13855_apriltag_vision.msg import TagSurveyFrame
from drone_navigation.core import rotation
from indoor3_runner import Frame, PHOTO_STREAMS, save_photo
from nav_indoor3_1 import pose_values, valid_frame


WINDOW_NS = 2_000_000_000
MAX_GAP_NS = 500_000_000
CONFIG_DIR = Path(rospkg.RosPack().get_path('barcode_reader_camera')) / 'config'


class ScanStream(Frame):
    def __init__(self, camera, config_dir):
        self.stamps = deque(maxlen=256)
        self.surveys = deque(maxlen=64)
        self.calibration = None
        self.config_dir = config_dir
        self.calibration_error = ''
        super().__init__(camera, history=5 if camera == 'ov13855' else 12)
        image_topic = PHOTO_STREAMS[camera][0]
        self.info_sub = rospy.Subscriber(image_topic.rsplit('/', 1)[0] + '/camera_info',
                                         CameraInfo, self.on_info, queue_size=1)
        self.survey_sub = rospy.Subscriber('/%s_apriltag_vision/survey' % camera, TagSurveyFrame,
                                           self.on_survey, queue_size=16)

    def receive(self, msg):
        super().receive(msg)
        if self.recent and self.recent[-1] is msg:
            self.stamps.append(msg.header.stamp.to_nsec())

    def on_survey(self, msg):
        if valid_frame(msg, self.frame_id):
            self.surveys.append(msg)

    def on_info(self, msg):
        try:
            if ((msg.width, msg.height, msg.header.frame_id) != (self.width, self.height, self.frame_id)
                    or msg.distortion_model not in ('plumb_bob', 'rational_polynomial')):
                raise ValueError('相机标定坐标系、分辨率或畸变模型不匹配')
            key = tuple(msg.K), tuple(msg.D)
            if self.calibration and self.calibration[0] == key:
                return
            reader = orange_qr.label_reader(str(self.config_dir / 'barcode_reader_camera.yaml'),
                                            str(self.config_dir / 'layouts/qr_top_tag_bottom.yaml'), *key)
            self.calibration = key, reader
            self.calibration_error = ''
        except (ValueError, RuntimeError) as error:
            self.calibration = None
            self.calibration_error = str(error)

    def ready(self):
        return bool(self.recent and self.surveys and self.calibration)

    def close(self):
        self.survey_sub.unregister()
        self.info_sub.unregister()
        super().close()

    def next_pair(self, after, start, end):
        # Self/main paths on OV13855 have independent timestamps for the same exposure.
        tolerance = 10_000_000 if self.camera == 'ov13855' else 0
        surveys = [(s.header.stamp.to_nsec(), s) for s in self.surveys.copy()
                   if start <= s.header.stamp.to_nsec() < end]
        if not surveys:
            return None
        now = rospy.Time.now().to_nsec()
        for image in reversed(self.recent.copy()):
            stamp = image.header.stamp.to_nsec()
            if not (after < stamp < end and start <= stamp and -20_000_000 <= now-stamp <= 250_000_000):
                continue
            ts, survey = min(surveys, key=lambda entry: abs(entry[0]-stamp))
            tags = [t for t in survey.tags if t.id == 1]
            if abs(ts-stamp) > tolerance or 1 not in survey.ids or len(tags) != 1:
                continue
            pose = np.asarray(pose_values(tags[0].pose), dtype=float)
            if np.isfinite(pose).all() and pose[2] > 0 and abs(float(pose[3:] @ pose[3:]) - 1.) <= .002:
                return image, ts, pose
        return None


def coverage(stamps, start, stop):
    stamps = sorted({stamp for stamp in stamps if start <= stamp <= stop})
    edges = [start] + stamps + [stop]
    return len(stamps), max(b-a for a, b in zip(edges, edges[1:]))


def scan_one(stream, target, start_ns, wall_start, cancel):
    end_ns = start_ns + WINDOW_NS
    deadline = wall_start + WINDOW_NS / 1e9
    reads = []
    image = None
    last_stamp = start_ns - 1
    hit_ms = None
    strategy = None
    status = 'timeout'
    reason = ''
    try:
        while not cancel.is_set() and time.monotonic() < deadline:
            calibration = stream.calibration
            pair = stream.next_pair(last_stamp, start_ns, end_ns) if calibration else None
            if pair is None:
                cancel.wait(.005)
                continue
            image, pose_stamp, pose = pair
            last_stamp = image.header.stamp.to_nsec()
            rvec = cv2.Rodrigues(rotation(pose[3:]))[0].reshape(3)
            codes = orange_qr.decode_label(calibration[1], image.data, image.width, image.height,
                                           image.step, tuple(rvec), tuple(pose[:3]))
            ms = (time.monotonic() - wall_start) * 1000
            reads.append(dict(stamp_ns=last_stamp, pose_stamp_ns=pose_stamp, elapsed_ms=ms,
                              range_m=float(np.linalg.norm(pose[:3])), depth_m=float(pose[2]),
                              texts=[text for text, _ in codes], strategies=[src for _, src in codes]))
            for text, src in codes:
                if text == target and ms <= WINDOW_NS/1e6 and not cancel.is_set():
                    strategy, hit_ms, status = src, ms, 'success'
                    break
            if status == 'success':
                break
    except Exception as error:
        status, reason = 'decoder_fault', str(error)
    if cancel.is_set():
        status = 'interrupted'
    stop_ns = min(end_ns, start_ns + int((time.monotonic() - wall_start) * 1e9))
    frames, gap = coverage(stream.stamps.copy(), start_ns, stop_ns)
    surveys = [s for s in stream.surveys.copy() if start_ns <= s.header.stamp.to_nsec() <= stop_ns]
    survey_frames, survey_gap = coverage((s.header.stamp.to_nsec() for s in surveys), start_ns, stop_ns)
    tag_frames = sum(1 in s.ids for s in surveys)
    if status in ('timeout', 'success'):
        if not frames or gap > MAX_GAP_NS:
            status, reason = 'stream_fault', 'image_stream'
        elif not survey_frames or survey_gap > MAX_GAP_NS:
            status, reason = 'stream_fault', 'tag_survey_stream'
        elif not stream.calibration:
            status, reason = 'stream_fault', stream.calibration_error or 'no_camera_info'
        elif status == 'timeout':
            if reads:
                reason = 'roi_no_target'
            else:
                reason = 'no_matched_pose' if tag_frames else 'no_tag'
    if image is None:
        image = next((msg for msg in reversed(stream.recent.copy())
                      if start_ns <= msg.header.stamp.to_nsec() <= stop_ns), None)
    last = reads[-1] if reads else {}
    result = dict(status=status, reason=reason, frames=frames, survey_frames=survey_frames,
                  tag_frames=tag_frames, max_gap_ms=gap/1e6, max_survey_gap_ms=survey_gap/1e6,
                  attempts=len(reads), first_match_ms=hit_ms, strategy=strategy,
                  range_m=last.get('range_m'), depth_m=last.get('depth_m'), reads=reads)
    return result, image


def open_streams(resources):
    streams = {}
    for camera in PHOTO_STREAMS:
        streams[camera] = ScanStream(camera, CONFIG_DIR)
        resources.callback(streams[camera].close)
    return streams


def paired_scan(folder, target, guard, record):
    # Two camera workers already run concurrently on the flight computer.
    cv2.setNumThreads(1)
    cancel = threading.Event()
    with ExitStack() as resources:
        streams = open_streams(resources)
        until = time.monotonic() + 1.5
        while time.monotonic() < until and not all(s.ready() for s in streams.values()):
            guard()
            time.sleep(.02)
        guard()
        with ThreadPoolExecutor(max_workers=2) as pool:
            start = rospy.Time.now().to_nsec()
            wall = time.monotonic()
            record.update(start_ns=start, end_ns=start+WINDOW_NS, cameras={})
            futures = {c: pool.submit(scan_one, s, target, start, wall, cancel) for c,s in streams.items()}
            try:
                while not all(f.done() for f in futures.values()):
                    guard()
                    time.sleep(.015)
                guard()
            finally:
                cancel.set()
                for camera, future in futures.items():
                    result, image = future.result()
                    record['cameras'][camera] = result
                    if image is not None:
                        path = folder / (camera + '.png')
                        save_photo(image, path, lambda: None)
                        result.update(photo=path.name, photo_stamp_ns=image.header.stamp.to_nsec())
