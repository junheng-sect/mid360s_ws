"""A paired, source-stamped QR test with independent per-camera early exit."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from contextlib import ExitStack
import threading
import time

import rospy
import orange_qr
from indoor3_runner import Frame, PHOTO_STREAMS, save_photo


class ScanStream(Frame):
    def __init__(self, camera):
        self.stamps = deque(maxlen=256)
        super().__init__(camera)

    def receive(self, msg):
        super().receive(msg)
        if self.recent and self.recent[-1] is msg:
            self.stamps.append(msg.header.stamp.to_nsec())


def scan_one(stream, target, start_ns, wall_start, cancel):
    deadline = wall_start + 2.
    end_ns = start_ns + 2_000_000_000
    seen = []
    last = None
    status = 'timeout'
    hit_ms = None
    reason = ''
    try:
        while not cancel.is_set() and time.monotonic() < deadline:
            msg = stream.recent[-1] if stream.recent else None
            stamp = msg.header.stamp.to_nsec() if msg else 0
            if (not start_ns <= stamp < end_ns or (last and msg.header.stamp <= last.header.stamp)
                    or not -.02 <= (rospy.Time.now() - msg.header.stamp).to_sec() <= .25):
                cancel.wait(.005)
                continue
            last = msg
            texts = orange_qr.decode(msg.data, msg.width, msg.height, msg.step)
            ms = (time.monotonic() - wall_start) * 1000
            seen.append(dict(stamp_ns=stamp, elapsed_ms=ms, texts=texts))
            if target in texts and ms <= 2000 and not cancel.is_set():
                status, hit_ms = 'success', ms
                break
    except Exception as error:
        status, reason = 'decoder_fault', str(error)
    if cancel.is_set():
        status = 'interrupted'
    stop_ns = min(end_ns, start_ns + int((time.monotonic() - wall_start) * 1e9))
    stamps = sorted({s for s in stream.stamps.copy() if start_ns <= s <= stop_ns})
    gaps = [b-a for a,b in zip([start_ns]+stamps, stamps+[stop_ns])]
    if status in ('timeout','success') and (not stamps or max(gaps) > 500_000_000):
        status = 'stream_fault'
    return dict(status=status, reason=reason, frames=len(stamps), max_gap_ms=max(gaps)/1e6,
                attempts=len(seen), first_match_ms=hit_ms, reads=seen), last


def paired_scan(folder, target, guard, record):
    cancel = threading.Event()
    with ExitStack() as resources:
        streams = {}
        for camera in PHOTO_STREAMS:
            streams[camera] = ScanStream(camera)
            resources.callback(streams[camera].close)
        # Negotiate both TCPROS image connections before starting either clock.
        until = time.monotonic() + 1.5
        while time.monotonic() < until and not all(s.recent for s in streams.values()):
            guard()
            time.sleep(.02)
        guard()
        with ThreadPoolExecutor(max_workers=2) as pool:
            start = rospy.Time.now().to_nsec()
            wall = time.monotonic()
            record.update(start_ns=start, end_ns=start+2_000_000_000, cameras={})
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
