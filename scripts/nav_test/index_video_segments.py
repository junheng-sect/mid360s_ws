#!/usr/bin/env python3
"""Index finalized test3-4 segments for import-flight-experiment after ground cleanup."""
import argparse
import csv
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flight_archive.pairing import bag_window, describe, run_identity, write_json


def index(directory, root):
    profile = directory / 'video_cameras.txt'
    cameras = profile.read_text().split() if profile.exists() else ['ov13855', 'ov9281']
    if cameras not in (['ov13855'], ['ov13855', 'ov9281']):
        raise ValueError('无法识别录像相机清单：' + str(cameras))
    for bag in sorted(directory.glob('test_indoor3_4_*.bag')):
        stem = bag.stem
        expected = [directory / (stem + '_' + camera + '.mkv') for camera in cameras]
        expected.extend(directory / (stem + suffix) for suffix in ('_frames.csv', '_events.csv'))
        events = expected[-1]
        terminal = None
        timing_gaps = 0
        if events.is_file():
            with events.open(newline='') as stream:
                for row in csv.DictReader(stream):
                    terminal = row['event']
                    timing_gaps += terminal == 'timing_gap'
        missing = [path.name for path in expected if not path.is_file()]
        complete = not missing and terminal == 'finalized'
        window = bag_window(bag, native=True)
        run = run_identity(bag, window)
        files = [describe(path) for path in [bag] + expected if path.is_file()]
        if profile.is_file():
            files.append(describe(profile))
        # Session logs may be shared by several bags. Index them after the task has stopped.
        for pattern in ('*.log', 'drone_reloc_*.json', 'dll_observer.json', 'dll_*.yaml'):
            files.extend(describe(path) for path in sorted(directory.glob(pattern)) if path.is_file())
        dest = root / run / 'manifest.json'
        dest.parent.mkdir(parents=True, exist_ok=True)
        record = dict(run_id=run, status='missing_ulog', window=window, pairing={}, files=files,
                      video=dict(enabled_cameras=cameras, complete=complete, final_event=terminal, missing=missing, timing_gaps=timing_gaps))
        if dest.exists():
            previous = json.loads(dest.read_text())
            old_bag = next(item for item in previous['files'] if item['name'] == bag.name)
            if old_bag != files[0]:
                raise ValueError('实验 ID 已被不同的 bag 占用：' + run)
        write_json(dest, record)
        print(run, '视频完整' if complete else '视频有缺帧或文件未完整封存', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    parser.add_argument('--root', type=Path, default=Path.home() / 'test_data/video_exports/runs')
    args = parser.parse_args()
    index(args.directory.resolve(), args.root)
