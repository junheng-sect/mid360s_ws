#!/usr/bin/env python3
"""Exercise real CH6 bag boundaries and lossless videos on a private Noetic master."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess as sp
import tempfile
import time

import numpy as np


def run(folder, dual):
    scripts = Path(__file__).resolve().parents[2]
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        port = sock.getsockname()[1]
    os.environ.update(ROS_MASTER_URI='http://127.0.0.1:' + str(port), ROS_IP='127.0.0.1',
                      ROS_HOSTNAME='127.0.0.1', RC_BAG_PREFIX='test_indoor3_4', RC_BAG_DIR=str(folder))
    import rosgraph
    import rospy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import RCIn
    from sensor_msgs.msg import Image
    from std_msgs.msg import String

    processes = []

    def start(name, *args):
        with (folder / (name + '.log')).open('w') as log:
            proc = sp.Popen(args, stdout=log, stderr=sp.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(12)
            except sp.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(5)

    heights = {'ov13855': 960}
    if dual:
        heights['ov9281'] = 800
    interrupted = 'ov9281' if dual else 'ov13855'
    try:
        start('master', 'roscore', '-p', str(port))
        for _ in range(80):
            if rosgraph.is_master_online():
                break
            time.sleep(.1)
        rospy.init_node('test34_fixture', disable_signals=True)
        video = start('video', 'rosrun', 'dual_camera_recorder', 'dual_camera_recorder_node',
                      '_directory:=' + str(folder), '_ov13855_only:=' + ('false' if dual else 'true'))
        bag = start('bag', str(scripts / 'rc_record/start_rc_record.sh'), '_rc_topic:=/fixture/ch6')
        rc = rospy.Publisher('/fixture/ch6', RCIn, queue_size=5)
        pose = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=4)
        events = []
        rospy.Subscriber('/rc_record/bag_path', String, lambda msg: events.append(msg.data))
        frames = {}
        row = np.arange(1280, dtype=np.uint8)
        for name, height in heights.items():
            pub = rospy.Publisher(('/ov13855' if name == 'ov13855' else '/usb_cam') + '/image_raw', Image, queue_size=5)
            frames[name] = (pub, {}, np.add.outer(np.arange(height, dtype=np.uint8), row))
        deadline = time.monotonic() + 20
        while not (rc.get_num_connections() and all(v[0].get_num_connections() for v in frames.values())):
            assert time.monotonic() < deadline, 'fixture subscribers unavailable'
            time.sleep(.1)
        tick = 0
        def level(pwm, duration, pause_stream=False):
            nonlocal tick
            start_time = time.monotonic()
            while time.monotonic() - start_time < duration:
                t = rospy.Time.now()
                switch = RCIn(); switch.channels = [1500] * 5 + [pwm]; rc.publish(switch)
                sample = PoseStamped(); sample.header.stamp = t; pose.publish(sample)
                for name, (pub, expected, base) in frames.items():
                    if (name == 'ov13855' and tick % 2) or (name == interrupted and pause_stream):
                        continue
                    arr = (base + np.uint8(tick % 253))
                    msg = Image()
                    msg.header.stamp = t
                    msg.header.frame_id = 'ov13855_optical_frame' if name == 'ov13855' else 'usb_camera_optical_frame'
                    msg.width, msg.height = 1280, heights[name]
                    msg.encoding, msg.step = 'mono8', 1284
                    msg.data = np.pad(arr, ((0, 0), (0, 4)), constant_values=231).tobytes()
                    expected[t.to_nsec()] = hashlib.sha256(arr).digest()
                    pub.publish(msg)
                tick += 1
                time.sleep(.028)
        level(1000, .6)
        level(1900, .8)
        level(1000, .4)
        assert len(events) == 1 and events[0].startswith('OPEN ')
        level(1900, .7)
        release_ns = rospy.Time.now().to_nsec()
        level(1000, 1.7)
        assert len(events) == 2 and events[1] == 'CLOSED ' + events[0][5:]
        level(1900, .6)
        level(1900, 1.4, pause_stream=True)
        level(1900, .6)
        assert len(events) == 3
        stop(bag)  # Task exit while CH6 is still high.
        level(1000, .4)
        stop(video)
        bags = sorted(folder.glob('*.bag'))
        assert len(bags) == 2 and len(set(events[::2])) == 2 and not list(folder.glob('*.partial'))
        import rosbag
        for n, path in enumerate(bags):
            with rosbag.Bag(str(path)) as recorded:
                assert recorded.get_message_count('/mavros/local_position/pose') > 0
            csv_rows = list(csv.DictReader(path.with_name(path.stem + '_frames.csv').open()))
            log = list(csv.DictReader(path.with_name(path.stem + '_events.csv').open()))
            if not dual:
                assert not list(folder.glob('*_ov9281.mkv'))
                assert all(r['camera'] == 'ov13855' for r in csv_rows + log)
            opened = int(log[0]['stamp_ns'])
            closed = next(int(e['stamp_ns']) for e in log if e['event'] == 'close')
            if n == 0:
                assert .97 < (closed-release_ns)/1e9 < 1.5
            else:
                assert any(e['event'] == 'stream_lost' and e['camera'] == interrupted for e in log)
                assert any(e['event'] == 'stream_recovered' and e['camera'] == interrupted for e in log)
            counts = {}
            for name, height in heights.items():
                table = [r for r in csv_rows if r['camera'] == name]
                fname = str(path.with_name(path.stem+'_'+name+'.mkv'))
                info = json.loads(sp.check_output(['ffprobe', '-v', 'error', '-show_streams', '-show_packets',
                                                   '-show_entries', 'stream=codec_name,pix_fmt,width,height,time_base:packet=pts', '-of', 'json', fname]))
                fmt = info['streams'][0]
                assert (fmt['codec_name'], fmt['pix_fmt'], fmt['width'], fmt['height'], fmt['time_base']) == ('ffvhuff','gray',1280,height,'1/1000')
                assert [int(p['pts']) for p in info['packets']] == [int(r['pts_ms']) for r in table]
                assert len(table) > 8
                with sp.Popen(['ffmpeg','-v','error','-i',fname,'-vsync','0','-enc_time_base','1:1000','-pix_fmt','gray','-f','rawvideo','-'], stdout=sp.PIPE) as dec:
                    for i, r in enumerate(table):
                        ns = int(r['source_stamp_ns'])
                        assert opened <= ns <= closed
                        assert int(r['frame_index']) == i and int(r['pts_ms']) == (ns-opened+500000)//1000000
                        assert hashlib.sha256(dec.stdout.read(1280*height)).digest() == frames[name][1][ns]
                    assert not dec.stdout.read(1) and dec.wait() == 0
                counts[name] = len(table)
            print(path.name, counts, 'pixels, PTS and frame indices verified', flush=True)
    finally:
        for proc in reversed(processes):
            stop(proc)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dual', action='store_true', help='Exercise the original two-camera recorder mode')
    args = parser.parse_args()
    folder = Path(tempfile.mkdtemp(prefix='test34-boundaries.'))
    print(folder, flush=True)
    run(folder, args.dual)
