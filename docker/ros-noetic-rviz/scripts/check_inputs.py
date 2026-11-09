#!/usr/bin/env python3
"""Receive actual display inputs; graph registration alone is not acceptance."""
import argparse
import collections
import io
import socket
import time
import uuid
import xmlrpc.client


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=('fastlio', 'reloc', 'ego'), default='fastlio')
    parser.add_argument('--odom', default='/drone_nav/odom')
    parser.add_argument('--fixed-frame', default='drone_odom')
    parser.add_argument('--timeout', type=float, default=6.)
    args = parser.parse_args()
    if args.timeout <= 0:
        parser.error('--timeout must be positive')
    import rospy
    import tf2_ros
    from sensor_msgs.msg import PointCloud2, CompressedImage
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage
    import yaml
    required = {'/cloud_registered'}
    optional = {'/tf', '/tf_static', '/ov13855_apriltag_vision/image_preview/compressed'}
    if args.mode == 'fastlio':
        required.add('/Odometry')
        optional.add('/path')
    elif args.mode == 'reloc':
        required.update(('/drone_reloc/map', '/drone_reloc/status'))
        optional.add('/car/status')
    else:
        required.add(args.odom)
        optional.update(('/drone_0_ego_planner_node/optimal_list',
                         '/drone_0_ego_planner_node/grid_map/occupancy',
                         '/drone_0_ego_planner_node/grid_map/occupancy_inflate'))
    # Container PIDs and millisecond start times can coincide. rospy's anonymous
    # suffix alone then collides across simultaneous checks on the same Master.
    rospy.init_node('rviz_display_check_' + uuid.uuid4().hex, disable_signals=True)
    counts = collections.Counter()
    sizes = collections.Counter()
    frames = {}
    latest_age = {}
    def receive(message, topic):
        counts[topic] += 1
        sizes[topic] += len(message._buff)
        cls = None
        if topic in ('/cloud_registered', '/drone_reloc/map'):
            cls = PointCloud2
        elif topic in ('/Odometry', args.odom):
            cls = Odometry
        elif topic.endswith('/compressed'):
            cls = CompressedImage
        if cls is not None:
            decoded = cls().deserialize(message._buff)
            frames[topic] = decoded.header.frame_id
            if decoded.header.stamp.to_sec() > 0:
                latest_age[topic] = (rospy.Time.now() - decoded.header.stamp).to_sec()
    def receive_tf(message, topic):
        counts[topic] += 1
        serialized = io.BytesIO()
        message.serialize(serialized)
        sizes[topic] += serialized.tell()
    # rospy shares one transport/type per topic within a process. TF statistics must
    # use TFMessage, matching TransformListener; AnyMsg would break its callbacks.
    subscribers = [rospy.Subscriber(topic, TFMessage if topic in ('/tf', '/tf_static') else rospy.AnyMsg,
                                   receive_tf if topic in ('/tf', '/tf_static') else receive,
                                   callback_args=topic, queue_size=1)
                   for topic in sorted(required | optional)]
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)
    node_uri = rospy.get_node_uri()
    print('NODE_URI=' + node_uri, flush=True)
    started = time.monotonic()
    while not rospy.is_shutdown() and time.monotonic() - started < args.timeout:
        time.sleep(.1)
    failed = False
    elapsed = time.monotonic() - started
    for topic in sorted(required | optional):
        print('%-12s %s messages=%d bytes=%d hz=%.1f%s%s' %
              ('RECEIVED' if counts[topic] else 'WAITING', topic, counts[topic], sizes[topic],
               counts[topic] / elapsed, ' required' if topic in required else ' optional',
               ' stamp_age_s=%.3f' % latest_age[topic] if topic in latest_age else ''))
        failed |= topic in required and not counts[topic]
    tree = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
    known_frames = set(tree) | {entry.get('parent') for entry in tree.values() if isinstance(entry, dict)}
    frame_ok = args.fixed_frame in known_frames
    print('FIXED_FRAME %s %s' % (args.fixed_frame, 'OK' if frame_ok else 'MISSING'))
    failed |= not frame_ok
    for topic in sorted(required):
        if topic in frames:
            source = frames[topic]
            transform_ok = bool(source) and buffer.can_transform(args.fixed_frame, source, rospy.Time(0))
            print('TRANSFORM %s <- %s %s (%s)' %
                  (args.fixed_frame, source or '<empty>', 'OK' if transform_ok else 'MISSING', topic))
            failed |= not transform_ok
    try:
        socket.setdefaulttimeout(3)
        code, _, _ = xmlrpc.client.ServerProxy(node_uri).getPid('/rviz_display_check')
        if code != 1:
            raise RuntimeError('getPid returned %s' % code)
        print('ADVERTISED_XMLRPC_OK (external board callback still requires live verification)')
    except Exception as exc:
        print('ADVERTISED_XMLRPC_FAILED ' + str(exc))
        failed = True
    rospy.signal_shutdown('check complete')
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
