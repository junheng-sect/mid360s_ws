#!/usr/bin/env python3
"""Small, clearly labelled synthetic scene, restricted to a local test master."""
import argparse
import copy
import json
import math
import os
import struct


def main():
    if os.environ.get('ROS_MASTER_URI') != 'http://127.0.0.1:21111':
        raise SystemExit('Offline fixture requires ROS_MASTER_URI=http://127.0.0.1:21111')
    parser = argparse.ArgumentParser()
    parser.add_argument('--localized', action='store_true')
    parser.add_argument('--repeat-cloud', action='store_true', help='5 Hz repeated-cloud history stress test')
    args = parser.parse_args()
    import cv2
    import numpy as np
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    rospy.init_node('offline_display_fixture')
    static = tf2_ros.StaticTransformBroadcaster()
    dynamic = tf2_ros.TransformBroadcaster()
    def transform(parent, child, x=0.):
        message = TransformStamped()
        message.header.frame_id = parent
        message.child_frame_id = child
        message.transform.translation.x = x
        message.transform.rotation.w = 1.
        return message
    edges = [transform('drone_odom', 'camera_init'), transform('map', 'drone_map', 1.)]
    if args.localized:
        edges.append(transform('drone_map', 'drone_odom', .5))
    static.sendTransform(edges)
    cloud = PointCloud2()
    cloud.header.frame_id = 'camera_init'
    cloud.height = 1
    cloud.fields = [PointField(name=n, offset=4*i, datatype=PointField.FLOAT32, count=1)
                    for i, n in enumerate(('x', 'y', 'z', 'intensity', 'rgb'))]
    points = []
    rgb = struct.unpack('f', struct.pack('I', 0x20A0F0))[0]
    for x in range(-40, 41):
        for y in range(-40, 41):
            z = .3 * math.sin(x * .15) * math.cos(y * .15)
            points.append(struct.pack('<fffff', x * .1, y * .1, z, z + .3, rgb))
    cloud.width = len(points)
    cloud.point_step = 20
    cloud.row_step = cloud.width * cloud.point_step
    cloud.is_dense = True
    cloud.data = b''.join(points)
    mapping = copy.deepcopy(cloud)
    mapping.header.frame_id = 'map'
    image = np.zeros((360, 640, 3), np.uint8)
    cv2.putText(image, 'OFFLINE DISPLAY TEST', (30, 180), cv2.FONT_HERSHEY_SIMPLEX, 1., (0, 220, 255), 2)
    preview = CompressedImage()
    preview.header.frame_id = 'camera_init'
    preview.format = 'png'
    preview.data = cv2.imencode('.png', image)[1].tobytes()
    pubs = {
        'cloud': rospy.Publisher('/cloud_registered', PointCloud2, queue_size=1, latch=True),
        'map': rospy.Publisher('/drone_reloc/map', PointCloud2, queue_size=1, latch=True),
        'odom': rospy.Publisher('/Odometry', Odometry, queue_size=1),
        'nav': rospy.Publisher('/drone_nav/odom', Odometry, queue_size=1),
        'status': rospy.Publisher('/drone_reloc/status', String, queue_size=1),
        'image': rospy.Publisher('/ov13855_apriltag_vision/image_preview/compressed', CompressedImage, queue_size=1),
        'label': rospy.Publisher('/drone_0_ego_planner_node/optimal_list', Marker, queue_size=1),
    }
    label = Marker()
    label.header.frame_id = 'drone_odom'
    label.ns = 'offline_test'
    label.id = 0
    label.type = Marker.TEXT_VIEW_FACING
    label.pose.orientation.w = 1.
    label.pose.position.z = 2.
    label.scale.z = .3
    label.color.r = label.color.a = 1.
    label.text = 'OFFLINE DISPLAY TEST'
    pubs['cloud'].publish(cloud)
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        stamp = rospy.Time.now()
        cloud.header.stamp = preview.header.stamp = label.header.stamp = stamp
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'drone_odom'
        odom.child_frame_id = 'drone_base'
        odom.pose.pose.position.z = 1.
        odom.pose.pose.orientation.w = 1.
        body = transform('drone_odom', 'drone_base')
        body.header.stamp = stamp
        body.transform.translation.z = 1.
        dynamic.sendTransform(body)
        if args.repeat_cloud:
            pubs['cloud'].publish(cloud)
        pubs['map'].publish(mapping)
        pubs['odom'].publish(odom)
        pubs['nav'].publish(odom)
        pubs['image'].publish(preview)
        pubs['label'].publish(label)
        pubs['status'].publish(String(data=json.dumps({'valid': args.localized,
            'state': 'LOCALIZED' if args.localized else 'FAILED', 'test': 'OFFLINE DISPLAY TEST'})))
        rate.sleep()


if __name__ == '__main__':
    main()
