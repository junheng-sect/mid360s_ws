#!/usr/bin/env python3
"""Check optional cloud subscriptions and standard types without opening a bag."""
import builtins
import unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import Mock, patch
import yaml
import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import PointCloud2
from mavros_msgs.msg import ExtendedState, EstimatorStatus
from bag_recorder_node import RcBagRecorder, topic_type_registry


class TopicTests(unittest.TestCase):
    def test_landing_feedback_topics_are_registered_and_in_defaults(self):
        with Path(__file__).with_name('rc_record.yaml').open() as stream:
            topics=yaml.safe_load(stream)['rc_bag_recorder']['topics']
        registry=topic_type_registry()
        self.assertIn('/mavros/extended_state',topics)
        self.assertIn('/mavros/estimator_status',topics)
        self.assertIs(registry['/mavros/extended_state'],ExtendedState)
        self.assertIs(registry['/mavros/estimator_status'],EstimatorStatus)

    def recorder(self, cloud=False, topics=None):
        params = {'~record_registered_cloud': cloud,
                  '~topics': topics if topics is not None else ['/fastlio/velocity']}
        with ExitStack() as stack:
            stack.enter_context(patch.object(rospy, 'get_param', side_effect=lambda k, d=None: params.get(k, d)))
            stack.enter_context(patch.object(rospy, 'get_published_topics', return_value=[]))
            subscriber = stack.enter_context(patch.object(rospy, 'Subscriber'))
            for name in ('Publisher', 'Timer', 'on_shutdown', 'logwarn', 'loginfo'):
                stack.enter_context(patch.object(rospy, name))
            node = RcBagRecorder()
        return node, [c.args[0] for c in subscriber.call_args_list]

    def test_cloud_default_off_does_not_subscribe(self):
        _, subscribed = self.recorder()
        self.assertIn('/fastlio/velocity', subscribed)
        self.assertNotIn('/cloud_registered', subscribed)
        with Path(__file__).with_name('rc_record.yaml').open() as stream:
            c = yaml.safe_load(stream)['rc_bag_recorder']
        self.assertFalse(c['record_registered_cloud'])
        self.assertNotIn('/cloud_registered', c['topics'])

    def test_cloud_enabled_adds_one_subscription(self):
        _, subscribed = self.recorder(cloud=True)
        self.assertEqual(subscribed.count('/cloud_registered'), 1)
        _, subscribed = self.recorder(cloud=True, topics=['/cloud_registered'])
        self.assertEqual(subscribed.count('/cloud_registered'), 1)

    def test_explicit_custom_topic_list_can_request_cloud(self):
        _, subscribed = self.recorder(topics=['/cloud_registered'])
        self.assertIn('/cloud_registered', subscribed)

    def test_standard_types_survive_missing_navigation_messages(self):
        original = builtins.__import__
        def no_navigation(name, *args, **kwargs):
            if name in ('traj_utils.msg', 'drone_navigation.msg'):
                raise ImportError('navigation intentionally unavailable')
            return original(name, *args, **kwargs)
        with patch('builtins.__import__', side_effect=no_navigation), patch.object(rospy, 'logwarn'):
            registry = topic_type_registry()
        self.assertIs(registry['/fastlio/velocity'], Vector3Stamped)
        self.assertIs(registry['/cloud_registered'], PointCloud2)
        self.assertNotIn('/drone_nav/bspline', registry)


if __name__ == '__main__':
    unittest.main()
