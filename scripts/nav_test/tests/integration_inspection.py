#!/usr/bin/env python3
"""Run test11 preview with real EGO/executor/gateway and synthetic sensors."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
import rosgraph
import rospy
from drone_navigation.core import wrap
from drone_navigation.ros_common import xyz
from geometry_msgs.msg import Point
from traj_utils.msg import PathTrajectory
from std_srvs.srv import Trigger

sys.path.insert(0, '/home/orangepi/orange/src/drone_navigation/test')
from integration import Harness


class InspectionHarness(Harness):
    def __init__(self):
        # Dense 4.8 m radial returns provide observed free space out to the
        # configured 4.5 m ray limit. No occupancy/clearance service is mocked.
        az, el = np.meshgrid(np.linspace(-np.pi, np.pi, 144, endpoint=False),
                             np.linspace(-np.pi/2, np.pi/2, 37))
        self.rays = 4.8 * np.column_stack([
            (np.cos(el)*np.cos(az)).ravel(), (np.cos(el)*np.sin(az)).ravel(), np.sin(el).ravel()])
        self.commands = []
        self.trajectories = []
        super().__init__()
        self.track = True
        rospy.Subscriber('/drone_nav/path_trajectory', PathTrajectory, self.on_trajectory)

    @property
    def points(self):
        return self.rays + self.p

    @points.setter
    def points(self, _):
        pass

    def on_trajectory(self, message):
        if not message.pending_start:
            self.trajectories.append([message.identity.path_seq, message.spline.traj_id])

    def cmd(self, message):
        super().cmd(message)
        self.commands.append([message.header.stamp.to_sec(), message.generation,
                              *xyz(message.position), *xyz(message.velocity),
                              *xyz(message.acceleration), message.yaw, int(message.terminal)])


def main(out):
    assert os.environ.get('ROS_MASTER_URI') == 'http://127.0.0.1:11445'
    rospy.init_node('inspection_isolated_test')
    # The existing synthetic harness publishes map_id=synthetic. Adjust just
    # that publisher so test11 still verifies its production map identifier.
    map_id = rospy.get_param('/drone_nav/config/map_id')
    original = rospy.Publisher

    class RelocResultPublisher:
        def __init__(self, publisher):
            self.publisher = publisher

        def publish(self, message):
            message.data = json.dumps(dict(confirmed=True, map=map_id))
            self.publisher.publish(message)

    def publisher(topic, *args, **kwargs):
        pub = original(topic, *args, **kwargs)
        return RelocResultPublisher(pub) if topic == '/drone_reloc/result' else pub

    rospy.Publisher = publisher
    try:
        harness = InspectionHarness()
    finally:
        rospy.Publisher = original
    try:
        harness.wait(lambda: 'READY' in harness.status.get('input_status', ''), 15, 'input transform and cloud')
        rospy.wait_for_service('/drone_nav/prepare', timeout=10)
        prepare = rospy.ServiceProxy('/drone_nav/prepare', Trigger)
        harness.wait(lambda: prepare().success, 10, 'synthetic PX4 alignment')
        script = Path(__file__).resolve().parents[1] / 'nav_outdoor_inspection.py'
        child = subprocess.Popen([sys.executable, '-B', str(script), '--preview', '--out', str(out)])
        result = child.wait(timeout=255)
        assert result == 0, (result, harness.status)
        pubs, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
        assert not any(topic == '/mavros/setpoint_raw/local' for topic, _ in pubs)
        records = list(out.glob('goal_test11_*.json'))
        assert len(records) == 1
        record = json.loads(records[0].read_text())
        assert record['end'] == 'SUCCEEDED'
        assert [leg['end'] for leg in record['legs']] == ['SUCCEEDED']*5
        assert all(leg['hold_elapsed_s'] >= 3. for leg in record['legs'][:3])
        assert record['legs'][-1]['kind'] == 'return_heading'
        rows = np.array(harness.commands)
        metrics = dict(length=0., max_heading_error_deg=0., max_yaw_rate_deg_s=0., moving_turn_samples=0)
        for generation in np.unique(rows[:, 1]):
            if generation < 2**32:
                continue
            samples = rows[rows[:, 1] == generation]
            metrics['length'] += np.linalg.norm(np.diff(samples[:, 2:5], axis=0), axis=1).sum()
            moving = np.linalg.norm(samples[:, 5:7], axis=1) >= .03
            error = [abs(wrap(yaw - math.atan2(vy, vx))) for vx, vy, yaw in samples[moving][:, [5, 6, 11]]]
            metrics['max_heading_error_deg'] = max(metrics['max_heading_error_deg'], math.degrees(max(error)))
            dt = np.diff(samples[:, 0])
            yaw_delta = np.abs(np.diff(np.unwrap(samples[:, 11])))
            rates = np.rad2deg(yaw_delta[dt > .005] / dt[dt > .005])
            metrics['max_yaw_rate_deg_s'] = max(metrics['max_yaw_rate_deg_s'], rates.max())
            metrics['moving_turn_samples'] += int(np.sum(moving[1:] & (yaw_delta > math.radians(.05))))
            assert np.max(np.linalg.norm(samples[:, 5:8], axis=1)) <= 1.001
            assert np.max(np.linalg.norm(samples[:, 8:11], axis=1)) <= 1.001
            assert np.max(np.abs(samples[:, [7, 10]])) <= .3003
        assert 30 <= metrics['length'] <= 45, metrics
        assert metrics['max_heading_error_deg'] < .001, metrics
        assert metrics['moving_turn_samples'] >= 30, metrics
        assert len({g for g in rows[:, 1] if g >= 2**32}) == 4
        assert len(harness.trajectories) > 4
        (out/'metrics.json').write_text(json.dumps(metrics, indent=2)+'\n')
        print('PASS complete test11 preview:', metrics, flush=True)
    finally:
        harness.stop = True
        if 'child' in locals() and child.poll() is None:
            child.terminate()
            child.wait(timeout=5)
        np.savetxt(out/'commands.csv', np.asarray(harness.commands).reshape((-1, 13)),
                   delimiter=',', header='time,generation,x,y,z,vx,vy,vz,ax,ay,az,yaw,terminal', comments='')
        (out/'trajectories.json').write_text(json.dumps(harness.trajectories)+'\n')


if __name__ == '__main__':
    main(Path(sys.argv[1]))
