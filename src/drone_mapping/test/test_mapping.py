"""drone_mapping 的离线单元测试（不需要 ROS master、雷达和原始包）：
python3 -m unittest discover -s src/drone_mapping/test"""
import math
import os
import sys
import tempfile
import unittest

import numpy as np

sys.dont_write_bytecode = True
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
from drone_mapping import assemble, mapio, voxelslam  # noqa: E402


def rotz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


class TestMapio(unittest.TestCase):
    def test_floor_fit_recovers_tilt(self):
        rng = np.random.default_rng(0)
        xy = rng.uniform(-3, 3, (40000, 2))
        a, b = math.tan(math.radians(0.4)), math.tan(math.radians(-0.3))
        z = a * xy[:, 0] + b * xy[:, 1] - 0.16 + rng.normal(0, 0.005, len(xy))
        wall = np.c_[np.full(5000, 3.0), rng.uniform(-3, 3, 5000), rng.uniform(-0.1, 2.5, 5000)]
        fl = mapio.floor_fit(np.r_[np.c_[xy, z], wall])
        self.assertTrue(fl['trusted'])
        self.assertAlmostEqual(fl['height_m'], 0.16, delta=0.003)
        self.assertAlmostEqual(fl['tilt_deg'], math.degrees(math.atan(math.hypot(a, b))), delta=0.02)

    def test_voxel_down_averages(self):
        p = np.array([[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [0.11, 0.0, 0.0]], np.float32)
        q, qi = mapio.voxel_down(p, np.array([1, 3, 5], np.float32), 0.05)
        self.assertEqual(len(q), 2)
        k = np.argmin(q[:, 0])
        np.testing.assert_allclose(q[k], [0.02, 0.02, 0.02], atol=1e-6)
        self.assertAlmostEqual(float(qi[k]), 2.0)

    def test_gravity_rotation(self):
        g = np.array([0.3, -0.2, -9.7])
        R = mapio.R_from_gravity(g)
        np.testing.assert_allclose(R @ (g / np.linalg.norm(g)), [0, 0, -1], atol=1e-12)
        np.testing.assert_allclose(mapio.quat_R(*mapio.R_quat(R)), R, atol=1e-6)   # R_quat 小角度时开方损失约 1e-8


class TestAssemble(unittest.TestCase):
    def test_to_world_undistorts_moving_scan(self):
        """匀速平移 + 匀角速度转 yaw 时，各时刻看到的同一个世界点要拼回同一处（插值是精确的）"""
        tt = np.array([0.0, 0.1, 0.2])
        traj = np.zeros((3, 8))
        traj[:, 0] = tt
        for i, t in enumerate(tt):
            traj[i, 1:4] = [0.5 * t, 0.2 * t, 0.0]
            yaw = 0.8 * t
            traj[i, 4:8] = [0, 0, math.sin(yaw / 2), math.cos(yaw / 2)]
        X = np.array([2.0, 1.0, 0.5])
        T = np.linspace(0.0, 0.2, 9)
        body = np.array([rotz(0.8 * t).T @ (X - np.array([0.5 * t, 0.2 * t, 0.0])) for t in T])
        W = assemble.to_world(body, T, traj, np.eye(3))
        np.testing.assert_allclose(W, np.tile(X, (len(T), 1)), atol=1e-12)

    def test_state_and_params(self):
        with tempfile.TemporaryDirectory() as d:
            row = [1.0, 0, 0, 0, 0, 0, 0, 1] + [0] * 9 + [0.01, -0.02, -9.8] + [0] * 6
            np.savetxt(os.path.join(d, 's.txt'), np.array([row, row]))
            traj, g = assemble.read_state(os.path.join(d, 's.txt'))
            self.assertEqual(traj.shape, (2, 8))
            np.testing.assert_allclose(g, [0.01, -0.02, -9.8])
            cfg = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                               'config', 'voxelslam_mid360s.yaml')
            prm = assemble.load_params(cfg)
            np.testing.assert_allclose(prm['T'], [-0.011, -0.02329, 0.04412])
            np.testing.assert_array_equal(prm['R'], np.eye(3))
            self.assertEqual((prm['blind'], prm['every']), (0.5, 3))

    def test_launch_xml_well_formed(self):
        """生成的 launch 和包里的 launch 都要是合法 XML（注释里写 -- 会让 roslaunch 解析失败，10-20 踩过）"""
        import xml.etree.ElementTree as ET
        from drone_mapping import build
        for head in ('', build.DRIVER):
            for pcd in ('', build.PCD_SAVE):
                root = ET.fromstring(build.LAUNCH % {'head': head, 'pcd': pcd, 'extra': '  <param name="/a" value="1"/>\n'})
                self.assertEqual(root.tag, 'launch')
        pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        ET.parse(os.path.join(pkg, 'launch', 'voxelslam.launch'))

    def test_count_frames(self):
        with tempfile.TemporaryDirectory() as d:
            for n in ('0.pcd', '1.pcd', 'alidarState.txt'):
                open(os.path.join(d, n), 'w').close()
            self.assertEqual(voxelslam.count_frames(d), 2)
            self.assertEqual(voxelslam.count_frames(os.path.join(d, 'nope')), 0)


if __name__ == '__main__':
    unittest.main()
