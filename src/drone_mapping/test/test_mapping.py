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

    def test_read_bag_accepts_old_tf_name(self):
        # 10-21 起重力桥发 drone_odom→camera_init，之前的包叫 world→camera_init，两种都要读到
        import rosbag
        import rospy
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        for parent in ('drone_odom', 'world', 'map'):
            tr = TransformStamped()
            tr.header.frame_id, tr.child_frame_id = parent, 'camera_init'
            tr.transform.translation.z = 0.25
            tr.transform.rotation.w = 1.0
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, 'meta.bag')
                with rosbag.Bag(path, 'w') as b:
                    b.write('/tf_static', TFMessage(transforms=[tr]), rospy.Time(1))
                tf, _, _ = mapio.read_bag(path)
            if parent == 'map':
                self.assertIsNone(tf)
            else:
                self.assertEqual(tf, ((0.0, 0.0, 0.0, 1.0), (0.0, 0.0, 0.25)))


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

    def test_near_path_matches_segments(self):
        """扁椭球判定和到折线的精确距离一致：加密间隔 r/4 只会在边界上差几毫米，而且只会漏、不会多"""
        rng = np.random.default_rng(1)
        tw = np.cumsum(rng.normal(0, 0.3, (60, 3)) * [1, 1, 0.2], axis=0)
        c = tw[rng.integers(0, 60, 4000)] + rng.uniform(-0.5, 0.5, (4000, 3))
        r, h = 0.25, 0.12
        got = assemble.near_path(c, tw, r, h)
        s = np.array([1, 1, r / h])
        p, a, b = c * s, tw[:-1] * s, tw[1:] * s
        t = np.clip(((p[:, None] - a) * (b - a)).sum(-1) / ((b - a) ** 2).sum(-1), 0, 1)
        d = np.linalg.norm(p[:, None] - (a + t[..., None] * (b - a)), axis=-1).min(1)
        self.assertGreater(got.sum(), 500)
        self.assertFalse((got & (d >= r)).any())
        self.assertTrue(np.all(np.abs(d[got != (d < r)] - r) < 0.005))

    def test_carve_drops_transient_keeps_static(self):
        """沿 x 走 10 s：轨迹上只出现 1 s 的点（人）去掉；全程都看得到的擦边静止物体、
        轨迹下方 0.16 m 的地面（起点离地高度）、水平 0.4 m 外的墙都留下"""
        tw = np.c_[np.linspace(0, 4, 101), np.zeros(101), np.full(101, 1.2)]
        groups = {'person': ([2.0, 0.0, 1.25], np.linspace(5.0, 5.9, 30)),
                  'static': ([3.0, 0.1, 1.2], np.linspace(0.0, 9.9, 30)),
                  'floor': ([1.0, 0.0, 1.04], np.linspace(1.0, 1.9, 30)),
                  'wall': ([1.5, 0.4, 1.2], np.linspace(1.0, 1.9, 30))}
        P = np.concatenate([np.tile(xyz, (30, 1)) for xyz, _ in groups.values()]).astype(np.float32)
        T = np.concatenate([t for _, t in groups.values()]).astype(np.float32)
        keep, cv = assemble.carve(P, T, tw)
        keep = dict(zip(groups, keep.reshape(4, 30)))
        self.assertFalse(keep['person'].any())
        for g in ('static', 'floor', 'wall'):
            self.assertTrue(keep[g].all(), g)
        self.assertEqual((cv['cells_near_path'], cv['cells_dropped'], cv['points_dropped']), (2, 1, 30))

    def test_carve_still_drops_bystander(self):
        """原点放 10 s 再沿 +x 走开：只在静止时出现、后来被看穿的人去掉；没被看穿的（挡住）、
        挨着一直看得到的墙的、地面都留下"""
        tt = np.linspace(0, 30, 301)
        tw = np.c_[np.clip((tt - 10) * 0.2, 0, None), np.zeros_like(tt), np.zeros_like(tt)]
        t_still = np.linspace(0, 9, 30)
        groups = {'person': ([-2.0, 0.0, 1.0], t_still), 'hidden': ([0.0, -3.0, 1.0], t_still),
                  'edge': ([-4.0, 1.0, 1.0], t_still), 'wall': ([-4.0, 1.1, 1.0], np.linspace(0, 29, 30))}
        for name, xy in (('g_person', [-2.0, 0.0]), ('g_hidden', [0.0, -3.0]), ('g_edge', [-4.0, 1.0])):
            groups[name] = (xy + [-0.2], np.linspace(0, 29, 30))
        P = [np.tile(xyz, (len(t), 1)) for xyz, t in groups.values()]
        T = [t for _, t in groups.values()]
        n = sum(len(t) for t in T)
        # 走开以后每 0.05 s 一束激光穿过人和 edge 打到后面 1 m（hidden 那边没有）
        for t in np.arange(15.025, 30, 0.05):
            o = np.array([np.interp(t, tt, tw[:, j]) for j in range(3)])
            for c in ([-2.0, 0.0, 1.0], [-4.0, 1.0, 1.0]):
                u = np.array(c) - o
                P.append([o + u * (1 + 1.0 / np.linalg.norm(u))])
                T.append([t])
        P, T = np.concatenate(P).astype(np.float32), np.concatenate(T).astype(np.float32)
        keep, cs = assemble.carve_still(P, T, tt, tw)
        keep = dict(zip(groups, keep[:n].reshape(len(groups), 30)))
        self.assertFalse(keep['person'].any())
        for g in groups:
            if g != 'person':
                self.assertTrue(keep[g].all(), g)
        self.assertEqual(cs['windows_s'], [[0.0, 9.6]])     # 9.6 s 时往后 0.5 s 只走了 2 cm，还算静止
        self.assertEqual((cs['cells_candidate'], cs['cells_dropped'], cs['points_dropped']), (3, 1, 30))

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
