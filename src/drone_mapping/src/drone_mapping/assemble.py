"""Voxel-SLAM 轨迹 + 原始包 → map_world.pcd / preview.png / meta.json（格式同 FAST-LIO 那一路，drone_reloc 直接用）。

- 位姿：alidarState.txt 每行 t x y z qx qy qz qw …，是 IMU 在扫描结束时刻的位姿；第 18–20 列是它估的重力。
- 点：取法和 Voxel-SLAM / FAST-LIO 的预处理一样（tag 正常、每 point_filter_num 点取 1、离雷达 > blind），
  按轨迹逐点插值去畸变；外参、blind、point_filter_num 取本次 Voxel-SLAM 参数（voxelslam.yaml）。
  不用 Voxel-SLAM 自己存的逐帧点云：那是 0.1 m 体素平均过的，点更稀，面也显得更薄（10-19 对比时统一用原始点）。
- world：z 朝上，原点 = 起点时的雷达 IMU，yaw 不动（同 FAST-LIO 那一路）；只按它估的重力再摆一次（通常 < 0.1°）。
  不按地面二次摆正：10-20 试过，重定位的可信次数反而变少。
"""
import json
import math
import os
import struct
import time

import numpy as np

from drone_mapping import mapio

# livox_ros_driver2/CustomMsg 里 points 的一个元素：offset_time（相对 header.stamp，ns）x y z reflectivity tag line
PT = np.dtype([('t', '<u4'), ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('r', 'u1'), ('tag', 'u1'), ('line', 'u1')])


def read_state(path):
    st = np.loadtxt(path, ndmin=2)
    return st[:, :8], st[:, 17:20].mean(axis=0)


def load_params(path):
    import yaml
    with open(path) as f:
        g = (yaml.safe_load(f) or {}).get('General', {})
    return dict(T=np.array(g.get('extrinsic_tran', [0, 0, 0]), float),
                R=np.array(g.get('extrinsic_rota', [1, 0, 0, 0, 1, 0, 0, 0, 1]), float).reshape(3, 3),
                blind=float(g.get('blind', 0.5)), every=int(g.get('point_filter_num', 1)))


def slerp_R(q0, q1, a):
    """逐点四元数插值（x y z w），返回 N×3×3"""
    dot = np.sum(q0 * q1, 1)
    q1 = np.where(dot[:, None] < 0, -q1, q1)
    dot = np.abs(dot).clip(-1, 1)
    th = np.arccos(dot)
    s = np.sin(th)
    small = s < 1e-6
    w0 = np.where(small, 1 - a, np.sin((1 - a) * th) / np.where(small, 1, s))
    w1 = np.where(small, a, np.sin(a * th) / np.where(small, 1, s))
    q = w0[:, None] * q0 + w1[:, None] * q1
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q.T
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z); R[:, 0, 1] = 2 * (x * y - z * w); R[:, 0, 2] = 2 * (x * z + y * w)  # noqa: E702
    R[:, 1, 0] = 2 * (x * y + z * w); R[:, 1, 1] = 1 - 2 * (x * x + z * z); R[:, 1, 2] = 2 * (y * z - x * w)  # noqa: E702
    R[:, 2, 0] = 2 * (x * z - y * w); R[:, 2, 1] = 2 * (y * z + x * w); R[:, 2, 2] = 1 - 2 * (x * x + y * y)  # noqa: E702
    return R


def to_world(xyz, T, traj, Rg):
    """IMU 系的点 xyz（时刻 T）→ world：在前后两个位姿之间插值，再按重力摆正"""
    tt = traj[:, 0]
    j = np.clip(np.searchsorted(tt, T) - 1, 0, len(tt) - 2)
    w = ((T - tt[j]) / (tt[j + 1] - tt[j])).clip(0, 1)
    R = slerp_R(traj[j, 4:8], traj[j + 1, 4:8], w)
    pos = traj[j, 1:4] * (1 - w)[:, None] + traj[j + 1, 1:4] * w[:, None]
    return (np.einsum('nij,nj->ni', R, xyz) + pos) @ Rg.T


def gather(bag, traj, prm, Rg):
    """逐帧解原始 CustomMsg（不经 rospy 反序列化），返回 world 系点、强度、用到的雷达帧数"""
    import rosbag
    t_lo, t_hi = traj[0, 0], traj[-1, 0]
    rot = not np.array_equal(prm['R'], np.eye(3))
    pts, inten, n_msg = [], [], 0
    with rosbag.Bag(bag) as b:
        for _, raw, _ in b.read_messages(topics=['/livox/lidar'], raw=True):
            d = raw[1]
            secs, nsecs = struct.unpack_from('<II', d, 4)
            tb = secs + nsecs * 1e-9
            if tb + 0.11 < t_lo or tb > t_hi:
                continue
            off = 12 + 4 + struct.unpack_from('<I', d, 12)[0]      # header：seq、stamp、frame_id
            off += 8 + 4 + 4                                       # timebase、point_num、lidar_id + rsvd[3]
            n = struct.unpack_from('<I', d, off)[0]
            off += 4
            c = np.frombuffer(d, PT, count=n, offset=off)
            tg = c['tag'] & 0x30
            c = c[(tg == 0x00) | (tg == 0x10)][::prm['every']]
            xyz = np.stack([c['x'], c['y'], c['z']], 1).astype(np.float64)
            T = tb + c['t'] * 1e-9
            k = (np.einsum('ij,ij->i', xyz, xyz) > prm['blind'] ** 2) & (T >= t_lo) & (T <= t_hi)
            if not k.any():
                continue
            xyz = xyz[k] @ prm['R'].T if rot else xyz[k]
            pts.append(to_world(xyz + prm['T'], T[k], traj, Rg).astype(np.float32))
            inten.append(c['r'][k].astype(np.float32))
            n_msg += 1
    if not pts:
        raise RuntimeError('%s 里没有落在轨迹时间段内的雷达点' % bag)
    return np.concatenate(pts), np.concatenate(inten), n_msg


def build(d, bag, state, config, voxel, meta=None, say=print):
    """在目录 d 里写 map_world.pcd / preview.png / meta.json；meta 里已有的键（run、voxelslam 等）保留"""
    t0 = time.time()
    meta = dict(meta or {})
    traj, g = read_state(state)
    Rg = mapio.R_from_gravity(g)
    P, I, n_msg = gather(bag, traj, load_params(config), Rg)
    q, qi = mapio.voxel_down(P, I, voxel)
    mapio.write_pcd(os.path.join(d, 'map_world.pcd'), q, qi)
    fl = mapio.floor_fit(q)

    tw = traj[:, 1:4] @ Rg.T
    step = np.linalg.norm(np.diff(tw, axis=0), axis=1)

    def yaw(row):  # body 的 x 轴投到水平面上的朝向
        ax = Rg @ mapio.quat_R(*row[4:8])[:, 0]
        return math.atan2(ax[1], ax[0])
    dyaw = (yaw(traj[-1]) - yaw(traj[0]) + math.pi) % (2 * math.pi) - math.pi
    meta.update(
        algo='voxelslam', source_bag=os.path.abspath(bag),
        world_from_voxelslam=dict(q_xyzw=mapio.R_quat(Rg), t=[0.0, 0.0, 0.0],
                                  source='Voxel-SLAM 估计的重力（位姿文件第 18–20 列均值）转到 -z 的最小旋转'),
        gravity_voxelslam=g.tolist(), lidar_msgs_used=n_msg,
        voxel_m=voxel, points_raw=int(len(P)), points_world=int(len(q)),
        extent_world=dict(min=q.min(axis=0).tolist(), max=q.max(axis=0).tolist()), floor=fl,
        trajectory=dict(n=int(len(traj)), duration_s=float(traj[-1, 0] - traj[0, 0]), path_m=float(step.sum()),
                        start_world=tw[0].tolist(), end_world=tw[-1].tolist(), end_minus_start_m=(tw[-1] - tw[0]).tolist(),
                        end_minus_start_dist_m=float(np.linalg.norm(tw[-1] - tw[0])), yaw_change_deg=math.degrees(dyaw)))
    tr = meta['trajectory']
    text = ('Voxel-SLAM: points raw %d -> world %d (voxel %.2f m)   frame: world = start lidar IMU, z up\n'
            % (len(P), len(q), voxel)
            + 'trajectory %.0f s, %.1f m, end-start %.3f m / yaw %+.1f deg   ' %
            (tr['duration_s'], tr['path_m'], tr['end_minus_start_dist_m'], tr['yaw_change_deg'])
            + ('floor %.3f m below start, tilt %.2f deg, rms %.3f m, %d pts%s' % (fl['height_m'], fl['tilt_deg'], fl['rms_m'],
               fl['n'], '' if fl['trusted'] else ' (too few, untrusted)') if fl else 'floor not found'))
    mapio.preview(os.path.join(d, 'preview.png'), os.path.basename(os.path.normpath(d)), q, tw, fl, text)
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    say('拼图 %.1f s：%d 帧 %d 点 → %d 点（%.2f m 体素），范围 x %.1f…%.1f、y %.1f…%.1f、z %.1f…%.1f m'
        % (time.time() - t0, n_msg, len(P), len(q), voxel, *[v for pair in zip(q.min(axis=0), q.max(axis=0)) for v in pair]))
    say('轨迹 %.0f s、%.1f m；终点 − 起点 = %.3f m、yaw %+.2f°（回到起点停的话就是闭环差）'
        % (tr['duration_s'], tr['path_m'], tr['end_minus_start_dist_m'], tr['yaw_change_deg']))
    say('地面：起点离地 %.3f m，残余倾斜 %.2f°，rms %.3f m（%d 点%s）' % (fl['height_m'], fl['tilt_deg'], fl['rms_m'], fl['n'],
        '' if fl['trusted'] else '，太少，不可信') if fl else '地面：没找到（起点下方没有大的水平面）')
    return meta
