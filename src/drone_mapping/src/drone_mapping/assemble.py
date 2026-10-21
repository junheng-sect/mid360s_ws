"""Voxel-SLAM 轨迹 + 原始包 → map_world.pcd / preview.png / meta.json（格式同 FAST-LIO 那一路，drone_reloc 直接用）。

- 位姿：alidarState.txt 每行 t x y z qx qy qz qw …，是 IMU 在扫描结束时刻的位姿；第 18–20 列是它估的重力。
- 点：取法和 Voxel-SLAM / FAST-LIO 的预处理一样（tag 正常、每 point_filter_num 点取 1、离雷达 > blind），
  按轨迹逐点插值去畸变；外参、blind、point_filter_num 取本次 Voxel-SLAM 参数（voxelslam.yaml）。
  不用 Voxel-SLAM 自己存的逐帧点云：那是 0.1 m 体素平均过的，点更稀，面也显得更薄（10-19 对比时统一用原始点）。
- 地图系 = 建图那次的 drone_odom：z 朝上，原点 = 起点时的雷达 IMU，yaw 不动（同 FAST-LIO 那一路）；只按它估的重力再摆一次（通常 < 0.1°）。
  不按地面二次摆正：10-20 试过，重定位的可信次数反而变少。
- 去人（10-21 加，carve）：手持建图时拿设备的人会被扫进图里。设备走过的空间里不会有静止物体，所以离轨迹水平
  CARVE_R、上下 CARVE_H 以内（扁椭球），且只在不到 CARVE_SECS 个不同的秒里被看到的 CARVE_CELL 格子整格去掉。
  - 只看距离会误删：10-21 那次设备从一个静止物体旁约 0.1 m 擦过，它全程 144 s 都看得到，靠「被看到的秒数」留下。
  - 上下只取 ±0.12 m：起点时雷达 IMU 离地约 0.16 m，地面不能切；比设备低很多的东西（桌面等）受 FOV 限制本来就
    只在走近时短暂可见，没法按时长区分，所以不往下延伸。人在设备高度的部分（手臂、肩、头）能去掉，腿和腰留着。
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

CARVE_R = 0.25      # 去人：离轨迹的水平半径（m），0 关闭
CARVE_H = 0.12      # 去人：上下半高（m），要小于起点时雷达 IMU 离地的高度（10-21 那次 0.16 m）
CARVE_SECS = 5      # 去人：格子在少于这么多个不同的秒里被看到才算人（静止物体基本每秒都看得到）
CARVE_CELL = 0.1    # 去人按这么大的格子判断、整格去掉；是地图体素 0.05 的整数倍，格子互相嵌套。
                    # 不用 0.05：静止物体边缘的 0.05 小格只偶尔被扫到，10-21 那次会把擦边物体削掉约 35 个点


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
    """IMU 系的点 xyz（时刻 T）→ 地图系：在前后两个位姿之间插值，再按重力摆正"""
    tt = traj[:, 0]
    j = np.clip(np.searchsorted(tt, T) - 1, 0, len(tt) - 2)
    w = ((T - tt[j]) / (tt[j + 1] - tt[j])).clip(0, 1)
    R = slerp_R(traj[j, 4:8], traj[j + 1, 4:8], w)
    pos = traj[j, 1:4] * (1 - w)[:, None] + traj[j + 1, 1:4] * w[:, None]
    return (np.einsum('nij,nj->ni', R, xyz) + pos) @ Rg.T


def gather(bag, traj, prm, Rg):
    """逐帧解原始 CustomMsg（不经 rospy 反序列化），返回地图系的点、强度、时间（相对轨迹起点，s）、用到的雷达帧数"""
    import rosbag
    t_lo, t_hi = traj[0, 0], traj[-1, 0]
    rot = not np.array_equal(prm['R'], np.eye(3))
    pts, inten, tim, n_msg = [], [], [], 0
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
            tim.append((T[k] - t_lo).astype(np.float32))
            n_msg += 1
    if not pts:
        raise RuntimeError('%s 里没有落在轨迹时间段内的雷达点' % bag)
    return np.concatenate(pts), np.concatenate(inten), np.concatenate(tim), n_msg


def path_samples(tw, step):
    """轨迹折线按不超过 step 的间隔加密"""
    seg = np.diff(tw, axis=0)
    n = np.maximum(1, np.ceil(np.linalg.norm(seg, axis=1) / step).astype(np.int64))
    a = np.repeat(np.arange(len(seg)), n)
    f = (np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n) + 1) / np.repeat(n, n)
    return np.r_[tw[:1], tw[a] + seg[a] * f[:, None]]


def near_path(c, tw, r, h):
    """c 里哪些点在轨迹 tw 的水平 r、上下 h 扁椭球以内：z 放大 r/h 倍后就是半径 r 的管子"""
    s = np.array([1.0, 1.0, r / h])
    q, cs = path_samples(tw * s, r / 4), c * s
    B = 1 << 18

    def key(k):
        k = k + B
        return k[:, 0] + 2 * B * (k[:, 1] + 2 * B * k[:, 2])
    # 粗筛：边长 r 的格子，只有轨迹样本所在格子及其 26 邻格里的点才逐个算距离
    off = np.array([(i, j, l) for i in (-1, 0, 1) for j in (-1, 0, 1) for l in (-1, 0, 1)])
    kq = np.floor(q / r).astype(np.int64)
    idx = np.nonzero(np.isin(key(np.floor(cs / r).astype(np.int64)), np.unique(key((kq[:, None] + off).reshape(-1, 3)))))[0]
    cand, hit = cs[idx], np.zeros(len(idx), bool)
    for i in range(0, len(q), 64):   # 轨迹按 64 个样本（约 4 m）一段，段的包围盒里的候选点才算距离，内存有界
        qq = q[i:i + 64]
        j = np.nonzero(~hit & np.all((cand >= qq.min(0) - r) & (cand <= qq.max(0) + r), axis=1))[0]
        if len(j):
            hit[j[((cand[j, None] - qq[None]) ** 2).sum(-1).min(1) < r * r]] = True
    out = np.zeros(len(c), bool)
    out[idx[hit]] = True
    return out


def carve(P, T, tw, r=CARVE_R, h=CARVE_H, secs=CARVE_SECS, cell=CARVE_CELL):
    """去人：返回要保留的点（bool）和统计。格子是 floor(p / cell)，整格去掉"""
    k = np.floor(P / cell).astype(np.int64)
    k -= k.min(axis=0)
    m = k.max(axis=0) + 1
    _, inv, cnt = np.unique(k[:, 0] + m[0] * (k[:, 1] + m[1] * k[:, 2]), return_inverse=True, return_counts=True)
    c = np.stack([np.bincount(inv, P[:, j], len(cnt)) for j in range(3)], 1) / cnt[:, None]
    near = near_path(c, tw, r, h)
    sel = near[inv]
    sec = np.floor(T[sel]).astype(np.int64)
    w = int(sec.max()) + 1 if len(sec) else 1
    nsec = np.bincount(np.unique(inv[sel] * w + sec) // w, minlength=len(cnt))   # 每格被看到的不同秒数
    drop = near & (nsec < secs)
    keep = ~drop[inv]
    return keep, dict(radius_m=r, half_height_m=h, min_secs=secs, cell_m=cell, cells_near_path=int(near.sum()),
                      cells_dropped=int(drop.sum()), points_dropped=int((~keep).sum()))


def build(d, bag, state, config, voxel, meta=None, say=print, carve_r=CARVE_R):
    """在目录 d 里写 map_world.pcd / preview.png / meta.json；meta 里已有的键（run、voxelslam 等）保留。
    carve_r：去人的水平半径，0 不去"""
    t0 = time.time()
    meta = dict(meta or {})
    traj, g = read_state(state)
    Rg = mapio.R_from_gravity(g)
    P, I, T, n_msg = gather(bag, traj, load_params(config), Rg)
    n_raw = len(P)
    tw = traj[:, 1:4] @ Rg.T
    cv = dict(radius_m=0)
    if carve_r > 0:
        keep, cv = carve(P, T, tw, carve_r)
        P, I = P[keep], I[keep]
        say('去人：轨迹附近 %d 个 %.2f m 格子里去掉 %d 个、%d 点（水平 %.2f m、上下 %.2f m 以内，只在不到 %d 个不同的秒里被看到）'
            % (cv['cells_near_path'], cv['cell_m'], cv['cells_dropped'], cv['points_dropped'], carve_r, cv['half_height_m'],
               cv['min_secs']))
    n_carved = n_raw - len(P)
    q, qi = mapio.voxel_down(P, I, voxel)
    mapio.write_pcd(os.path.join(d, 'map_world.pcd'), q, qi)
    fl = mapio.floor_fit(q)

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
        voxel_m=voxel, points_raw=int(n_raw), points_world=int(len(q)), carve=cv,
        extent_world=dict(min=q.min(axis=0).tolist(), max=q.max(axis=0).tolist()), floor=fl,
        trajectory=dict(n=int(len(traj)), duration_s=float(traj[-1, 0] - traj[0, 0]), path_m=float(step.sum()),
                        start_world=tw[0].tolist(), end_world=tw[-1].tolist(), end_minus_start_m=(tw[-1] - tw[0]).tolist(),
                        end_minus_start_dist_m=float(np.linalg.norm(tw[-1] - tw[0])), yaw_change_deg=math.degrees(dyaw)))
    tr = meta['trajectory']
    text = ('Voxel-SLAM: points raw %d -> map %d (voxel %.2f m)%s   frame: drone_odom = start lidar IMU, z up\n'
            % (n_raw, len(q), voxel, '   carve -%d raw pts near path' % n_carved if carve_r > 0 else '')
            + 'trajectory %.0f s, %.1f m, end-start %.3f m / yaw %+.1f deg   ' %
            (tr['duration_s'], tr['path_m'], tr['end_minus_start_dist_m'], tr['yaw_change_deg'])
            + ('floor %.3f m below start, tilt %.2f deg, rms %.3f m, %d pts%s' % (fl['height_m'], fl['tilt_deg'], fl['rms_m'],
               fl['n'], '' if fl['trusted'] else ' (too few, untrusted)') if fl else 'floor not found'))
    mapio.preview(os.path.join(d, 'preview.png'), os.path.basename(os.path.normpath(d)), q, tw, fl, text)
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    say('拼图 %.1f s：%d 帧 %d 点 → %d 点（%.2f m 体素），范围 x %.1f…%.1f、y %.1f…%.1f、z %.1f…%.1f m'
        % (time.time() - t0, n_msg, n_raw, len(q), voxel, *[v for pair in zip(q.min(axis=0), q.max(axis=0)) for v in pair]))
    say('轨迹 %.0f s、%.1f m；终点 − 起点 = %.3f m、yaw %+.2f°（回到起点停的话就是闭环差）'
        % (tr['duration_s'], tr['path_m'], tr['end_minus_start_dist_m'], tr['yaw_change_deg']))
    say('地面：起点离地 %.3f m，残余倾斜 %.2f°，rms %.3f m（%d 点%s）' % (fl['height_m'], fl['tilt_deg'], fl['rms_m'], fl['n'],
        '' if fl['trusted'] else '，太少，不可信') if fl else '地面：没找到（起点下方没有大的水平面）')
    return meta
