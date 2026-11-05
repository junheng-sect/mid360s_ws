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
- 去静止旁人（11-05 加，carve_still）：设备放在地上不动时，操作者常在旁边站着、走动，离轨迹远，上面那条去不掉。
  设备静止 ≥ STILL_SECS 的时段前后各放宽 STILL_PAD 秒；静止位置水平 STILL_R 以内、离当地地面 STILL_Z 以内的格子，
  只在这些时段里被看到、其余时间又有 ≥ STILL_SEEN 次射线穿过它打到后面，就是当时在那儿的人，整格去掉。
  - 只看「射线穿过」会误删：贴着地面、墙面掠射的射线和透光的树丛都能穿过真东西。所以还要「只在静止前后出现过」，
    并且不挨着（26 邻域）其他时间也看得到的格子：墙、树丛上漏看的格子旁边一定有同一面墙。
  - 11-05 室外图只去掉起点西侧的人（1.2 万个原始点）；10-20、10-21 两张旧图同样只去掉站在起点旁边的人。
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
STILL_SECS = 3.0    # 去静止旁人：设备至少静止这么久（前后 0.5 s 位移 < STILL_MOVE）才算放着
STILL_MOVE = 0.03
STILL_PAD = 5.0     # 静止前后各放宽这么多秒：拿起、放下那几秒人就在旁边
STILL_R = 8.0       # 只看静止位置水平这么远以内
STILL_Z = (0.25, 2.2)   # 只看离当地地面（0.5 m 柱子里最低 1% 的点）这个高度范围：地面、树冠不动
STILL_SEEN = 5      # 其余时间至少这么多次（每 0.05 s 一组）射线穿过格子、打到它后面 0.15 m 以外


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


def still_windows(tt, tw, secs=STILL_SECS, move=STILL_MOVE):
    """设备静止（前后 0.5 s 位移 < move）且持续 ≥ secs 的时段，返回轨迹下标 [(i0, i1), ...]"""
    j0 = np.searchsorted(tt, tt - 0.5)
    j1 = np.clip(np.searchsorted(tt, tt + 0.5), 0, len(tt) - 1)
    still = np.linalg.norm(tw[j1] - tw[j0], axis=1) < move
    e = np.flatnonzero(np.diff(np.r_[0, still.astype(np.int8), 0]))
    return [(int(a), int(b) - 1) for a, b in zip(e[::2], e[1::2]) if tt[b - 1] - tt[a] >= secs]


def dir_bin(v, r, ang, ne):
    """方向 → 角度格编号（方位、俯仰各 ang 弧度一格）"""
    return (np.floor((np.arctan2(v[:, 1], v[:, 0]) + np.pi) / ang).astype(np.int64) * ne
            + np.floor((np.arcsin(np.clip(v[:, 2] / np.maximum(r, 1e-6), -1, 1)) + np.pi / 2) / ang).astype(np.int64))


def carve_still(P, T, tt, tw, cell=CARVE_CELL, sub=0.05, ang=math.radians(0.5), margin=0.15, blind=0.5):
    """去静止旁人：返回要保留的点（bool）和统计。T、tt 是相对轨迹起点的秒，tw 是地图系的轨迹；
    射线起点用 IMU 位置（雷达外参约 2 cm，忽略）。格子同 carve，是 floor(p / cell)，整格去掉"""
    W = still_windows(tt, tw)
    stat = dict(windows_s=[[round(float(tt[a]), 1), round(float(tt[b]), 1)] for a, b in W], pad_s=STILL_PAD,
                radius_m=STILL_R, height_m=list(STILL_Z), min_seen=STILL_SEEN, cell_m=cell,
                cells_candidate=0, cells_dropped=0, points_dropped=0)
    if not W:
        return np.ones(len(P), bool), stat
    near_t = np.zeros(len(T), bool)
    for a, b in W:
        near_t |= (T >= tt[a] - STILL_PAD) & (T <= tt[b] + STILL_PAD)
    k = np.floor(P / cell).astype(np.int64)
    k -= k.min(axis=0) - 1      # 四周空一格，邻格编号不会绕到另一边
    m = k.max(axis=0) + 2
    key, inv, cnt = np.unique(k[:, 0] + m[0] * (k[:, 1] + m[1] * k[:, 2]), return_inverse=True, return_counts=True)
    c = np.stack([np.bincount(inv, P[:, j], len(cnt)) for j in range(3)], 1) / cnt[:, None]
    seen_out = np.zeros(len(cnt), bool)     # 静止前后以外也被看到过
    seen_out[inv[~near_t]] = True
    # 当地地面：每个 0.5 m 柱子里最低 1% 的点
    kc = np.floor(P[:, :2] / 0.5).astype(np.int64)
    kc -= kc.min(axis=0)
    col = kc[:, 0] * (kc[:, 1].max() + 1) + kc[:, 1]
    o = np.lexsort((P[:, 2], col))
    first = np.flatnonzero(np.r_[True, col[o][1:] != col[o][:-1]])
    ground = np.zeros(col.max() + 1)
    ground[col[o[first]]] = P[o[first + (np.diff(np.r_[first, len(o)]) * 0.01).astype(np.int64)], 2]
    col_cell = np.zeros(len(cnt), np.int64)
    col_cell[inv] = col
    hg = c[:, 2] - ground[col_cell]
    ctr = np.array([tw[a:b + 1].mean(axis=0) for a, b in W])
    rh = np.hypot(c[:, None, 0] - ctr[None, :, 0], c[:, None, 1] - ctr[None, :, 1]).min(axis=1)
    cand = np.flatnonzero(~seen_out & (rh <= STILL_R) & (hg > STILL_Z[0]) & (hg < STILL_Z[1]))
    through = np.zeros(len(cand), np.int32)
    if len(cand):
        # 静止前后以外的点每 sub 秒一组，射线起点取这组中间时刻的轨迹位置；每个方向格只看最近的点，
        # 它都打在候选格子后面 margin 以外，才算这组看穿了这个格子
        cp = c[cand]
        ne = int(math.ceil(math.pi / ang)) + 2
        g = np.floor(T[~near_t] / sub).astype(np.int64)
        o = np.argsort(g, kind='stable')
        Po, g = P[~near_t][o], g[o]
        st = np.flatnonzero(np.r_[True, g[1:] != g[:-1]])
        for s, e in zip(st, np.r_[st[1:], len(g)]):
            org = np.array([np.interp((g[s] + 0.5) * sub, tt, tw[:, j]) for j in range(3)])
            v = Po[s:e] - org
            r = np.linalg.norm(v, axis=1)
            b = dir_bin(v, r, ang, ne)
            oo = np.lexsort((r, b))
            b, r = b[oo], r[oo]
            f = np.r_[True, b[1:] != b[:-1]]
            ub, mr = b[f], r[f]
            vc = cp - org
            rc = np.linalg.norm(vc, axis=1)
            bc = dir_bin(vc, rc, ang, ne)
            i = np.clip(np.searchsorted(ub, bc), 0, len(ub) - 1)
            through += (ub[i] == bc) & (mr[i] > rc + margin) & (rc > blind)
    off = np.array([i + m[0] * (j + m[1] * l) for i in (-1, 0, 1) for j in (-1, 0, 1) for l in (-1, 0, 1) if i or j or l])
    tied = np.isin(key[cand][:, None] + off[None], key[seen_out]).any(axis=1)
    drop = np.zeros(len(cnt), bool)
    drop[cand[(through >= STILL_SEEN) & ~tied]] = True
    keep = ~drop[inv]
    stat.update(cells_candidate=int(len(cand)), cells_dropped=int(drop.sum()), points_dropped=int((~keep).sum()))
    return keep, stat


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
    cv, cs = dict(radius_m=0), {}
    if carve_r > 0:
        keep, cv = carve(P, T, tw, carve_r)
        P, I, T = P[keep], I[keep], T[keep]
        say('去人：轨迹附近 %d 个 %.2f m 格子里去掉 %d 个、%d 点（水平 %.2f m、上下 %.2f m 以内，只在不到 %d 个不同的秒里被看到）'
            % (cv['cells_near_path'], cv['cell_m'], cv['cells_dropped'], cv['points_dropped'], carve_r, cv['half_height_m'],
               cv['min_secs']))
        keep, cs = carve_still(P, T, traj[:, 0] - traj[0, 0], tw)
        P, I = P[keep], I[keep]
        win = '、'.join('%.1f–%.1f' % tuple(w) for w in cs['windows_s']) or '无'
        say('去静止旁人：设备静止 %d 段（%s s），附近 %d 个候选格子里去掉 %d 个、%d 点（只在静止前后 %.0f s 内出现过、'
            '其余时间被看穿 ≥ %d 次、不挨着其他时间也看得到的格子）'
            % (len(cs['windows_s']), win, cs['cells_candidate'], cs['cells_dropped'], cs['points_dropped'], cs['pad_s'],
               cs['min_seen']))
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
        voxel_m=voxel, points_raw=int(n_raw), points_world=int(len(q)), carve=cv, carve_still=cs,
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
