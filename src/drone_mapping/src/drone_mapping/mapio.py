"""地图读写和后处理的公用函数（原 scripts/build_map.py 里的，10-20 挪进 drone_mapping 包；drone_reloc.py 也用）：
PCD 读写、体素降采样、地面拟合、预览图、四元数/重力摆正，以及读原始包里的 /tf_static、/fastlio/gravity_init、/Odometry。"""
import math
import os

import numpy as np

META_TOPICS = ['/tf_static', '/fastlio/gravity_init', '/Odometry']
FLOOR_MIN_PTS = 1000     # 地面拟合少于这么多体素就不可信（台架上录的包只扫到一小块）


def quat_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def R_quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return [x, y, z, w]


def R_from_gravity(g):
    """把重力方向转到 -z 的最小旋转（和重力桥的做法一致），tf 缺失时兜底、平时用来交叉核对"""
    a = np.asarray(g, float) / np.linalg.norm(g)
    b = np.array([0.0, 0.0, -1.0])
    v, c = np.cross(a, b), float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K / (1 + c)


def read_bag(path):
    import rosbag
    tf = grav = None
    odom = []
    with rosbag.Bag(path) as b:
        for topic, m, _ in b.read_messages(topics=META_TOPICS):
            if topic == '/tf_static':
                for tr in m.transforms:
                    if tr.header.frame_id.strip('/') == 'world' and tr.child_frame_id.strip('/') == 'camera_init':
                        q, t = tr.transform.rotation, tr.transform.translation
                        tf = ((q.x, q.y, q.z, q.w), (t.x, t.y, t.z))
            elif topic == '/fastlio/gravity_init':
                grav = (m.vector.x, m.vector.y, m.vector.z)
            else:
                p, q = m.pose.pose.position, m.pose.pose.orientation
                odom.append((m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    return tf, grav, np.array(odom).reshape(-1, 8)


def read_pcd(path):
    """PCL 的 binary PCD（FAST-LIO 用 writeBinary 写）→ (N×3 float32, N intensity)"""
    hdr = {}
    with open(path, 'rb') as f:
        while True:
            line = f.readline()
            if not line:
                raise ValueError('%s 没有 DATA 行' % path)
            s = line.decode('ascii', 'replace').strip()
            if not s or s.startswith('#'):
                continue
            k, _, v = s.partition(' ')
            hdr[k.upper()] = v.split()
            if k.upper() == 'DATA':
                break
        off = f.tell()
    if hdr['DATA'][0] != 'binary':
        raise ValueError('只支持 binary PCD，这个是 %s' % hdr['DATA'][0])
    names, sizes, types = hdr['FIELDS'], [int(s) for s in hdr['SIZE']], hdr['TYPE']
    counts = [int(c) for c in hdr.get('COUNT', ['1'] * len(names))]
    dt = []
    for i, (n, s, t, c) in enumerate(zip(names, sizes, types, counts)):
        fmt = '<' + {'F': 'f', 'U': 'u', 'I': 'i'}[t] + str(s)
        dt.append((n if n != '_' else '_pad%d' % i, fmt, (c,)) if c > 1 else (n if n != '_' else '_pad%d' % i, fmt))
    dt = np.dtype(dt)
    n = int(hdr['POINTS'][0])
    need = off + n * dt.itemsize
    if os.path.getsize(path) < need:
        raise ValueError('scans.pcd 不完整（%d / %d 字节），FAST-LIO 可能没写完就被杀了' % (os.path.getsize(path), need))
    a = np.memmap(path, dtype=dt, mode='r', offset=off, shape=(n,))
    p = np.stack([a['x'], a['y'], a['z']], axis=1).astype(np.float32)
    inten = np.asarray(a['intensity'], np.float32) if 'intensity' in names else np.zeros(n, np.float32)
    ok = np.isfinite(p).all(axis=1)
    return p[ok], inten[ok]


def write_pcd(path, p, inten):
    n = len(p)
    hdr = ('# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
           'TYPE F F F F\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (n, n))
    body = np.empty((n, 4), '<f4')
    body[:, :3], body[:, 3] = p, inten
    with open(path, 'wb') as f:
        f.write(hdr.encode('ascii'))
        f.write(body.tobytes())


def voxel_down(p, inten, v):
    k = np.floor(p / v).astype(np.int64)
    k -= k.min(axis=0)
    m = k.max(axis=0) + 1
    key = k[:, 0] + m[0] * (k[:, 1] + m[1] * k[:, 2])
    u, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    out = np.empty((len(u), 3), np.float32)
    for j in range(3):
        out[:, j] = np.bincount(inv, p[:, j], len(u)) / cnt
    return out, (np.bincount(inv, inten, len(u)) / cnt).astype(np.float32)


def floor_fit(p):
    """地面 = 起点下方最低的一个大的水平面（原点是起点时的雷达 IMU，地面一定在它下面）"""
    z = p[:, 2]
    zb = z[z < -0.05]
    if len(zb) < 500:
        return None
    e = np.arange(np.percentile(zb, 0.2) - 0.1, -0.05 + 0.02, 0.02)   # 往下留 0.1 m：没有噪点时地面峰会落在第一格，找峰会漏掉
    if len(e) < 4:
        return None
    h, e = np.histogram(zb, e)
    hs = np.convolve(h, [1, 2, 1], 'same') / 4.0
    peaks = [i for i in range(1, len(hs) - 1) if hs[i] >= 0.3 * hs.max() and hs[i] >= hs[i - 1] and hs[i] >= hs[i + 1]]
    if not peaks:
        return None
    sel = np.abs(z - 0.5 * (e[peaks[0]] + e[peaks[0] + 1])) < 0.06
    coef = res = None
    for tol in (0.06, 0.03):
        if sel.sum() < 100:
            return None
        A = np.c_[p[sel, 0], p[sel, 1], np.ones(int(sel.sum()))]
        coef = np.linalg.lstsq(A, z[sel], rcond=None)[0]
        res = z - (coef[0] * p[:, 0] + coef[1] * p[:, 1] + coef[2])
        sel = np.abs(res) < tol
    return dict(height_m=-float(coef[2]), tilt_deg=math.degrees(math.atan(math.hypot(coef[0], coef[1]))),
                n=int(sel.sum()), rms_m=float(np.sqrt(np.mean(res[sel] ** 2))), trusted=bool(sel.sum() >= FLOOR_MIN_PTS))


def preview(path, title, p, traj, fl, text):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    lo, hi = np.percentile(p, 0.5, axis=0) - 0.5, np.percentile(p, 99.5, axis=0) + 0.5  # 远处零星离群点不撑大画面
    if len(traj):
        lo, hi = np.minimum(lo, traj.min(axis=0) - 0.5), np.maximum(hi, traj.max(axis=0) + 0.5)
    p = p[((p >= lo) & (p <= hi)).all(axis=1)]
    fig = plt.figure(figsize=(10, 11), dpi=110)
    gs = fig.add_gridspec(2, 1, height_ratios=[3, 1.1])
    for k, (i, j, name) in enumerate([(0, 1, 'top view (x-y)'), (0, 2, 'side view (x-z)')]):
        ax = fig.add_subplot(gs[k])
        b = 0.05 if k == 0 else 0.03
        H, xe, ye = np.histogram2d(p[:, i], p[:, j], [np.arange(lo[i], hi[i] + b, b), np.arange(lo[j], hi[j] + b, b)])
        ax.imshow(np.log1p(H.T), origin='lower', extent=(xe[0], xe[-1], ye[0], ye[-1]), cmap='Greys',
                  interpolation='nearest', aspect='equal')
        if len(traj):
            ax.plot(traj[:, i], traj[:, j], color='#D62728', lw=1.0, label='trajectory')
            ax.plot(traj[0, i], traj[0, j], 'o', mfc='none', mec='#1F77B4', ms=9, mew=2, label='start (origin)')
            ax.plot(traj[-1, i], traj[-1, j], 'x', color='#1F77B4', ms=9, mew=2, label='end')
        if k == 1 and fl:
            ax.axhline(-fl['height_m'], color='#2CA02C', ls='--', lw=1, label='floor fit')
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)' if k == 0 else 'z (m)')
        ax.set_title(title + '  ' + name if k == 0 else name, fontsize=10)
        ax.grid(True, lw=0.3, alpha=0.5)
        ax.legend(loc='upper right', fontsize=8)
    fig.text(0.01, 0.005, text, fontsize=8, family='monospace', va='bottom')
    fig.tight_layout(rect=(0, 0.06, 1, 1))
    fig.savefig(path)
    plt.close(fig)
