#!/usr/bin/env python3
"""无人机建图：拉起 MID-360S 驱动 + FAST-LIO2（只在本次打开 PCD 保存）+ 重力桥，同时录原始包；
Ctrl+C 收尾后把地图摆正、降采样、出预览图，全部放进 ~/test_data/mapping/<时间>/。

背景（2026-10-18 核对）：
- pcd_save_en 在 FAST_LIO/config/mid360s.yaml 里，fusion.launch 和 start_mid360s_fastlio.sh 都加载它；
  直接改 yaml，飞行时也会一直攒点。本脚本不改 yaml，只在自己生成的 launch 里覆盖这一个参数。
- FAST-LIO 只在节点正常退出时把攒下的点一次写成 src/FAST_LIO/PCD/scans.pcd（camera_init 系，
  point_filter_num 抽点后、未降采样，每分钟约 400 万点），每次运行覆盖同名文件。
- camera_init 不水平（雷达前倾约 19° 安装），摆正用重力桥发的 world→camera_init（/tf_static），
  它和 /fastlio/gravity_init 一起录进包里。world：z 朝上，原点 = 起点时的雷达 IMU，yaw 不动。

用法:
  build_map.py [--name NAME]                          实机建图（雷达要上电），Ctrl+C 结束
  build_map.py --bag RAW.bag [--rate R] [--name NAME] 用录好的原始包离线重建（只回放 /livox/lidar、/livox/imu）
  build_map.py --post DIR                             只重做 DIR 的后处理
  共同选项: --voxel 0.05（摆正后地图的体素边长，m）  --out-root ~/test_data/mapping
  实机会先等 chrony 对上时间（同 fusion.sh，最多 60 s）；确定没外网也要建图，加 --allow-unsynced-clock
  --set NAME=VALUE  只对本次覆盖 FAST-LIO 参数，可重复，例如 --set preprocess/blind=1.0 --set filter_size_surf=0.3
产出（DIR = out-root/<YYYYmmdd_HHMMSS>[_NAME]）:
  raw.bag        实机：/livox/lidar /livox/imu /tf_static /fastlio/gravity_init /Odometry
  odom.bag       离线：只有后三个话题（原始数据就是 --bag 给的那个包）
  scans.pcd      FAST-LIO 原样输出（camera_init 系，稠密）
  map_world.pcd  摆正到 world 并体素降采样，字段 x y z intensity
  preview.png    俯视 + 侧视 + 轨迹
  meta.json      点数、world←camera_init、地面拟合（起点离地高度、残余倾斜）、首尾位姿差、git 版本
  params.yaml / build_map.launch / roslaunch.log / rosbag.log / build_map.log
退出码: 0 地图已存好  1 没拿到地图  2 用法错误  3 前置检查不通过
"""

import argparse
import fcntl
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import time

import numpy as np

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROS_SETUP = '/opt/ros/noetic/setup.bash'
WS_SETUP = os.path.join(WS, 'devel/setup.bash')
PCD_DIR = os.path.join(WS, 'src/FAST_LIO/PCD')        # FAST-LIO 编译时写死的 ROOT_DIR/PCD
BOARD_IP, LIDAR_NET_IP, LIDAR_IP = '192.168.31.32', '192.168.1.5', '192.168.1.194'
MASTER_PORT = 11311
LOCK_FILE = '/tmp/mid360s_build_map.lock'
REC_BIN, PLAY_BIN = '/opt/ros/noetic/lib/rosbag/record', '/opt/ros/noetic/lib/rosbag/play'
RAW_TOPICS = ['/livox/lidar', '/livox/imu']
META_TOPICS = ['/tf_static', '/fastlio/gravity_init', '/Odometry']
NODES = ['/laserMapping', '/gravity_bridge']
DRIVER_NODE = '/livox_lidar_publisher2'
CONFLICT_PROCS = ['livox_ros_driver2_node', 'fastlio_mapping', 'gravity_bridge_node']
SOFT_LIMIT = 300.0       # 建图超过 5 min 提醒收尾
STILL_AFTER_INIT = 3.0   # 收到第一帧里程计后再静止这么久，保证 raw.bag 开头有静止段（离线重建要靠它初始化）
STOP_TIMEOUT = 60.0      # 等 roslaunch 收尾的上限；FAST-LIO 退出时才写 scans.pcd
MIN_FREE_GB = 5.0
CLOCK_SYNC_TIMEOUT = 60.0  # 等 chrony 对上（s）；开机约 30 s 完成，和 fusion.sh 一致
CLOCK_MAX_OFFSET = 0.05    # 对上之后允许的剩余偏差（s），超过说明还在大幅修正
FLOOR_MIN_PTS = 1000     # 地面拟合少于这么多体素就不可信（台架上录的包只扫到一小块）

LAUNCH = '''<launch>
  <!-- build_map.py 生成。和 start_mid360s_fastlio.sh 的联合 launch 相同，只多两处：
       覆盖 mid360s.yaml 的 pcd_save（只对本次生效）；离线回放时不带驱动 -->
%(head)s
  <include file="$(find fast_lio)/launch/mapping_mid360s.launch">
    <arg name="rviz" value="false"/>
  </include>
  <param name="/pcd_save/pcd_save_en" type="bool" value="true"/>
  <param name="/pcd_save/interval" type="int" value="-1"/>
%(extra)s
  <include file="$(find fastlio_gravity_bridge)/launch/gravity_bridge.launch">
    <arg name="send_to_mavros" value="false"/>
  </include>
</launch>
'''
DRIVER = '''  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
    <arg name="rviz_enable" value="false"/>
    <arg name="rosbag_enable" value="false"/>
  </include>'''
# 离线回放不用仿真时间：FAST-LIO 只认消息时间戳，但主循环用 ros::Rate，仿真时间下回放一停 /clock 不走，
# SIGINT 后出不了循环，15 s 后被 roslaunch 改发 SIGTERM，地图就存不下来（10-18 实测）

LOG = {'f': None}
STOP = {'sig': None}


def say(msg):
    line = '[%s] %s' % (time.strftime('%H:%M:%S'), msg)
    try:                                   # ssh 断了以后 stdout 写不出去，不能因此中断收尾
        print(line, flush=True)
    except OSError:
        pass
    if LOG['f']:
        LOG['f'].write(line + '\n')
        LOG['f'].flush()


def ensure_ros_env():
    """没 source 过 ROS 就套一层 bash 重新执行自己；同时按 start_mid360s_fastlio.sh 设 ROS_IP，本机 RViz 能看"""
    if os.environ.get('BUILD_MAP_ENV') == '1':
        return
    for f in (ROS_SETUP, WS_SETUP):
        if not os.path.isfile(f):
            print('[ERROR] 找不到 %s（工作空间没编译？）' % f)
            sys.exit(3)
    ip = BOARD_IP if BOARD_IP in subprocess.run(['ip', '-4', 'addr'], stdout=subprocess.PIPE,
                                                universal_newlines=True).stdout else '127.0.0.1'
    cmd = ('source %s && source %s && export BUILD_MAP_ENV=1 ROS_MASTER_URI=http://%s:%d ROS_IP=%s '
           '&& unset ROS_HOSTNAME && exec python3 "$0" "$@"' % (ROS_SETUP, WS_SETUP, ip, MASTER_PORT, ip))
    os.execvp('bash', ['bash', '-c', cmd, os.path.abspath(__file__)] + sys.argv[1:])


# ---------------------------------------------------------------- 后处理 ----
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
    e = np.arange(np.percentile(zb, 0.2), -0.05 + 0.02, 0.02)
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


def post(d, voxel, run=None):
    """scans.pcd + 包里的 world←camera_init → map_world.pcd / preview.png / meta.json；返回 meta"""
    t0 = time.time()
    meta_path = os.path.join(d, 'meta.json')
    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    if run:
        meta['run'] = run
    bag = next((os.path.join(d, b) for b in ('raw.bag', 'odom.bag') if os.path.isfile(os.path.join(d, b))), None)
    tf, grav, odom = read_bag(bag) if bag else (None, None, np.zeros((0, 8)))
    if bag is None:
        say('[WARN] 目录里没有 raw.bag / odom.bag，无法摆正')
    Rg = R_from_gravity(grav) if grav else None
    if tf:
        R, t, src = quat_R(*tf[0]), np.array(tf[1]), 'tf_static world->camera_init'
    elif Rg is not None:
        R, t, src = Rg, np.zeros(3), 'gravity_init（tf_static 缺失，按最小旋转推算）'
        say('[WARN] 包里没有 world→camera_init，改用 /fastlio/gravity_init 推算')
    else:
        R, t, src = np.eye(3), np.zeros(3), 'none（没摆正）'
        say('[WARN] 包里既没有 tf_static 也没有 gravity_init，map_world.pcd 没有摆正')
    meta['world_from_camera_init'] = dict(q_xyzw=R_quat(R), t=t.tolist(), source=src)
    if grav:
        meta['gravity_init'] = list(grav)
        dR = Rg.T @ R
        meta['tf_vs_gravity_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))

    p, inten = read_pcd(os.path.join(d, 'scans.pcd'))
    pw = (p @ R.T.astype(np.float32)) + t.astype(np.float32)
    q, qi = voxel_down(pw, inten, voxel)
    write_pcd(os.path.join(d, 'map_world.pcd'), q, qi)
    fl = floor_fit(q)
    meta.update(voxel_m=voxel, points_raw=int(len(p)), points_world=int(len(q)),
                extent_world=dict(min=q.min(axis=0).tolist(), max=q.max(axis=0).tolist()), floor=fl)

    traj = np.zeros((0, 3))
    if len(odom):
        traj = odom[:, 1:4] @ R.T + t
        step = np.linalg.norm(np.diff(traj, axis=0), axis=1)

        def yaw(row):  # body 的 x 轴投到水平面上的朝向
            ax = R @ quat_R(*row[4:8])[:, 0]
            return math.atan2(ax[1], ax[0])
        dyaw = (yaw(odom[-1]) - yaw(odom[0]) + math.pi) % (2 * math.pi) - math.pi
        meta['trajectory'] = dict(n=int(len(odom)), duration_s=float(odom[-1, 0] - odom[0, 0]), path_m=float(step.sum()),
                                  start_world=traj[0].tolist(), end_world=traj[-1].tolist(),
                                  end_minus_start_m=(traj[-1] - traj[0]).tolist(),
                                  end_minus_start_dist_m=float(np.linalg.norm(traj[-1] - traj[0])),
                                  yaw_change_deg=math.degrees(dyaw))
    tr = meta.get('trajectory')
    text = ('points raw %d -> world %d (voxel %.2f m)   frame: world = start lidar IMU, z up\n' % (len(p), len(q), voxel)
            + ('trajectory %.0f s, %.1f m, end-start %.3f m / yaw %+.1f deg   ' %
               (tr['duration_s'], tr['path_m'], tr['end_minus_start_dist_m'], tr['yaw_change_deg']) if tr else '')
            + ('floor %.3f m below start, tilt %.2f deg, rms %.3f m, %d pts%s' % (fl['height_m'], fl['tilt_deg'], fl['rms_m'],
               fl['n'], '' if fl['trusted'] else ' (too few, untrusted)') if fl else 'floor not found'))
    preview(os.path.join(d, 'preview.png'), os.path.basename(os.path.normpath(d)), q, traj, fl, text)
    with open(meta_path, 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)
    say('后处理 %.1f s：原始 %d 点 → 摆正降采样 %d 点（%.2f m 体素），范围 x %.1f…%.1f、y %.1f…%.1f、z %.1f…%.1f m'
        % (time.time() - t0, len(p), len(q), voxel, *[v for pair in zip(q.min(axis=0), q.max(axis=0)) for v in pair]))
    if 'tf_vs_gravity_deg' in meta:
        say('world←camera_init 来自 %s；和按重力推算的差 %.3f°' % (src, meta['tf_vs_gravity_deg']))
    if tr:
        say('轨迹 %.0f s、%.1f m；终点 − 起点 = %.3f m、yaw %+.2f°（回到起点停的话就是漂移）'
            % (tr['duration_s'], tr['path_m'], tr['end_minus_start_dist_m'], tr['yaw_change_deg']))
    say('地面：起点离地 %.3f m，残余倾斜 %.2f°，rms %.3f m（%d 点%s）' % (fl['height_m'], fl['tilt_deg'], fl['rms_m'], fl['n'],
        '' if fl['trusted'] else '，太少，不可信') if fl else '地面：没找到（起点下方没有大的水平面）')
    return meta


# ---------------------------------------------------------------- 运行 ----
def master_up():
    try:
        socket.create_connection((BOARD_IP, MASTER_PORT), timeout=1).close()
        return True
    except OSError:
        pass
    try:
        socket.create_connection(('127.0.0.1', MASTER_PORT), timeout=1).close()
        return True
    except OSError:
        return False


def rosnodes():
    try:
        r = subprocess.run(['rosnode', 'list'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                           universal_newlines=True, timeout=8)
        return set(r.stdout.split())
    except (subprocess.TimeoutExpired, OSError):
        return set()


def git_rev():
    try:
        r = subprocess.run(['git', '-C', WS, 'rev-parse', '--short', 'HEAD'], stdout=subprocess.PIPE,
                           universal_newlines=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', '-C', WS, 'status', '--porcelain', '-uno'], stdout=subprocess.PIPE,
                               universal_newlines=True, timeout=5).stdout.strip()
        return r + ('-dirty' if dirty else '')
    except (subprocess.TimeoutExpired, OSError):
        return ''


def clock_status():
    """(Leap status, |System time 偏差| s)；chronyc 不可用返回 None"""
    try:
        out = subprocess.run(['chronyc', '-n', 'tracking'], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             universal_newlines=True, timeout=3).stdout
    except (subprocess.TimeoutExpired, OSError):
        return None
    leap = off = None
    for line in out.splitlines():
        k, _, v = line.partition(':')
        if k.strip() == 'Leap status':
            leap = v.strip()
        elif k.strip() == 'System time' and v.split():
            off = abs(float(v.split()[0]))
    return (leap, off) if leap is not None and off is not None else None


def wait_clock(allow):
    """雷达/IMU 用板端系统时间打戳（没跑 PTP）：建图中 chrony 一跳，FAST-LIO 出错、桥接 fatal、录的包时间也乱。
    板端 RTC 掉电不保持，开机约 30 s 才跳到正确时间（10-18 这次 +3748 s），所以先等它对上"""
    t0, why, said = time.monotonic(), '', False
    while True:
        st = clock_status()
        if st is None:
            why = 'chronyc 不可用'
        elif st[0] == 'Not synchronised':
            why = 'chrony 还没同步上时间源'
        elif st[1] < CLOCK_MAX_OFFSET:
            print('[OK] 板端时间已由 chrony 对上（剩余偏差 %.6f s）' % st[1], flush=True)
            return True
        else:
            why = 'chrony 已同步，但剩余偏差 %.3f s，还在修正' % st[1]
        if time.monotonic() - t0 >= CLOCK_SYNC_TIMEOUT:
            break
        if not said:
            said = True
            print('板端时间还没对上（%s），最多等 %.0f s……' % (why, CLOCK_SYNC_TIMEOUT), flush=True)
        time.sleep(2)
    if allow:
        print('[WARN] 板端时间没对上（%s），按 --allow-unsynced-clock 继续；建图中时间一跳，这次地图就可能作废' % why)
        return True
    print('[ERROR] 板端时间没对上（%s）。确认外网后重试；确定没外网也要建图，加 --allow-unsynced-clock' % why)
    return False


def preflight(a):
    bad = []
    if a.bag:
        import rosbag
        try:
            with rosbag.Bag(a.bag) as b:
                info = b.get_type_and_topic_info().topics
        except Exception as e:  # noqa: BLE001
            bad.append('读不了 %s：%s' % (a.bag, e))
            info = {}
        for t in RAW_TOPICS:
            if info and t not in info:
                bad.append('%s 里没有 %s' % (a.bag, t))
        if '/livox/lidar' in info and info['/livox/lidar'].msg_type != 'livox_ros_driver2/CustomMsg':
            bad.append('/livox/lidar 是 %s，FAST-LIO 这里要 livox_ros_driver2/CustomMsg' % info['/livox/lidar'].msg_type)
    else:
        addr = subprocess.run(['ip', '-4', 'addr'], stdout=subprocess.PIPE, universal_newlines=True).stdout
        if LIDAR_NET_IP not in addr:
            bad.append('板端没有雷达网口地址 %s（enP3p49s0 DOWN / 网线没插）' % LIDAR_NET_IP)
        elif subprocess.run(['ping', '-c', '1', '-W', '2', LIDAR_IP], stdout=subprocess.DEVNULL).returncode:
            bad.append('雷达 %s ping 不通（没上电？）' % LIDAR_IP)
    if master_up():
        bad.append('已经有 ROS master 在跑（fusion / start_mid360s_fastlio / roscore？），先停掉')
    for n in CONFLICT_PROCS:
        if subprocess.run(['pgrep', '-f', n], stdout=subprocess.DEVNULL).returncode == 0:
            bad.append('已有 %s 进程在跑' % n)
    os.makedirs(a.out_root, exist_ok=True)
    free = shutil.disk_usage(a.out_root).free / 1e9
    if free < MIN_FREE_GB:
        bad.append('%s 只剩 %.1f GB' % (a.out_root, free))
    return bad


def check_override(launch):
    """确认生成的 launch 里 pcd_save_en 最终是 true（roslaunch 解析，不启动任何节点）"""
    import yaml
    r = subprocess.run(['roslaunch', '--dump-params', launch], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                       universal_newlines=True, timeout=30)
    try:
        params = yaml.safe_load(r.stdout) or {}
    except yaml.YAMLError:
        params = {}
    return params.get('/pcd_save/pcd_save_en') is True, r.stdout[-500:]


def stop_proc(p, name, timeout):
    """先 SIGINT（每 5 s 补一次），超时再 SIGTERM / SIGKILL；返回退出码"""
    if p is None or p.poll() is not None:
        return None if p is None else p.returncode
    say('SIGINT -> %s（pid %d）' % (name, p.pid))
    t0, last = time.time(), 0.0
    while p.poll() is None and time.time() - t0 < timeout:
        if time.time() - last >= 5.0:
            p.send_signal(signal.SIGINT)
            last = time.time()
        time.sleep(0.2)
    if p.poll() is None:
        say('[WARN] %s %.0f s 内没退出，SIGTERM' % (name, timeout))
        p.terminate()
        try:
            p.wait(10)
        except subprocess.TimeoutExpired:
            say('[WARN] %s 还在，SIGKILL' % name)
            p.kill()
            p.wait()
    return p.returncode


class Odo:
    """订阅 /Odometry：判断 FAST-LIO 初始化完成、打印离起点多远"""

    def __init__(self):
        import rospy
        from nav_msgs.msg import Odometry
        self.first = self.last = self.prev = None
        self.t_first, self.path = None, 0.0
        rospy.init_node('build_map_monitor', anonymous=True, disable_signals=True)
        self.sub = rospy.Subscriber('/Odometry', Odometry, self.cb, queue_size=5)

    def cb(self, m):
        p = m.pose.pose.position
        v = np.array([p.x, p.y, p.z])
        if self.first is None:
            self.first, self.t_first, self.prev = v, time.time(), v
        if np.linalg.norm(v - self.prev) > 0.05:
            self.path += float(np.linalg.norm(v - self.prev))
            self.prev = v
        self.last = v

    def status(self):
        return '离起点 %.2f m，已走 %.1f m' % (float(np.linalg.norm(self.last - self.first)), self.path)


def run(a):
    stamp = time.strftime('%Y%m%d_%H%M%S')
    name = a.name or ('bag_' + os.path.splitext(os.path.basename(a.bag))[0] if a.bag else '')
    out = os.path.join(a.out_root, stamp + ('_' + name if name else ''))
    os.makedirs(out)
    LOG['f'] = open(os.path.join(out, 'build_map.log'), 'a')
    say('输出目录 %s' % out)
    launch = os.path.join(out, 'build_map.launch')
    with open(launch, 'w') as f:
        f.write(LAUNCH % {'head': '' if a.bag else DRIVER,
                          'extra': ''.join('  <param name="/%s" value="%s"/>\n' % kv for kv in a.sets)})
    ok, dump = check_override(launch)
    if not ok:
        say('[ERROR] 生成的 launch 里 pcd_save_en 没被覆盖成 true，不启动。roslaunch --dump-params 输出末尾：\n' + dump)
        return 3
    for fn in sorted(os.listdir(PCD_DIR)):
        if fn.startswith('scans') and fn.endswith('.pcd'):  # 上次手动开 pcd_save 留下的，挪走免得和本次混
            src = os.path.join(PCD_DIR, fn)
            dst = os.path.join(a.out_root, 'leftover_%s_%s' % (time.strftime('%Y%m%d_%H%M%S', time.localtime(os.path.getmtime(src))), fn))
            shutil.move(src, dst)
            say('[WARN] %s 里有旧的 %s，已挪到 %s' % (PCD_DIR, fn, dst))

    run_info = dict(mode='bag' if a.bag else 'live', source_bag=os.path.abspath(a.bag) if a.bag else None,
                    rate=a.rate if a.bag else None, start=time.strftime('%F %T'), git=git_rev(), host=socket.gethostname(),
                    chrony=clock_status(), fastlio_set=dict(a.sets))
    t_start = time.time()
    rl_log = open(os.path.join(out, 'roslaunch.log'), 'w')
    bag_log = open(os.path.join(out, 'rosbag.log'), 'w')
    rl = subprocess.Popen(['roslaunch', '--screen', '--required', launch], stdout=rl_log, stderr=subprocess.STDOUT,
                          stdin=subprocess.DEVNULL, start_new_session=True)
    rec = play = odo = None
    code = 1
    try:
        need = NODES + ([] if a.bag else [DRIVER_NODE])
        t_end = time.time() + 40
        while STOP['sig'] is None and rl.poll() is None and time.time() < t_end:
            if not set(need) - rosnodes():
                break
            time.sleep(1.0)
        miss = set(need) - rosnodes()
        if STOP['sig'] is not None or rl.poll() is not None or miss:
            say('[ERROR] 节点没起全（缺 %s，roslaunch %s），看 %s' % (' '.join(sorted(miss)) or '无',
                '已退出' if rl.poll() is not None else '在运行', os.path.join(out, 'roslaunch.log')))
            return 1
        subprocess.run(['rosparam', 'dump', os.path.join(out, 'params.yaml')], timeout=15)
        got = subprocess.run(['rosparam', 'get', '/pcd_save/pcd_save_en'], stdout=subprocess.PIPE,
                             universal_newlines=True, timeout=10).stdout.strip()
        if got != 'true':
            say('[ERROR] 运行时 /pcd_save/pcd_save_en = %r，地图不会保存，停止' % got)
            return 1
        topics = META_TOPICS if a.bag else RAW_TOPICS + META_TOPICS
        rec = subprocess.Popen([REC_BIN, '-O', os.path.join(out, 'odom.bag' if a.bag else 'raw.bag')] + topics,
                               stdout=bag_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        odo = Odo()
        time.sleep(1.5)  # 等录包节点连上话题
        if a.bag:
            play = subprocess.Popen([PLAY_BIN, '-q', '-d', '2', '-r', str(a.rate), a.bag, '--topics'] + RAW_TOPICS,
                                    stdout=bag_log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
            say('开始回放 %s（%.1f 倍速）；放完自动收尾，Ctrl+C 提前结束' % (a.bag, a.rate))
        else:
            say('节点已起来，正在录包。请保持静止，等 FAST-LIO 初始化……')
        t_nodes, ready, warned, t_last, t_ready = time.time(), False, set(), 0.0, None
        while STOP['sig'] is None:
            if rl.poll() is not None:
                say('[ERROR] roslaunch 自己退出了（%s），看 roslaunch.log' % rl.returncode)
                break
            if rec.poll() is not None:
                say('[ERROR] 录包进程退出了（%s），看 rosbag.log；停止建图' % rec.returncode)
                break
            if play is not None and play.poll() is not None:
                say('回放结束，3 s 后收尾')
                time.sleep(3.0)
                break
            now = time.time()
            if odo.first is None:
                if now - t_nodes > 20 and 'noodom' not in warned:
                    warned.add('noodom')
                    say('[WARN] 20 s 了还没有 /Odometry：雷达没数据，或者 IMU 一直没初始化完')
                if now - t_nodes > 90:
                    say('[ERROR] 90 s 没有 /Odometry，停止')
                    break
            elif not ready and (a.bag or now - odo.t_first >= STILL_AFTER_INIT):
                ready, t_ready = True, now
                if not a.bag:
                    say('初始化完成，可以开始走了。慢慢走、别猛转，低处也扫到；最后回到起点再按 Ctrl+C')
            if ready and now - t_last >= 10.0:
                t_last = now
                say('[%4.0f s] %s' % (now - t_ready, odo.status()))
                if not a.bag and now - t_ready > SOFT_LIMIT and 'soft' not in warned:
                    warned.add('soft')
                    say('[WARN] 已经超过 %.0f min，建议回到起点收尾（scans.pcd 是一次写出，点越多停得越慢）' % (SOFT_LIMIT / 60))
            time.sleep(0.5)
    finally:
        say('收尾：先停录包，再停 FAST-LIO（它退出时写 scans.pcd）')
        stop_proc(play, 'rosbag play', 10)
        stop_proc(rec, 'rosbag record', 20)
        if odo is not None:  # 先退掉监视节点，免得 master 停了以后 rospy 一直报连不上
            import rospy
            odo.sub.unregister()
            rospy.signal_shutdown('build_map 收尾')
        rc = stop_proc(rl, 'roslaunch', STOP_TIMEOUT)
        run_info.update(end=time.strftime('%F %T'), roslaunch_rc=rc,
                        stop=('signal %d' % STOP['sig']) if STOP['sig'] else 'auto')
        src = os.path.join(PCD_DIR, 'scans.pcd')
        if os.path.isfile(src) and os.path.getmtime(src) >= t_start:
            shutil.move(src, os.path.join(out, 'scans.pcd'))
            say('scans.pcd %.0f MB 已挪进输出目录' % (os.path.getsize(os.path.join(out, 'scans.pcd')) / 1e6))
            try:
                post(out, a.voxel, run_info)
                code = 0
            except Exception as e:  # noqa: BLE001  后处理失败不影响已经存下的原始地图，可以 --post 重做
                say('[ERROR] 后处理失败：%s（scans.pcd 和包都在，可以 build_map.py --post %s 重做）' % (e, out))
        else:
            say('[ERROR] 没有拿到本次的 scans.pcd（FAST-LIO 没正常退出，或者没收到点）')
            with open(os.path.join(out, 'meta.json'), 'w') as f:
                json.dump({'run': run_info}, f, ensure_ascii=False, indent=1)
        for fn in ('raw.bag.active', 'odom.bag.active'):
            if os.path.exists(os.path.join(out, fn)):
                say('[WARN] %s 没收尾，可以 rosbag reindex 试试' % fn)
        rl_log.close()
        bag_log.close()
        os.sync()
        say('已 sync。%s' % ('完成：' + out if code == 0 else '没完成，看 ' + out))
    return code


def main():
    ensure_ros_env()
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = p.add_mutually_exclusive_group()
    g.add_argument('--bag', help='用录好的原始包离线重建')
    g.add_argument('--post', metavar='DIR', help='只重做 DIR 的后处理')
    p.add_argument('--rate', type=float, default=1.0, help='--bag 回放倍速，默认 1')
    p.add_argument('--name', default='', help='输出目录名后缀')
    p.add_argument('--voxel', type=float, default=0.05)
    p.add_argument('--out-root', default=os.path.expanduser('~/test_data/mapping'))
    p.add_argument('--allow-unsynced-clock', action='store_true', help='实机：板端时间没对上也照样建图')
    p.add_argument('--set', action='append', default=[], metavar='NAME=VALUE', help='只对本次覆盖 FAST-LIO 参数，可重复')
    a = p.parse_args()
    a.sets = []
    for kv in a.set:
        k, sep, v = kv.partition('=')
        if not sep or not k.strip('/ ') or not v.strip():
            print('--set 要写成 NAME=VALUE：%r' % kv)
            return 2
        a.sets.append((k.strip('/ '), v.strip()))
    if a.post:
        LOG['f'] = open(os.path.join(a.post, 'build_map.log'), 'a')
        say('重做后处理 %s' % a.post)
        try:
            post(a.post, a.voxel)
        except Exception as e:  # noqa: BLE001
            say('[ERROR] %s' % e)
            return 1
        os.sync()
        return 0
    if a.bag and not os.path.isfile(a.bag):
        print('找不到 %s' % a.bag)
        return 2
    lock = open(LOCK_FILE, 'a')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        print('[ERROR] 已有 build_map.py 在跑（%s 被占用）' % LOCK_FILE)
        return 3
    if not a.bag and not wait_clock(a.allow_unsynced_clock):
        return 3
    bad = preflight(a)
    if bad:
        for b in bad:
            print('[ERROR] ' + b)
        return 3

    def on_sig(sig, _frm):
        if STOP['sig'] is None:
            STOP['sig'] = sig
            say('收到 %s，开始收尾，请等它跑完（存图 + 后处理约 10–60 s）' % signal.Signals(sig).name)
        else:
            say('收尾进行中，不要再按')
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):  # ssh 断开（SIGHUP）也按正常收尾存图
        signal.signal(s, on_sig)
    return run(a)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:  # 还没进入建图（前置检查、等对时）时按 Ctrl+C
        sys.exit(130)
