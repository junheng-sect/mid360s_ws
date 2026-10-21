"""无人机建图（rosrun drone_mapping build_map）：实机拉起 MID-360S 驱动 + FAST-LIO2 + 重力桥并录原始包；
Ctrl+C 收尾后离线跑 Voxel-SLAM，再拼图、摆正、降采样、出预览图，全部放进 ~/test_data/mapping/<时间>/。

背景：
- 10-19/20 用同样三个原始包离线对比：Voxel-SLAM 闭环差 2 mm、面厚度约 6 mm；FAST-LIO 是 8–11 cm、约 11 mm。
  实时 FAST-LIO 在起点静止的十几秒里就漂了 3–5 cm、yaw 1°，旧图整体偏了约 9 cm。所以建图换成 Voxel-SLAM。
- Voxel-SLAM 不发 nav_msgs/Odometry，飞行定位仍用 FAST-LIO。实机建图时 FAST-LIO 照样跑：给 raw.bag 录 /Odometry、
  在终端里报走了多远，但不再存它的点云。
- Voxel-SLAM 在录完以后离线跑（见 voxelslam.py），1 倍速回放，耗时约为录制时长再加 30 s；结果只取决于原始包。
- --algo fastlio 保留旧做法：FAST-LIO 的 pcd_save 只在本脚本生成的 launch 里打开（mid360s.yaml 不动，飞行不攒点），
  退出时写 src/FAST_LIO/PCD/scans.pcd（camera_init 系），按重力桥发的 world→camera_init（/tf_static）摆正。
- world：z 朝上，原点 = 起点时的雷达 IMU，yaw 不动（两种算法一样）。

用法:
  build_map [--name NAME]                          实机建图（雷达要上电），Ctrl+C 结束
  build_map --bag RAW.bag [--rate R] [--name NAME] 用录好的原始包离线重建（只用 /livox/lidar、/livox/imu）
  build_map --post DIR                             只重做 DIR 的拼图 / 后处理（比如换 --voxel），不重跑 SLAM
  共同选项: --algo voxelslam|fastlio（默认 voxelslam）  --voxel 0.05（体素边长，m）  --out-root ~/test_data/mapping
  --rate R        离线回放倍速，默认 1（只要不丢消息，倍速不改变结果；Voxel-SLAM 1 倍速约占 1 个核）
  --keep-frames   保留 Voxel-SLAM 的逐帧点云 voxelslam_tmp/（默认成功后删掉，约 0.4 MB/s）
  --carve R       去人：拼图时去掉设备走过空间里（水平 R、上下 0.12 m 以内）只短暂出现的点，默认 0.25，0 关闭
                  （拿设备的人；静止物体全程都看得到，不会被去掉。见 assemble.py；FAST-LIO 那一路不做）
  实机会先等 chrony 对上时间（同 fusion.sh，最多 60 s）；确定没外网也要建图，加 --allow-unsynced-clock
  --set NAME=VALUE  只对本次覆盖实时 FAST-LIO 的参数，可重复，例如 --set preprocess/blind=1.0 --set filter_size_surf=0.3
产出（DIR = out-root/<YYYYmmdd_HHMMSS>[_NAME]）:
  raw.bag             实机：/livox/lidar /livox/imu /tf_static /fastlio/gravity_init /Odometry
  map_world.pcd       world 系、体素降采样，字段 x y z intensity
  preview.png         俯视 + 侧视 + 轨迹
  meta.json           算法、点数、地面拟合（起点离地高度、残余倾斜）、首尾位姿差、git 版本
  voxelslam_state.txt Voxel-SLAM 位姿（--post 靠它重新拼图）；voxelslam.yaml 本次参数；voxelslam.log
  fastlio 时：scans.pcd（camera_init 系、稠密），离线另有 odom.bag（只有后三个话题）
  params.yaml / build_map.launch / roslaunch.log / rosbag.log / build_map.log
退出码: 0 地图已存好  1 没拿到地图  2 用法错误  3 前置检查不通过  130 中途放弃
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

from drone_mapping import assemble, mapio, voxelslam

PKG_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))   # src/drone_mapping
WS = os.path.dirname(os.path.dirname(PKG_DIR))
ROS_SETUP = '/opt/ros/noetic/setup.bash'
WS_SETUP = os.path.join(WS, 'devel/setup.bash')
PCD_DIR = os.path.join(WS, 'src/FAST_LIO/PCD')        # FAST-LIO 编译时写死的 ROOT_DIR/PCD
VS_CONFIG = os.path.join(PKG_DIR, 'config/voxelslam_mid360s.yaml')
VS_BIN = os.path.join(WS, 'devel/lib/voxel_slam/voxelslam')
BOARD_IP, LIDAR_NET_IP, LIDAR_IP = '192.168.31.32', '192.168.1.5', '192.168.1.194'
MASTER_PORT = 11311
LOCK_FILE = '/tmp/mid360s_build_map.lock'
REC_BIN, PLAY_BIN = '/opt/ros/noetic/lib/rosbag/record', '/opt/ros/noetic/lib/rosbag/play'
RAW_TOPICS = ['/livox/lidar', '/livox/imu']
META_TOPICS = mapio.META_TOPICS
NODES = ['/laserMapping', '/gravity_bridge']
DRIVER_NODE = '/livox_lidar_publisher2'
CONFLICT_PROCS = ['livox_ros_driver2_node', 'fastlio_mapping', 'gravity_bridge_node']
SOFT_LIMIT = 300.0       # 建图超过 5 min 提醒收尾
STILL_AFTER_INIT = 3.0   # 收到第一帧里程计后再静止这么久，保证 raw.bag 开头有静止段（离线重建要靠它初始化）
STOP_TIMEOUT = 60.0      # 等 roslaunch 收尾的上限；FAST-LIO 退出时才写 scans.pcd
MIN_FREE_GB = 5.0
MIN_LIDAR_MSGS = 50      # 原始包里雷达帧少于这么多（5 s）就不建图
CLOCK_SYNC_TIMEOUT = 60.0  # 等 chrony 对上（s）；开机约 30 s 完成，和 fusion.sh 一致
CLOCK_MAX_OFFSET = 0.05    # 对上之后允许的剩余偏差（s），超过说明还在大幅修正

LAUNCH = '''<launch>
  <!-- build_map 生成。和 start_mid360s_fastlio.sh 的联合 launch 相同，只多两处：
       algo=fastlio 时覆盖 mid360s.yaml 的 pcd_save（只对本次生效）；离线回放时不带驱动。
       注释里不能出现两个连写的减号，否则 roslaunch 解析失败 -->
%(head)s
  <include file="$(find fast_lio)/launch/mapping_mid360s.launch">
    <arg name="rviz" value="false"/>
  </include>
%(pcd)s%(extra)s
  <include file="$(find fastlio_gravity_bridge)/launch/gravity_bridge.launch">
    <arg name="send_to_mavros" value="false"/>
  </include>
</launch>
'''
PCD_SAVE = '''  <param name="/pcd_save/pcd_save_en" type="bool" value="true"/>
  <param name="/pcd_save/interval" type="int" value="-1"/>
'''
DRIVER = '''  <include file="$(find livox_ros_driver2)/launch_ROS1/msg_MID360s.launch">
    <arg name="rviz_enable" value="false"/>
    <arg name="rosbag_enable" value="false"/>
  </include>'''
# 离线回放不用仿真时间：FAST-LIO 只认消息时间戳，但主循环用 ros::Rate，仿真时间下回放一停 /clock 不走，
# SIGINT 后出不了循环，15 s 后被 roslaunch 改发 SIGTERM，地图就存不下来（10-18 实测）

LOG = {'f': None}
# stage：record（录包 / FAST-LIO 回放，第一次信号开始收尾）→ slam（离线 Voxel-SLAM，SIGINT/SIGTERM 放弃建图）
STOP = {'sig': None, 'stage': 'record', 'abort': False}


def say(msg):
    line = '[%s] %s' % (time.strftime('%H:%M:%S'), msg)
    try:                                   # ssh 断了以后 stdout 写不出去，不能因此中断收尾
        print(line, flush=True)
    except OSError:
        pass
    if LOG['f']:
        LOG['f'].write(line + '\n')
        LOG['f'].flush()


def ensure_ros_env(entry):
    """没套过环境就套一层 bash 重新执行入口脚本：source ROS 和本工作空间，按 start_mid360s_fastlio.sh 设 ROS_IP，本机 RViz 能看"""
    if os.environ.get('BUILD_MAP_ENV') == '1':
        return
    for f in (ROS_SETUP, WS_SETUP):
        if not os.path.isfile(f):
            print('[ERROR] 找不到 %s（工作空间没编译？）' % f)
            sys.exit(3)
    ip = BOARD_IP if BOARD_IP in subprocess.run(['ip', '-4', 'addr'], stdout=subprocess.PIPE,
                                                universal_newlines=True).stdout else '127.0.0.1'
    # source 前先清空位置参数，不然 --help 之类会传给 catkin 的 setup 脚本
    cmd = ('ARGS=("$@"); set --; source %s && source %s && export BUILD_MAP_ENV=1 ROS_MASTER_URI=http://%s:%d ROS_IP=%s '
           '&& unset ROS_HOSTNAME && exec python3 "$0" "${ARGS[@]}"' % (ROS_SETUP, WS_SETUP, ip, MASTER_PORT, ip))
    os.execvp('bash', ['bash', '-c', cmd, entry] + sys.argv[1:])


def write_meta(d, meta):
    with open(os.path.join(d, 'meta.json'), 'w') as f:
        json.dump(meta, f, ensure_ascii=False, indent=1)


# ------------------------------------------------------- FAST-LIO 后处理 ----
def post_fastlio(d, voxel, run=None):
    """scans.pcd + 包里的 world←camera_init → map_world.pcd / preview.png / meta.json；返回 meta"""
    t0 = time.time()
    meta_path = os.path.join(d, 'meta.json')
    meta = {}
    if os.path.isfile(meta_path):
        with open(meta_path) as f:
            meta = json.load(f)
    if run:
        meta['run'] = run
    meta['algo'] = 'fastlio'
    bag = next((os.path.join(d, b) for b in ('raw.bag', 'odom.bag') if os.path.isfile(os.path.join(d, b))), None)
    tf, grav, odom = mapio.read_bag(bag) if bag else (None, None, np.zeros((0, 8)))
    if bag is None:
        say('[WARN] 目录里没有 raw.bag / odom.bag，无法摆正')
    Rg = mapio.R_from_gravity(grav) if grav else None
    if tf:
        R, t, src = mapio.quat_R(*tf[0]), np.array(tf[1]), 'tf_static world->camera_init'
    elif Rg is not None:
        R, t, src = Rg, np.zeros(3), 'gravity_init（tf_static 缺失，按最小旋转推算）'
        say('[WARN] 包里没有 world→camera_init，改用 /fastlio/gravity_init 推算')
    else:
        R, t, src = np.eye(3), np.zeros(3), 'none（没摆正）'
        say('[WARN] 包里既没有 tf_static 也没有 gravity_init，map_world.pcd 没有摆正')
    meta['world_from_camera_init'] = dict(q_xyzw=mapio.R_quat(R), t=t.tolist(), source=src)
    if grav:
        meta['gravity_init'] = list(grav)
        dR = Rg.T @ R
        meta['tf_vs_gravity_deg'] = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(dR) - 1) / 2))))

    p, inten = mapio.read_pcd(os.path.join(d, 'scans.pcd'))
    pw = (p @ R.T.astype(np.float32)) + t.astype(np.float32)
    q, qi = mapio.voxel_down(pw, inten, voxel)
    mapio.write_pcd(os.path.join(d, 'map_world.pcd'), q, qi)
    fl = mapio.floor_fit(q)
    meta.update(voxel_m=voxel, points_raw=int(len(p)), points_world=int(len(q)),
                extent_world=dict(min=q.min(axis=0).tolist(), max=q.max(axis=0).tolist()), floor=fl)

    traj = np.zeros((0, 3))
    if len(odom):
        traj = odom[:, 1:4] @ R.T + t
        step = np.linalg.norm(np.diff(traj, axis=0), axis=1)

        def yaw(row):  # body 的 x 轴投到水平面上的朝向
            ax = R @ mapio.quat_R(*row[4:8])[:, 0]
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
    mapio.preview(os.path.join(d, 'preview.png'), os.path.basename(os.path.normpath(d)), q, traj, fl, text)
    write_meta(d, meta)
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


# -------------------------------------------------------- Voxel-SLAM ----
def slam(out, bag, a, run_info):
    """离线跑 Voxel-SLAM，再拼图；返回退出码。参数快照 voxelslam.yaml 就是本次实际加载的那份"""
    STOP['stage'] = 'slam'
    cfg = os.path.join(out, 'voxelslam.yaml')
    shutil.copy(VS_CONFIG, cfg)
    work = os.path.join(out, 'voxelslam_tmp')
    meta = dict(run=run_info, algo='voxelslam', source_bag=os.path.abspath(bag))
    t0 = time.time()
    try:
        info = voxelslam.run(bag, work, cfg, os.path.join(out, 'voxelslam.log'), rate=a.rate, say=say,
                             abort=lambda: STOP['abort'])
    except voxelslam.Aborted:
        shutil.rmtree(work, ignore_errors=True)
        say('已放弃离线建图。原始包在 %s，之后可以：rosrun drone_mapping build_map --bag %s' % (bag, bag))
        write_meta(out, dict(meta, error='aborted'))
        return 130
    except Exception as e:  # noqa: BLE001
        say('[ERROR] Voxel-SLAM 没跑完：%s' % e)
        say('原始包在 %s，中间文件留在 %s。可以重试 rosrun drone_mapping build_map --bag %s，'
            '或者加 --algo fastlio 用旧做法' % (bag, work, bag))
        write_meta(out, dict(meta, error=str(e)))
        return 1
    shutil.copy(info['state'], os.path.join(out, 'voxelslam_state.txt'))
    info.update(state='voxelslam_state.txt', total_s=round(time.time() - t0, 1))
    meta['voxelslam'] = info
    try:
        assemble.build(out, bag, os.path.join(out, 'voxelslam_state.txt'), cfg, a.voxel, meta, say=say, carve_r=a.carve)
    except Exception as e:  # noqa: BLE001  位姿已经存下了，拼图可以 --post 重做
        say('[ERROR] 拼图失败：%s（位姿已存，可以 build_map --post %s 重做）' % (e, out))
        write_meta(out, dict(meta, error=str(e)))
        return 1
    if not a.keep_frames:
        shutil.rmtree(work, ignore_errors=True)
    return 0


def post(d, voxel, carve_r=assemble.CARVE_R):
    """--post：有 Voxel-SLAM 位姿就重新拼图，否则按 FAST-LIO 的 scans.pcd 重做"""
    meta = {}
    if os.path.isfile(os.path.join(d, 'meta.json')):
        with open(os.path.join(d, 'meta.json')) as f:
            meta = json.load(f)
    state = os.path.join(d, 'voxelslam_state.txt')
    if not os.path.isfile(state):
        post_fastlio(d, voxel)
        return
    bag = os.path.join(d, 'raw.bag')
    if not os.path.isfile(bag):
        bag = meta.get('source_bag') or ''
    if not os.path.isfile(bag):
        raise RuntimeError('找不到原始包（%s 里没有 raw.bag，meta.json 里的 source_bag=%r）' % (d, bag))
    cfg = os.path.join(d, 'voxelslam.yaml')
    assemble.build(d, bag, state, cfg if os.path.isfile(cfg) else VS_CONFIG, voxel, meta, say=say, carve_r=carve_r)


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
            bad.append('/livox/lidar 是 %s，这里要 livox_ros_driver2/CustomMsg' % info['/livox/lidar'].msg_type)
        elif '/livox/lidar' in info and info['/livox/lidar'].message_count < MIN_LIDAR_MSGS:
            bad.append('%s 里只有 %d 帧雷达' % (a.bag, info['/livox/lidar'].message_count))
    else:
        addr = subprocess.run(['ip', '-4', 'addr'], stdout=subprocess.PIPE, universal_newlines=True).stdout
        if LIDAR_NET_IP not in addr:
            bad.append('板端没有雷达网口地址 %s（enP3p49s0 DOWN / 网线没插）' % LIDAR_NET_IP)
        elif subprocess.run(['ping', '-c', '1', '-W', '2', LIDAR_IP], stdout=subprocess.DEVNULL).returncode:
            bad.append('雷达 %s ping 不通（没上电？）' % LIDAR_IP)
    if not (a.bag and a.algo == 'voxelslam'):   # 离线 Voxel-SLAM 用自己的 master（127.0.0.1 空闲端口），不和在跑的链路冲突
        if master_up():
            bad.append('已经有 ROS master 在跑（fusion / start_mid360s_fastlio / roscore？），先停掉')
        for n in CONFLICT_PROCS:
            if subprocess.run(['pgrep', '-f', n], stdout=subprocess.DEVNULL).returncode == 0:
                bad.append('已有 %s 进程在跑' % n)
    if a.algo == 'voxelslam':
        if not os.path.isfile(VS_BIN):
            bad.append('没有 %s（工作空间没编译 voxel_slam？跑 scripts/build.sh）' % VS_BIN)
        if not os.path.isfile(VS_CONFIG):
            bad.append('没有 Voxel-SLAM 参数 %s' % VS_CONFIG)
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


def record(a, out, launch, run_info):
    """起 roslaunch（实机：驱动 + FAST-LIO + 重力桥；fastlio 离线：不带驱动并回放包）并录包，
    等 Ctrl+C / 回放结束 / 出错，再按顺序收尾：先停回放和录包，再停 roslaunch"""
    rl_log = open(os.path.join(out, 'roslaunch.log'), 'w')
    bag_log = open(os.path.join(out, 'rosbag.log'), 'w')
    rl = subprocess.Popen(['roslaunch', '--screen', '--required', launch], stdout=rl_log, stderr=subprocess.STDOUT,
                          stdin=subprocess.DEVNULL, start_new_session=True)
    rec = play = odo = None
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
            return False
        subprocess.run(['rosparam', 'dump', os.path.join(out, 'params.yaml')], timeout=15)
        if a.algo == 'fastlio':
            got = subprocess.run(['rosparam', 'get', '/pcd_save/pcd_save_en'], stdout=subprocess.PIPE,
                                 universal_newlines=True, timeout=10).stdout.strip()
            if got != 'true':
                say('[ERROR] 运行时 /pcd_save/pcd_save_en = %r，地图不会保存，停止' % got)
                return False
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
                    say('[WARN] 已经超过 %.0f min，建议回到起点收尾（%s）' % (SOFT_LIMIT / 60,
                        'scans.pcd 是一次写出，点越多停得越慢' if a.algo == 'fastlio' else '离线建图耗时约等于录制时长'))
            time.sleep(0.5)
        return True
    finally:
        say('收尾：先停录包，再停 FAST-LIO%s' % ('（它退出时写 scans.pcd）' if a.algo == 'fastlio' else ''))
        stop_proc(play, 'rosbag play', 10)
        stop_proc(rec, 'rosbag record', 20)
        if odo is not None:  # 先退掉监视节点，免得 master 停了以后 rospy 一直报连不上
            import rospy
            odo.sub.unregister()
            rospy.signal_shutdown('build_map 收尾')
        rc = stop_proc(rl, 'roslaunch', STOP_TIMEOUT)
        run_info.update(end=time.strftime('%F %T'), roslaunch_rc=rc,
                        stop=('signal %d' % STOP['sig']) if STOP['sig'] else 'auto')
        for fn in ('raw.bag.active', 'odom.bag.active'):
            if os.path.exists(os.path.join(out, fn)):
                say('[WARN] %s 没收尾，可以 rosbag reindex 试试' % fn)
        rl_log.close()
        bag_log.close()


def run(a):
    stamp = time.strftime('%Y%m%d_%H%M%S')
    name = a.name or ('bag_' + os.path.splitext(os.path.basename(a.bag))[0] if a.bag else '')
    out = os.path.join(a.out_root, stamp + ('_' + name if name else ''))
    os.makedirs(out)
    LOG['f'] = open(os.path.join(out, 'build_map.log'), 'a')
    say('输出目录 %s（%s）' % (out, 'Voxel-SLAM' if a.algo == 'voxelslam' else 'FAST-LIO'))
    run_info = dict(mode='bag' if a.bag else 'live', algo=a.algo, source_bag=os.path.abspath(a.bag) if a.bag else None,
                    rate=a.rate if a.bag or a.algo == 'voxelslam' else None, start=time.strftime('%F %T'), git=git_rev(),
                    host=socket.gethostname(), chrony=clock_status(), fastlio_set=dict(a.sets))
    if a.bag and a.algo == 'voxelslam':
        code = slam(out, a.bag, a, run_info)
        os.sync()
        say('已 sync。%s' % ('完成：' + out if code == 0 else '没完成，看 ' + out))
        return code

    launch = os.path.join(out, 'build_map.launch')
    with open(launch, 'w') as f:
        f.write(LAUNCH % {'head': '' if a.bag else DRIVER, 'pcd': PCD_SAVE if a.algo == 'fastlio' else '',
                          'extra': ''.join('  <param name="/%s" value="%s"/>\n' % kv for kv in a.sets)})
    if a.algo == 'fastlio':
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

    t_start = time.time()
    try:
        recorded = record(a, out, launch, run_info)
    except Exception as e:  # noqa: BLE001  录包阶段意外出错：进程已在 record 里收尾，已经录下的照样处理
        say('[ERROR] 录包阶段出错：%s' % e)
        recorded = False
    code = 1
    if a.algo == 'fastlio':
        src = os.path.join(PCD_DIR, 'scans.pcd')
        if os.path.isfile(src) and os.path.getmtime(src) >= t_start:
            shutil.move(src, os.path.join(out, 'scans.pcd'))
            say('scans.pcd %.0f MB 已挪进输出目录' % (os.path.getsize(os.path.join(out, 'scans.pcd')) / 1e6))
            try:
                post_fastlio(out, a.voxel, run_info)
                code = 0
            except Exception as e:  # noqa: BLE001  后处理失败不影响已经存下的原始地图，可以 --post 重做
                say('[ERROR] 后处理失败：%s（scans.pcd 和包都在，可以 build_map --post %s 重做）' % (e, out))
        else:
            say('[ERROR] 没有拿到本次的 scans.pcd（FAST-LIO 没正常退出，或者没收到点）')
            write_meta(out, {'run': run_info, 'algo': 'fastlio'})
    else:
        raw = os.path.join(out, 'raw.bag')
        n = 0
        if os.path.isfile(raw):
            import rosbag
            try:
                with rosbag.Bag(raw) as b:
                    n = b.get_message_count('/livox/lidar')
            except Exception as e:  # noqa: BLE001
                say('[ERROR] raw.bag 读不了：%s' % e)
        if n < MIN_LIDAR_MSGS:
            say('[ERROR] raw.bag %s，不建图' % ('只有 %d 帧雷达' % n if os.path.isfile(raw) else '没录下来（或者没收尾）'))
            write_meta(out, {'run': run_info, 'algo': 'voxelslam'})
        else:
            if not recorded:
                say('[WARN] 录包阶段不正常，仍然用已录下的 raw.bag（%d 帧雷达）建图' % n)
            os.sync()   # 先把原始包落盘，离线建图要好几分钟，期间掉电也不丢包
            code = slam(out, raw, a, run_info)
    os.sync()
    say('已 sync。%s' % ('完成：' + out if code == 0 else '没完成，看 ' + out))
    return code


def main(entry):
    """entry：入口脚本的路径（ensure_ros_env 用它重新执行）"""
    ensure_ros_env(entry)
    p = argparse.ArgumentParser(prog='build_map', description=__doc__.splitlines()[0])
    g = p.add_mutually_exclusive_group()
    g.add_argument('--bag', help='用录好的原始包离线重建')
    g.add_argument('--post', metavar='DIR', help='只重做 DIR 的拼图 / 后处理')
    p.add_argument('--algo', choices=['voxelslam', 'fastlio'], default='voxelslam', help='建图算法，默认 voxelslam')
    p.add_argument('--rate', type=float, default=1.0, help='离线回放倍速，默认 1')
    p.add_argument('--name', default='', help='输出目录名后缀')
    p.add_argument('--voxel', type=float, default=0.05)
    p.add_argument('--out-root', default=os.path.expanduser('~/test_data/mapping'))
    p.add_argument('--keep-frames', action='store_true', help='保留 Voxel-SLAM 的逐帧点云')
    p.add_argument('--carve', type=float, default=assemble.CARVE_R, metavar='R',
                   help='去人：去掉设备走过空间里（水平 R m 以内）只短暂出现的点，默认 %(default)s，0 关闭')
    p.add_argument('--allow-unsynced-clock', action='store_true', help='实机：板端时间没对上也照样建图')
    p.add_argument('--set', action='append', default=[], metavar='NAME=VALUE', help='只对本次覆盖实时 FAST-LIO 参数，可重复')
    a = p.parse_args()
    a.sets = []
    for kv in a.set:
        k, sep, v = kv.partition('=')
        if not sep or not k.strip('/ ') or not v.strip():
            print('--set 要写成 NAME=VALUE：%r' % kv)
            return 2
        a.sets.append((k.strip('/ '), v.strip()))
    if a.sets and a.bag and a.algo == 'voxelslam':
        print('--set 只改 FAST-LIO 参数，离线 Voxel-SLAM 用不到')
        return 2
    if a.post:
        LOG['f'] = open(os.path.join(a.post, 'build_map.log'), 'a')
        say('重做后处理 %s' % a.post)
        try:
            post(a.post, a.voxel, a.carve)
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
        print('[ERROR] 已有 build_map 在跑（%s 被占用）' % LOCK_FILE)
        return 3
    if not a.bag and not wait_clock(a.allow_unsynced_clock):
        return 3
    bad = preflight(a)
    if bad:
        for b in bad:
            print('[ERROR] ' + b)
        return 3
    if a.bag and a.algo == 'voxelslam':
        STOP['stage'] = 'slam'

    def on_sig(sig, _frm):
        nm = signal.Signals(sig).name
        if STOP['stage'] == 'slam':   # 离线 Voxel-SLAM：SIGINT/SIGTERM 放弃；SIGHUP（ssh 断开）照样跑完
            if sig != signal.SIGHUP and not STOP['abort']:
                STOP['abort'] = True
                say('收到 %s，放弃离线建图（原始包保留），正在停 Voxel-SLAM……' % nm)
            return
        if STOP['sig'] is None:
            STOP['sig'] = sig
            if a.algo == 'fastlio':
                say('收到 %s，开始收尾，请等它跑完（存图 + 后处理约 10–60 s）' % nm)
            else:
                say('收到 %s，开始收尾：先停录包和 FAST-LIO，再离线跑 Voxel-SLAM（约等于录制时长）；'
                    '离线建图时再按 Ctrl+C 就放弃建图、只留 raw.bag' % nm)
        else:
            say('收尾进行中，不要再按')
    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):  # ssh 断开（SIGHUP）也按正常收尾存图
        signal.signal(s, on_sig)
    return run(a)
