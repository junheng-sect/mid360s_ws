"""离线跑一遍 Voxel-SLAM：独立 ROS master（127.0.0.1 上找一个空闲端口）→ 起 voxelslam 节点 → rosbag play 原始包
→ 等它把帧处理完 → 置 /finish 触发全局 BA → 取位姿文件 alidarState.txt。

Voxel-SLAM 的几个硬性行为（hku-mars 70fc8a2，src/voxel_slam/src/voxelslam.cpp）：
- 只有 is_save_map=1 才写位姿文件 <save_path><bagname>/alidarState.txt，同时把每帧点云写成 <序号>.pcd；
  <save_path><bagname>/ 事先存在就直接 exit(0)，所以每次给一个新目录。
- 主循环每帧读一次全局参数 /finish，为 true 就跳出去做全局 BA、存位姿；回放还没处理完就置 true，后面的帧就丢了。
  这里按逐帧 pcd 的个数判断：回放结束后连续 SETTLE 秒不再增加，才算处理完。
- 只按消息顺序处理、不看墙钟，订阅队列雷达 1000 帧、IMU 80000 条：只要不丢消息，回放倍速不改变结果
  （10-19 实测 0.5 / 1 倍速三个包逐位相同）。板端 1 倍速约占 1 个核，跟得上实时。
"""
import os
import signal
import socket
import subprocess
import time

PLAY_BIN = '/opt/ros/noetic/lib/rosbag/play'
TOPICS = ['/livox/lidar', '/livox/imu']
NODE = '/voxelslam'
SETTLE = 10.0           # 回放结束后逐帧 pcd 连续这么久不增加，算处理完
CATCHUP_MAX = 600.0     # 回放结束后最多再等这么久
GBA_MAX = 900.0         # 置 /finish 后等位姿文件的上限（房间 4 s）
MIN_FRAME_RATIO = 0.9   # 存下的帧数 / 包里的雷达帧数低于它当作丢帧（正常只少开头初始化用掉的 10–40 帧）


class Aborted(Exception):
    """用户按 Ctrl+C 放弃"""


def free_port():
    s = socket.socket()
    s.bind(('127.0.0.1', 0))
    port = s.getsockname()[1]
    s.close()
    return port


def count_frames(d):
    try:
        return sum(1 for e in os.scandir(d) if e.name.endswith('.pcd'))
    except FileNotFoundError:
        return 0


def _hint(log_path):
    """节点没存出位姿就退出时，按日志末尾给个能看懂的原因"""
    try:
        with open(log_path, 'rb') as f:
            f.seek(0, os.SEEK_END)
            f.seek(max(0, f.tell() - 20000))
            tail = f.read().decode('utf-8', 'replace')
    except OSError:
        return ''
    if 'Too Less Voxel' in tail:   # 上游 voxel_map.hpp 里的硬退出：全局 BA 的平面体素比线程数还少，exit(0)
        return '：全局 BA 报 Too Less Voxel 后退出，设备几乎没动（只有一个关键帧），静止数据建不了图，建图时要走动起来'
    return ''


def _stop(p, timeout=10.0):
    """整个进程组先 SIGINT，超时 SIGKILL（roslaunch 的节点和它在同一个进程组里）"""
    if p is None or p.poll() is not None:
        return
    try:
        os.killpg(p.pid, signal.SIGINT)
        p.wait(timeout)
    except (OSError, subprocess.TimeoutExpired):
        try:
            os.killpg(p.pid, signal.SIGKILL)
        except OSError:
            pass
        p.wait()


def run(bag, work, config, log_path, rate=1.0, say=print, abort=lambda: False):
    """对 bag 跑一遍 Voxel-SLAM，逐帧点云和位姿写在 work/vs/（work 必须还不存在）。
    返回 dict（state=位姿文件路径、frames、lidar_msgs、各段耗时）；出错抛 RuntimeError，abort() 为真时抛 Aborted。"""
    import rosbag
    with rosbag.Bag(bag) as b:
        n_lidar = b.get_message_count('/livox/lidar')
        dur = b.get_end_time() - b.get_start_time()
    os.makedirs(work)
    vs_dir = os.path.join(work, 'vs')
    state = os.path.join(vs_dir, 'alidarState.txt')
    port = free_port()
    env = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:%d' % port, ROS_IP='127.0.0.1')
    env.pop('ROS_HOSTNAME', None)
    log = open(log_path, 'a')
    procs = []

    def start(cmd):
        log.write('\n===== %s %s\n' % (time.strftime('%T'), ' '.join(cmd)))
        log.flush()
        p = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                             start_new_session=True)
        procs.append(p)
        return p

    def ros(*cmd, timeout=10):
        return subprocess.run(list(cmd), env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                              universal_newlines=True, timeout=timeout)

    def check(launch):
        if abort():
            raise Aborted()
        if launch.poll() is not None:
            raise RuntimeError('voxelslam 节点退出了（roslaunch %s），看 %s' % (launch.returncode, log_path))

    info = dict(lidar_msgs=n_lidar, bag_s=round(dur, 1), rate=rate, port=port)
    try:
        start(['roscore', '-p', str(port)])
        t_end = time.time() + 30
        while ros('rosparam', 'list').returncode != 0:
            if abort():
                raise Aborted()
            if time.time() > t_end:
                raise RuntimeError('独立 roscore（端口 %d）30 s 没起来，看 %s' % (port, log_path))
            time.sleep(0.5)
        launch = start(['roslaunch', 'drone_mapping', 'voxelslam.launch', 'config:=' + config,
                        'save_path:=' + work + '/', 'bagname:=vs'])
        t_end = time.time() + 30
        while NODE not in ros('rosnode', 'list').stdout.split():
            check(launch)
            if time.time() > t_end:
                raise RuntimeError('voxelslam 节点 30 s 没起来，看 %s' % log_path)
            time.sleep(0.5)
        time.sleep(1.0)   # 等订阅连上

        say('Voxel-SLAM：回放 %s（%.0f s、%d 帧雷达，%.1f 倍速）' % (os.path.basename(bag), dur, n_lidar, rate))
        t0 = time.time()
        play = start([PLAY_BIN, '-q', '-d', '1', '-r', str(rate), bag, '--topics'] + TOPICS)
        t_last = t0
        while play.poll() is None:
            check(launch)
            if time.time() - t_last >= 15:
                t_last = time.time()
                say('Voxel-SLAM：已处理 %d / %d 帧' % (count_frames(vs_dir), n_lidar))
            time.sleep(0.5)
        if play.returncode != 0:
            raise RuntimeError('rosbag play 出错（%s），看 %s' % (play.returncode, log_path))
        t1 = time.time()
        last, t_still = -1, time.time()
        while time.time() - t_still < SETTLE:
            check(launch)
            n = count_frames(vs_dir)
            if n != last:
                last, t_still = n, time.time()
            if time.time() - t1 > CATCHUP_MAX:
                raise RuntimeError('回放结束 %.0f s 了还在处理（%d 帧），看 %s' % (CATCHUP_MAX, n, log_path))
            time.sleep(0.5)
        t2 = time.time()
        catchup = max(0.0, t2 - t1 - SETTLE)
        info.update(frames=last, play_s=round(t1 - t0, 1), catchup_s=round(catchup, 1))
        say('Voxel-SLAM：处理完 %d / %d 帧（回放 %.0f s，之后又追了 %.0f s），开始全局 BA'
            % (last, n_lidar, t1 - t0, catchup))

        ros('rosparam', 'set', '/finish', 'true')
        size, t_still = -1, time.time()
        while True:   # 位姿文件出现且 1 s 不再变大才算写完；节点存完自己退出也行，出循环后再核对文件
            if abort():
                raise Aborted()
            s = os.path.getsize(state) if os.path.isfile(state) else -1
            if s != size:
                size, t_still = s, time.time()
            elif s > 0 and time.time() - t_still >= 1.0:
                break
            if launch.poll() is not None:
                break
            if time.time() - t2 > GBA_MAX:
                raise RuntimeError('置 /finish 后 %.0f s 没存出位姿，看 %s' % (GBA_MAX, log_path))
            time.sleep(0.2)
        info['gba_s'] = round(time.time() - t2 - 1.0, 1)
        if not os.path.isfile(state):
            raise RuntimeError('voxelslam 退出了（roslaunch %s），没有位姿文件%s，看 %s'
                               % (launch.returncode, _hint(log_path), log_path))
    finally:
        for p in reversed(procs):
            _stop(p)
        log.close()

    with open(state) as f:
        n_pose = sum(1 for line in f if line.strip())
    info.update(state=state, poses=n_pose)
    if n_pose == 0:
        raise RuntimeError('位姿文件是空的：%s' % state)
    if n_pose < MIN_FRAME_RATIO * n_lidar:
        raise RuntimeError('只存下 %d / %d 帧的位姿，可能丢了帧（回放太快或 CPU 被占满），看 %s' % (n_pose, n_lidar, log_path))
    return info
