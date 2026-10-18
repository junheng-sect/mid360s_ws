#!/usr/bin/env python3
"""小车定位：检查 M65 车（spirit）在它自己地图里的定位，不对就由板端激光匹配算出位姿，调 /set_relocation 重定位。

背景（2026-10-18 实测）：
- 车开机直接读回 /home/spirit/.ros/carto/carto_config.yaml 里关机前的位姿，不做搜索。关机时挪过车，
  位姿就一直是错的，App 的 locationstatus 照样报 10。
- 看得出错的是 cartographer 的 /local_global_score[1]（激光对全局地图的得分，错 0.29 / 对 0.86）
  和 /submap_in_map_score（错 1 / 对 0）。
- /set_relocation 把位姿原样转发到 /initialpose，cartographer 在它周围 ±1.5 m、±90° 内再搜一次
  （rtcsm）。App 里拖地图重定位走的也是它，只是初值靠人眼。
- 板端不需要初值：拿 /scan 在整张 /map 上暴力匹配（似然场），约 4–5 s，和车的定位差 < 1 cm。

流程：连 rosbridge → 等车静止 → 检查（板端激光在 /tracked_pose 上的得分、局部再匹配的偏移、
车自报的两个得分）→ 合格且没加 --force 就结束；否则全局匹配，得分低或有相近的第二个位置就不发
→ /set_relocation → 等位姿跳到位后再检查一遍。

只经 rosbridge（roslibpy）和车通信，不依赖板端 ROS master，fusion 没起也能用。整个过程不让车动，
不调 /move_calibration type=3（那个会原地转圈找定位）；万一重定位后车动了，发 /navi_stop 5。

用法: car_reloc.py [--host 192.168.31.231] [--port 9090] [--check | --force] [--wait-online S]
                   [--still-timeout S] [--save DIR]
  --check          只检查，不合格也不发
  --force          检查合格也重发一次（发的是板端全局匹配的结果）
  --wait-online    车不在线时最多等 S 秒（刚开机时用），默认不等
  --still-timeout  等车静止的上限（秒），默认 30
  --save           把地图、激光、各阶段位姿存成 DIR/car_reloc_<时间>.npz/.json，方便画图
退出码: 0 定位可信（本来就对，或重定位后检查通过）
        1 重定位后检查不通过，或车端拒绝（地图没加载好）
        3 连不上车 / 等不到数据 / 缺依赖      4 车一直在动，没检查或没发
        5 板端匹配不可信，没发                6 --check：定位不合格
        130 / 143 被 Ctrl+C / SIGTERM 停下
"""

import argparse
import json
import math
import os
import signal
import socket
import sys
import threading
import time

import numpy as np

try:
    import cv2
    import roslibpy
except ImportError as e:
    print("缺依赖：%s（roslibpy 用 pip --user 装，cv2 用 apt 的 python3-opencv）" % e)
    sys.exit(3)

SIG = 0.05                      # 似然场 σ (m)，和地图 0.05 m 栅格同量级
SCORE_OK = 0.6                  # 板端激光在位姿上的平均似然：实测对 0.87、错 0.13
LGS_OK = 0.6                    # 车自报 /local_global_score[1]：实测对 0.85–0.88、错 0.29
SHIFT_CM, SHIFT_DEG = 3.0, 1.0  # /tracked_pose 和板端局部再匹配之间允许的偏差
RATIO_MAX = 0.8                 # 全局匹配次优峰 / 最优峰，超过说明有两个相像的位置
STILL_V, STILL_W = 0.01, 0.02   # 静止判据 |vx| (m/s)、|wz| (rad/s)
MAX_CANDS = 6000                # 全局粗搜的候选位置上限，大地图自动放大步长
TOPICS = [('/tracked_pose', 'geometry_msgs/PoseStamped', 0),
          ('/raw_odom', 'nav_msgs/Odometry', 50),
          ('/scan', 'sensor_msgs/LaserScan', 0),
          ('/map', 'nav_msgs/OccupancyGrid', 0),
          ('/local_global_score', 'std_msgs/Float64MultiArray', 100),  # 原始约 860 Hz，必须节流
          ('/submap_in_map_score', 'std_msgs/Int16', 100)]


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def fmt(p):
    return '(%.3f, %.3f, %.2f°)' % (p[0], p[1], math.degrees(p[2]))


class Car:
    """rosbridge 连接和最新数据"""

    def __init__(self, host, port):
        self.host, self.port = host, port
        self.lock = threading.Lock()
        self.st, self.scans = {}, []
        self.ros = roslibpy.Ros(host=host, port=port)
        self.pub_stop = None

    def connect(self, timeout):
        try:
            self.ros.run(timeout=min(8.0, timeout))
        except Exception:  # noqa: BLE001  rosbridge 端口开了但还没就绪，客户端会自己重连
            pass
        t_end = time.time() + timeout
        while not self.ros.is_connected:
            if time.time() > t_end:
                return False
            time.sleep(0.5)
        for name, typ, thr in TOPICS:
            roslibpy.Topic(self.ros, name, typ, throttle_rate=thr, queue_length=1).subscribe(self._cb(name))
        return True

    def _cb(self, name):
        def cb(m):
            with self.lock:
                if name == '/scan':
                    self.scans.append((time.time(), m))
                    del self.scans[:-5]
                else:
                    self.st[name] = (time.time(), m)
        return cb

    def missing(self):
        with self.lock:
            return [n for n in ('/tracked_pose', '/raw_odom', '/map') if n not in self.st] + ([] if self.scans else ['/scan'])

    def pose(self):
        with self.lock:
            m = self.st['/tracked_pose'][1]['pose']
        return m['position']['x'], m['position']['y'], 2 * math.atan2(m['orientation']['z'], m['orientation']['w'])

    def moving(self):
        with self.lock:
            tw = self.st['/raw_odom'][1]['twist']['twist']
        return abs(tw['linear']['x']) > STILL_V or abs(tw['angular']['z']) > STILL_W

    def recv_time(self, name):
        with self.lock:
            return self.st[name][0] if name in self.st else 0.0

    def scores(self, since=0.0):
        """车自报的 (global score, submap_in_map_score)；since 之后没收到的为 None"""
        with self.lock:
            lgs = self.st.get('/local_global_score')
            sub = self.st.get('/submap_in_map_score')
        g = lgs[1]['data'][1] if lgs and lgs[0] > since and len(lgs[1]['data']) > 1 else None
        return g, (sub[1]['data'] if sub and sub[0] > since else None)

    def grid(self):
        with self.lock:
            return self.st['/map'][1]

    def wait_still(self, hold, timeout):
        t_end, since = time.time() + timeout, None
        while time.time() < t_end:
            if self.moving():
                since = None
            else:
                since = since or time.time()
                if time.time() - since >= hold:
                    return True
            time.sleep(0.05)
        return False

    def fresh_points(self, n=3, timeout=3.0):
        """取此刻之后的 n 帧激光，拼成车体系点（激光装在车体中心，只差 z）"""
        t0, t_end = time.time(), time.time() + timeout
        while True:
            with self.lock:
                fr = [m for t, m in self.scans if t > t0]
            if len(fr) >= n or time.time() > t_end:
                break
            time.sleep(0.05)
        if not fr:
            return None
        xs, ys = [], []
        for m in fr[-n:]:
            r = np.array([np.nan if v is None else v for v in m['ranges']], dtype=float)
            a = m['angle_min'] + m['angle_increment'] * np.arange(len(r))
            with np.errstate(invalid='ignore'):
                ok = np.isfinite(r) & (r > m['range_min']) & (r < min(m['range_max'], 30.0))
            xs.append(r[ok] * np.cos(a[ok]))
            ys.append(r[ok] * np.sin(a[ok]))
        return np.concatenate(xs), np.concatenate(ys)

    def set_relocation(self, p):
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25          # 和 10.16 手动调用一致：xy 0.25、yaw 0.0685
        cov[35] = 0.06853891945200942
        req = {'point': {'header': {'frame_id': 'map'},
                         'pose': {'pose': {'position': {'x': p[0], 'y': p[1], 'z': 0.0},
                                           'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(p[2] / 2),
                                                           'w': math.cos(p[2] / 2)}},
                                  'covariance': cov}}}
        srv = roslibpy.Service(self.ros, '/set_relocation', 'map_msgs/SetRelocation')
        return srv.call(roslibpy.ServiceRequest(req), timeout=8)['result']

    def stop(self):
        if self.pub_stop is None:
            self.pub_stop = roslibpy.Topic(self.ros, '/navi_stop', 'std_msgs/Int16')
            self.pub_stop.advertise()
            time.sleep(0.3)
        for _ in range(3):
            self.pub_stop.publish(roslibpy.Message({'data': 5}))
            time.sleep(0.1)

    def close(self):
        try:
            self.ros.terminate()
        except Exception:  # noqa: BLE001
            pass


class Matcher:
    """栅格地图的似然场打分"""

    def __init__(self, m):
        info = m['info']
        self.W, self.H, self.res = info['width'], info['height'], info['resolution']
        o = info['origin']
        self.ox, self.oy = o['position']['x'], o['position']['y']
        if abs(o['orientation']['z']) > 1e-3:
            print('[WARN] 地图原点带旋转（qz=%.4f），这里按无旋转处理' % o['orientation']['z'])
        self.g = np.array(m['data'], dtype=np.int16).reshape(self.H, self.W)
        self.dist = cv2.distanceTransform((self.g <= 50).astype(np.uint8), cv2.DIST_L2, 5) * self.res
        self.lik = self._lik(SIG)

    def _lik(self, sig):
        return np.exp(-self.dist ** 2 / (2 * sig ** 2)).astype(np.float32)

    def score(self, xs, ys, th, px, py, lik=None):
        """候选位置 xs, ys（同一朝向 th）上激光点的平均似然，双线性插值"""
        c, s = math.cos(th), math.sin(th)
        rx, ry = c * px - s * py, s * px + c * py
        mx = ((xs[:, None] + rx[None, :] - self.ox) / self.res - 0.5).astype(np.float32)
        my = ((ys[:, None] + ry[None, :] - self.oy) / self.res - 0.5).astype(np.float32)
        v = cv2.remap(self.lik if lik is None else lik, mx, my, cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return v.mean(axis=1)

    def score_at(self, p, px, py):
        return float(self.score(np.array([p[0]]), np.array([p[1]]), p[2], px, py)[0])

    def refine(self, p, px, py, stages=((0.1, 3.0), (0.02, 0.5)), n=21):
        x, y, th = p
        best = (self.score_at(p, px, py), p)
        for lin, ang in stages:
            gx, gy = np.meshgrid(np.linspace(x - lin, x + lin, n), np.linspace(y - lin, y + lin, n))
            gx, gy = gx.ravel(), gy.ravel()
            for t in np.linspace(th - math.radians(ang), th + math.radians(ang), n):
                sc = self.score(gx, gy, t, px, py)
                q = int(np.argmax(sc))
                if sc[q] > best[0]:
                    best = (float(sc[q]), (float(gx[q]), float(gy[q]), float(t)))
            x, y, th = best[1]
        return best

    def global_match(self, px, py):
        """无初值全局匹配：离墙 ≥ 0.2 m 的空闲格做候选、每 2° 一档粗搜，再两级细化"""
        k = 2
        while True:
            ii, jj = np.nonzero((self.g[::k, ::k] == 0) & (self.dist[::k, ::k] >= 0.2))
            if len(ii) <= MAX_CANDS:
                break
            k += 1
        step = k * self.res
        lik_c = self.lik if step <= 0.1 + 1e-9 else self._lik(step / 2)  # 步长大了把似然场放宽，免得漏掉峰
        cx = self.ox + (jj * k + 0.5) * self.res
        cy = self.oy + (ii * k + 0.5) * self.res
        sub = slice(None, None, max(1, len(px) // 300))
        peaks = []
        for th in np.radians(np.arange(-180, 180, 2.0)):
            sc = self.score(cx, cy, th, px[sub], py[sub], lik_c)
            q = int(np.argmax(sc))
            peaks.append((float(sc[q]), float(cx[q]), float(cy[q]), float(th)))
        peaks.sort(reverse=True)
        b = peaks[0]
        alt = next((p for p in peaks if math.hypot(p[1] - b[1], p[2] - b[2]) > 0.5
                    or abs(wrap(p[3] - b[3])) > math.radians(15)), (0.0, 0.0, 0.0, 0.0))
        stages = ((max(0.1, step), 3.0), (0.02, 0.5)) if step > 0.1 + 1e-9 else ((0.1, 3.0), (0.02, 0.5))
        s, p = self.refine((b[1], b[2], b[3]), px, py, stages)
        return dict(pose=p, score=s, ratio=alt[0] / b[0] if b[0] > 0 else 1.0, alt=alt, cands=len(cx), step=step)


def check(M, car, px, py):
    """车当前定位是否可信：板端打分、局部再匹配偏移、车自报得分"""
    tp = car.pose()
    s_tp = M.score_at(tp, px, py)
    s_best, best = M.refine(tp, px, py)
    d_cm = 100 * math.hypot(best[0] - tp[0], best[1] - tp[1])
    d_deg = math.degrees(wrap(best[2] - tp[2]))
    g, sub = car.scores()
    bad = []
    if s_tp < SCORE_OK:
        bad.append('板端得分 %.2f < %.2f' % (s_tp, SCORE_OK))
    if d_cm > SHIFT_CM or abs(d_deg) > SHIFT_DEG:
        bad.append('局部再匹配偏 %.1f cm / %.2f°' % (d_cm, d_deg))
    if g is not None and g < LGS_OK:
        bad.append('车自报 global score %.2f < %.2f' % (g, LGS_OK))
    if sub is not None and sub != 0:
        bad.append('submap_in_map_score = %d' % sub)
    return dict(pose=tp, score=s_tp, best=best, best_score=s_best, d_cm=d_cm, d_deg=d_deg,
                global_score=g, submap=sub, bad=bad, ok=not bad)


def report(tag, c):
    print('%s：tracked %s，板端得分 %.3f，局部再匹配 %s（偏 %.1f cm / %.2f°），车自报 global %s、submap %s → %s'
          % (tag, fmt(c['pose']), c['score'], fmt(c['best']), c['d_cm'], c['d_deg'],
             'None' if c['global_score'] is None else '%.3f' % c['global_score'], c['submap'],
             '可信' if c['ok'] else '不可信（%s）' % '；'.join(c['bad'])), flush=True)


def port_up(host, port):
    try:
        socket.create_connection((host, port), timeout=1.5).close()
        return True
    except OSError:
        return False


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--host', default='192.168.31.231')
    p.add_argument('--port', type=int, default=9090)
    g = p.add_mutually_exclusive_group()
    g.add_argument('--check', action='store_true')
    g.add_argument('--force', action='store_true')
    p.add_argument('--wait-online', type=float, default=0.0)
    p.add_argument('--still-timeout', type=float, default=30.0)
    p.add_argument('--save', default='')
    a = p.parse_args()

    t_end = time.time() + a.wait_online
    if not port_up(a.host, a.port):
        if a.wait_online <= 0:
            print('连不上 %s:%d' % (a.host, a.port))
            return 3
        print('车不在线，最多等 %.0f s' % a.wait_online, flush=True)
        while not port_up(a.host, a.port):
            if time.time() > t_end:
                print('%.0f s 内车没上线' % a.wait_online)
                return 3
            time.sleep(2.0)
    car = Car(a.host, a.port)
    rec = {'host': a.host, 'start': time.strftime('%F %T')}
    M = pts = None
    try:
        if not car.connect(max(10.0, t_end - time.time())):
            print('rosbridge 连不上 %s:%d' % (a.host, a.port))
            return 3
        t_data = time.time() + 30
        while car.missing():
            if time.time() > t_data:
                print('30 s 内等不到 %s' % car.missing())
                return 3
            time.sleep(0.2)
        t_s = time.time() + 3.0  # 车自报得分是节流订阅，第一条可能晚一点到；没有这两个话题也照常往下走
        while None in car.scores() and time.time() < t_s:
            time.sleep(0.1)
        if not car.wait_still(2.0, a.still_timeout):
            print('%.0f s 内车没停下来，不检查' % a.still_timeout)
            return 4
        M = Matcher(car.grid())
        pts = car.fresh_points()
        if pts is None:
            print('等不到新的激光帧')
            return 3
        c0 = check(M, car, *pts)
        rec['check'] = c0
        report('检查', c0)
        if c0['ok'] and not a.force:
            return 0
        if a.check:
            return 6

        for k in range(2):  # 有人从旁边走过会拉低得分，换一批激光再试一次
            t0 = time.time()
            gm = M.global_match(*pts)
            trusted = gm['score'] >= SCORE_OK and gm['ratio'] <= RATIO_MAX
            print('全局匹配：%s 得分 %.3f，次优 (%.2f, %.2f, %.0f°) 得分 %.3f（%.2f 倍），候选 %d 个 × 步长 %.2f m，%.1f s → %s'
                  % (fmt(gm['pose']), gm['score'], gm['alt'][1], gm['alt'][2], math.degrees(gm['alt'][3]), gm['alt'][0],
                     gm['ratio'], gm['cands'], gm['step'], time.time() - t0, '可信' if trusted else '不可信'), flush=True)
            if trusted:
                break
            pts = car.fresh_points() or pts
        rec['match'] = gm
        if not trusted:
            print('匹配不可信，不发重定位：请确认车在这张地图里、周围没有大片遮挡，或者在 App 里手动重定位')
            return 5
        if car.moving():
            print('匹配完车动了，不发')
            return 4

        G = gm['pose']
        have_lgs = car.scores()[0] is not None
        t_send = time.time()
        r = car.set_relocation(G)
        rec['sent'] = G
        print('已发 /set_relocation %s，车端返回 %d' % (fmt(G), r), flush=True)
        if r != 0:
            print('车端拒绝（1 = 地图没加载好）')
            return 1
        # 车端过程（实测）：约 0.3 s 后新建轨迹，在初值周围 ±1.5 m、±90° 搜索约 2–2.5 s，这期间
        # /tracked_pose 停发、global score 掉到 0 附近，约 3.5 s 恢复。所以 3 s 内的数据一律不认，
        # 之后要求新收到的位姿到了发出的位置附近、车自报得分恢复，最多等到 15 s
        t_new, t_v, arrived = t_send + 3.0, t_send + 15.0, False
        while time.time() < t_v:
            if car.moving():
                car.stop()
                print('重定位后车在动，已发 /navi_stop 5')
                return 1
            if time.time() > t_new and car.recv_time('/tracked_pose') > t_new:
                tp = car.pose()
                sg, sub = car.scores(since=t_new)
                arrived = math.hypot(tp[0] - G[0], tp[1] - G[1]) < 0.05 and abs(wrap(tp[2] - G[2])) < math.radians(2)
                if arrived and (not have_lgs or (sg is not None and sg >= LGS_OK)) and (sub is None or sub == 0):
                    break
            time.sleep(0.1)
        print('重定位后 %.1f s 车端恢复' % (time.time() - t_send) if time.time() < t_v else '15 s 内车端没恢复', flush=True)
        pts = car.fresh_points() or pts
        c1 = check(M, car, *pts)
        rec['after'] = c1
        report('重定位后', c1)
        if not arrived:
            print('车的位姿没到发出的位置附近（5 cm / 2° 内）')
        return 0 if (c1['ok'] and arrived) else 1
    finally:
        if a.save and M is not None:
            os.makedirs(a.save, exist_ok=True)
            base = os.path.join(a.save, time.strftime('car_reloc_%Y%m%d_%H%M%S'))
            np.savez(base + '.npz', g=M.g, res=M.res, ox=M.ox, oy=M.oy,
                     px=pts[0] if pts is not None else [], py=pts[1] if pts is not None else [])
            with open(base + '.json', 'w') as f:
                json.dump(rec, f, ensure_ascii=False, indent=1, default=float)
            print('已存 %s.npz/.json' % base)
        car.close()


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
