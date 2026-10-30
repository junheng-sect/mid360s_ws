#!/usr/bin/env python3
"""本机只读状态桥：在本机 docker 里订阅板端几个低频话题，汇总成一份 JSON，给网页的无人机面板（car_web）每秒拉一次。

板端不跑任何新进程，只多一个订阅者；本节点只读：不发布、不调服务、不写参数，HTTP 也只有 GET /status。
数据都来自板端开机常驻的 MAVROS（mid360s_ws/scripts/resident/）和 fusion 的节点：
  /mavros/state             1 Hz   连上飞控、解锁、模式、system_status（STANDBY=当前模式解锁检查全过）
  /mavros/battery           0.5 Hz 电压、电流、剩余；台架没接电池时电压 0、剩余 < 0
  /mavros/sys_status        5 Hz   各部件健康位（enabled 且不 healthy 的就是不健康部件，含遥控）
  /mavros/extended_state    5 Hz   在地 / 在空
  /mavros/estimator_status  1 Hz   EKF 水平位置是否有效（没有 GPS 时只能来自视觉）
  /mavros/statustext/recv   变化时 飞控消息（Preflight Fail 的原因只在检查结果变化时发一次，这里缓存最近几条）
  /mavros/vision_pose/pose_cov     送给飞控的视觉位姿（FAST-LIO 频率），只记收到的时刻算频率
另外每 2 s 用 XML-RPC 问一次板端 master：/run_id、节点列表（fusion 的核心节点在不在）、/mavros 和 /laserMapping 的
身份和 PID。fusion 运行期间 mavros_node 的 PID 变了，就标“MAVROS 重启过”（时间同步要重新收敛，在飞应降落），直到下次 fusion。
本节点晚于那次重启才起来也认得出：mavros_node 的 PID 比 /laserMapping 的大，就是 fusion 开始后才（重新）起来的
（板端 pid_max 4194304，正常使用不会回绕）。

镜像里没有 mavros_msgs：按 AnyMsg 订阅，用发布者连接头里带的消息定义现场生成消息类再解析。
板端重启后 master 是新的（/run_id 变），ROS1 节点不会重新注册，所以发现 /run_id 变了就 exec 自己整个重来。
入口：docker/ros-noetic-rviz/scripts/run_drone_status.sh。
"""
import argparse
import collections
import http.server
import json
import os
import sys
import threading
import time
import xmlrpc.client
sys.dont_write_bytecode = True

import genpy.dynamic  # noqa: E402
import rospy  # noqa: E402

NAME = '/drone_status_check'
POLL_S = 2.0            # 问 master 的周期
RPC_TIMEOUT_S = 2.0
CORE_NODES = ('/livox_lidar_publisher2', '/laserMapping', '/gravity_bridge')  # fusion_core.launch 的三个节点
# 各话题多久没更新算过期（秒）：发布频率的 3~5 倍
STALE = {'state': 3.5, 'battery': 6.0, 'sys_status': 2.0, 'extended_state': 2.0, 'estimator_status': 3.5}
VISION_STALE_S = 0.5    # 视觉位姿是 FAST-LIO 频率，半秒没来就算断
MESSAGES_KEEP = 8
# MAV_STATE：PX4 心跳里的 system_status
SYSTEM_STATUS = {0: '自检未通过', 1: '启动中', 2: '校准中', 3: '自检通过', 4: '已解锁',
                 5: '失效保护', 6: '紧急', 7: '关机中', 8: '终止飞行'}
LANDED = {0: '未知', 1: '在地面', 2: '在空中', 3: '起飞中', 4: '降落中'}
# MAV_SYS_STATUS_SENSOR 里 PX4 1.16 会填的部件（SYS_STATUS.hpp 的 fillOutComponent）；同名的合并显示
SENSOR_BITS = ((0x1, '陀螺'), (0x2, '加计'), (0x4, '磁罗盘'), (0x8, '气压计'), (0x10, '空速计'), (0x20, 'GPS'),
               (0x800, '姿态估计'), (0x4000, '位置估计'), (0x8000, '电机/电调'), (0x10000, '遥控'),
               (0x200000, '姿态估计'), (0x1000000, '日志'), (0x2000000, '电池'), (0x80000000, '降落伞'))
RC_BIT = 0x10000
TOPICS = {'/mavros/state': 'state', '/mavros/battery': 'battery', '/mavros/sys_status': 'sys_status',
          '/mavros/extended_state': 'extended_state', '/mavros/estimator_status': 'estimator_status',
          '/mavros/statustext/recv': 'statustext', '/mavros/vision_pose/pose_cov': 'vision'}


class _Transport(xmlrpc.client.Transport):
    """带超时的 XML-RPC：板端掉线时不能让轮询线程一直卡住"""

    def make_connection(self, host):
        conn = super().make_connection(host)
        conn.timeout = RPC_TIMEOUT_S
        return conn


def rpc(uri):
    return xmlrpc.client.ServerProxy(uri, transport=_Transport(), allow_none=True)


class Bridge:
    def __init__(self, master_uri):
        self.master_uri = master_uri
        self.lock = threading.Lock()
        self.msgs = {}              # 键 → (收到时刻, 解析后的消息)
        self.vision_t = collections.deque(maxlen=200)
        self.messages = collections.deque(maxlen=MESSAGES_KEEP)
        self.classes = {}           # 消息类型 → 现场生成的类
        self.master_ok = False
        self.master_err = '还没连过'
        self.run_id = None
        self.nodes = set()
        self.mavros_pid = None
        self.fusion_run = None      # /laserMapping 的 URI：换了就是新的一次 fusion
        self.fusion_mavros_pid = None
        self.mavros_restarted = False

    # ------------------------------------------------------------ 订阅 ----
    def decode(self, raw):
        hdr = raw._connection_header
        typ = hdr['type']
        cls = self.classes.get(typ)
        if cls is None:
            cls = self.classes[typ] = genpy.dynamic.generate_dynamic(typ, hdr['message_definition'])[typ]
        return cls().deserialize(raw._buff)

    def on_msg(self, raw, key):
        now = time.time()
        if key == 'vision':         # 只要收到的时刻，不用解析
            with self.lock:
                self.vision_t.append(now)
            return
        try:
            msg = self.decode(raw)
        except Exception as e:  # noqa: BLE001
            rospy.logwarn_throttle(10, '%s 解析失败：%s' % (key, e))
            return
        with self.lock:
            if key == 'statustext':
                self.messages.append({'t': now, 'severity': int(msg.severity), 'text': msg.text.strip()})
            else:
                self.msgs[key] = (now, msg)

    def subscribe(self):
        for topic, key in TOPICS.items():
            rospy.Subscriber(topic, rospy.AnyMsg, self.on_msg, callback_args=key, queue_size=5)

    # ---------------------------------------------------------- 问 master ----
    def poll_master(self):
        m = rpc(self.master_uri)
        code, _, run_id = m.getParam(NAME, '/run_id')
        code_s, _, state = m.getSystemState(NAME)
        nodes = set()
        if code_s == 1:
            for group in state:
                for _, ns in group:
                    nodes.update(ns)
        def node_pid(node):
            """(节点 URI, PID)；查不到的给 None"""
            c, _, uri = m.lookupNode(NAME, node)
            if c != 1:
                return None, None
            try:
                return uri, rpc(uri).getPid(NAME)[2]
            except Exception:  # noqa: BLE001 - 节点刚好在重启
                return uri, None

        mavros_pid = node_pid('/mavros')[1] if '/mavros' in nodes else None
        fusion_run = core_pid = None
        if all(n in nodes for n in CORE_NODES):
            fusion_run, core_pid = node_pid('/laserMapping')
            fusion_run = fusion_run or 'unknown'
        return (run_id if code == 1 else None), nodes, mavros_pid, fusion_run, core_pid

    def poll_loop(self):
        while True:
            try:
                run_id, nodes, mavros_pid, fusion_run, core_pid = self.poll_master()
            except Exception as e:  # noqa: BLE001 - 板端掉线、Wi-Fi 断了
                with self.lock:
                    self.master_ok, self.master_err = False, str(e)
                time.sleep(POLL_S)
                continue
            if self.run_id is not None and run_id is not None and run_id != self.run_id:
                # 板端重启过，master 换了：ROS1 节点不会自己重新注册，整个进程重来
                print('[drone_status] 板端 master 换了（/run_id %s → %s），重启本节点' % (self.run_id, run_id), flush=True)
                os.execv(sys.executable, [sys.executable] + sys.argv)
            with self.lock:
                self.master_ok, self.master_err = True, ''
                self.nodes, self.mavros_pid = nodes, mavros_pid
                if fusion_run is None:
                    self.fusion_run, self.fusion_mavros_pid, self.mavros_restarted = None, None, False
                elif fusion_run != self.fusion_run:         # 新的一次 fusion（或本节点刚起来）：从现在的 MAVROS 开始算
                    late = mavros_pid is not None and core_pid is not None and mavros_pid > core_pid
                    self.fusion_run, self.fusion_mavros_pid, self.mavros_restarted = fusion_run, mavros_pid, late
                elif self.fusion_mavros_pid is None:
                    self.fusion_mavros_pid = mavros_pid
                elif mavros_pid is not None and mavros_pid != self.fusion_mavros_pid:
                    self.mavros_restarted = True
            time.sleep(POLL_S)

    # -------------------------------------------------------------- 汇总 ----
    def fresh(self, key, now):
        """(消息, 多少秒前)；没收到过或过期了消息给 None"""
        t, msg = self.msgs.get(key, (None, None))
        if t is None:
            return None, None
        age = now - t
        return (msg if age <= STALE[key] else None), round(age, 1)

    def snapshot(self):
        now = time.time()
        with self.lock:
            state, state_age = self.fresh('state', now)
            bat, bat_age = self.fresh('battery', now)
            sys_st, sys_age = self.fresh('sys_status', now)
            ext, ext_age = self.fresh('extended_state', now)
            est, est_age = self.fresh('estimator_status', now)
            vision = [t for t in self.vision_t if now - t <= 2.0]
            out = {'t': now, 'master': {'ok': self.master_ok, 'uri': self.master_uri, 'error': self.master_err},
                   'messages': list(self.messages)}
            nodes, mavros_pid, restarted = set(self.nodes), self.mavros_pid, self.mavros_restarted
            master_ok = self.master_ok

        fc = bool(state and state.connected)
        armed = bool(state and state.armed)
        sysst = int(state.system_status) if state else None
        out['flight'] = {'connected': fc, 'armed': armed if state else None, 'mode': state.mode if state else None,
                         'landed': LANDED.get(int(ext.landed_state), '未知') if ext else None,
                         'failsafe': sysst == 5, 'age': state_age}

        if bat:
            pct = float(bat.percentage)
            connected = bat.voltage > 1.0 and pct >= 0
            out['battery'] = {'connected': connected, 'voltage': round(float(bat.voltage), 2),
                              'current': None if bat.current != bat.current else round(float(bat.current), 1),
                              'percent': round(pct * 100) if pct >= 0 else None, 'age': bat_age}
        else:
            out['battery'] = {'connected': None, 'age': bat_age}

        unhealthy = []
        if sys_st:
            bad = sys_st.sensors_enabled & ~sys_st.sensors_health
            for bit, name in SENSOR_BITS:
                if bad & bit and name not in unhealthy:
                    unhealthy.append(name)
        checks = []

        def check(key, name, ok, text):
            checks.append({'key': key, 'name': name, 'ok': ok, 'text': text})

        # 1. PX4 自检（当前模式能不能解锁）
        if not fc:
            check('px4', 'PX4 自检', None, '飞控未连接')
        elif armed:
            check('px4', 'PX4 自检', sysst != 5, SYSTEM_STATUS.get(sysst, str(sysst)))
        else:
            reason = '' if sysst == 3 or not unhealthy else '（不健康：%s）' % '、'.join(unhealthy)
            check('px4', 'PX4 自检', sysst == 3, SYSTEM_STATUS.get(sysst, str(sysst)) + reason)
        # 2. 定位：fusion 核心节点都在、视觉位姿在送、EKF 水平位置有效
        missing = [n for n in CORE_NODES if n not in nodes]
        vision_age = round(now - vision[-1], 2) if vision else None
        vision_hz = round(len(vision) / 2.0, 1)
        ekf = bool(est.pos_horiz_rel_status_flag) if est else None
        if not master_ok:
            check('pos', '定位', None, '连不上板端')
        elif missing:
            check('pos', '定位', False, 'fusion 没在跑' if len(missing) == len(CORE_NODES) else '缺节点 ' + ' '.join(missing))
        elif vision_age is None or vision_age > VISION_STALE_S:
            check('pos', '定位', False, '没在给飞控送视觉位姿（FAST-LIO 还没初始化？）')
        elif ekf is None:
            check('pos', '定位', None, '视觉 %.0f Hz，EKF 状态未知' % vision_hz)
        else:
            check('pos', '定位', ekf, '视觉 %.0f Hz，EKF %s' % (vision_hz, '已接受' if ekf else '没接受'))
        out['positioning'] = {'fusion': not missing, 'missing': missing, 'vision_hz': vision_hz,
                              'vision_age': vision_age, 'ekf_pos': ekf, 'ekf_age': est_age}
        # 3. 遥控
        if not sys_st:
            check('rc', '遥控', None, '没有飞控健康数据')
        elif not sys_st.sensors_enabled & RC_BIT:
            check('rc', '遥控', None, '飞控没报遥控')
        else:
            ok = bool(sys_st.sensors_health & RC_BIT) and bool(sys_st.sensors_present & RC_BIT)
            check('rc', '遥控', ok, '在线' if ok else '不在线')
        # 4. fusion 运行期间 MAVROS 有没有重启过
        if mavros_pid is None:
            check('mavros', 'MAVROS', None if not master_ok else False, '连不上板端' if not master_ok else '不在运行')
        elif restarted:
            check('mavros', 'MAVROS', False, '本次 fusion 开始后重启过（或才起来）：时间同步重新收敛，在飞应降落')
        else:
            check('mavros', 'MAVROS', True, '正常（pid %s）' % mavros_pid)
        # 5. 数据新鲜度：电量、健康、在地在空这几项过期了也要标出来
        stale = [n for n, (msg, age) in (('状态', (state, state_age)), ('电量', (bat, bat_age)),
                                        ('健康', (sys_st, sys_age)), ('在地/在空', (ext, ext_age)))
                 if msg is None]
        if not master_ok:
            check('fresh', '数据', None, '连不上板端 master')
        elif not fc:
            check('fresh', '数据', None, '飞控未连接')
        else:
            check('fresh', '数据', not stale, '都是新的' if not stale else '过期：' + '、'.join(stale))
        out['checks'] = checks
        out['unhealthy'] = unhealthy

        # 总灯：解锁前看能不能起飞，解锁后看在飞是否正常
        by = {c['key']: c['ok'] for c in checks}
        if not master_ok:
            lamp = ('unknown', '连不上板端')
        elif not fc:
            lamp = ('unknown', '飞控未连接')
        elif armed:
            lamp = ('bad', '失效保护') if sysst == 5 else (('ok', '已解锁 · 正常') if by['pos'] and by['mavros']
                                                           else ('bad', '已解锁 · 定位或 MAVROS 异常'))
        elif all(by[k] for k in ('px4', 'pos', 'rc', 'mavros', 'fresh')):
            lamp = ('ok', '可以起飞')
        else:
            lamp = ('bad', '不可起飞')
        out['lamp'] = {'level': lamp[0], 'text': lamp[1]}
        return out


def serve(bridge, bind, port):
    class Handler(http.server.BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            if self.path.split('?')[0] != '/status':
                self.send_error(404)
                return
            body = json.dumps(bridge.snapshot(), ensure_ascii=False).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Access-Control-Allow-Origin', '*')   # 页面由 car_web 的 8080 提供，跨端口取
            self.send_header('Content-Length', str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    srv = http.server.ThreadingHTTPServer((bind, port), Handler)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', type=int, default=8091)
    p.add_argument('--bind', default='0.0.0.0', help='默认所有网卡：手机经台式机 IP 打开页面时也能取到')
    a = p.parse_args(rospy.myargv()[1:])

    bridge = Bridge(os.environ.get('ROS_MASTER_URI', 'http://10.250.32.1:11311'))
    serve(bridge, a.bind, a.port)
    print('[drone_status] http://%s:%d/status  master %s' % (a.bind, a.port, bridge.master_uri), flush=True)
    # master 连上之前不 init_node：页面照样能取到“连不上板端”
    while True:
        try:
            run_id = bridge.poll_master()[0]
            break
        except Exception as e:  # noqa: BLE001
            with bridge.lock:
                bridge.master_ok, bridge.master_err = False, str(e)
            time.sleep(POLL_S)
    bridge.run_id = run_id
    rospy.init_node('drone_status', anonymous=True, disable_signals=True)
    bridge.subscribe()
    threading.Thread(target=bridge.poll_loop, daemon=True).start()
    print('[drone_status] 已连上板端 master（run_id %s）' % run_id, flush=True)
    while True:
        time.sleep(3600)


if __name__ == '__main__':
    main()
