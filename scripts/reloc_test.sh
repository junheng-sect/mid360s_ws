#!/usr/bin/env bash
# ==============================================================================
# 重定位实机测试 一键启动：MID-360S 驱动 + FAST-LIO2 + 重力桥（不送飞控）+ drone_relocalization
#
# 路径: /home/orangepi/mid360s_ws/scripts/reloc_test.sh
# 用法: ./reloc_test.sh [选项]
#   （默认）室外图 20261105_114605_outdoor + config/outdoor.yaml（起飞圆柱：原点 3 m 内）
#   --indoor           室内图 20261021_110311_indoor + config/indoor.yaml
#   --map DIR          指定地图目录（换图时一般也要 --config）
#   --config FILE      指定重定位配置
#   --no-record        不录包（默认录雷达 / IMU / 里程计 / 重定位话题，失败时能离线复现）
#   -h|--help          帮助
#
# 开机后直接运行。不在 tmux 里会自己开一个 tmux 会话 reloc：ssh 断了也接着跑，回来 tmux attach -t reloc。
# 飞机放好就别动，直到打印「已确认」；这段时间人别在飞机旁边走动（挡住墙，二维得分不够会被拒）。
# 定位成功后每 2 s 打印一次当前位置，可以搬到量过的位置核对；Ctrl+C 结束，统一收尾、恢复对时。
# 每次的日志、结果 json、包和参数在 ~/test_data/reloc_test/<时间>/。
# 只做重定位：不送飞控、不起 car_bridge、不改任何参数。和 fusion.sh / start_handheld_test.sh 不能同时跑。
# ==============================================================================
set -o pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$DIR/.." && pwd)"
ROS_SETUP="/opt/ros/noetic/setup.bash"
WS_SETUP="$WS/devel/setup.bash"
FASTLIO_LAUNCHER="$DIR/start_mid360s_fastlio.sh"
RELOC_CFG_DIR="$WS/src/drone_relocalization/config"
MAP_ROOT="$HOME/test_data/mapping"

MAP="$MAP_ROOT/20261105_114605_outdoor"; CONFIG="$RELOC_CFG_DIR/outdoor.yaml"
DO_RECORD=1
while [ $# -gt 0 ]; do
  case "$1" in
    --indoor) MAP="$MAP_ROOT/20261021_110311_indoor"; CONFIG="$RELOC_CFG_DIR/indoor.yaml" ;;
    --map) MAP="${2:-}"; shift ;;
    --config) CONFIG="${2:-}"; shift ;;
    --no-record) DO_RECORD=0 ;;
    -h|--help) awk 'NR>2 && /^# =+$/{exit} NR>2{sub(/^# ?/, ""); print}' "${BASH_SOURCE[0]}"; exit 0 ;;
    *) echo "未知参数: $1（-h 看用法）" >&2; exit 2 ;;
  esac
  shift
done

# ssh 断开不能把测试停在半路：不在 tmux 里就开一个会话再跑自己（已有会话就接回去）
if [ -z "${TMUX:-}" ] && [ -t 0 ] && command -v tmux >/dev/null 2>&1; then
  if tmux has-session -t reloc 2>/dev/null; then
    echo "已有 tmux 会话 reloc（上一次的测试），接回去；要重新开始先在里面 Ctrl+C、按回车关掉它"
    exec tmux attach -t reloc
  fi
  _args=(--map "$MAP" --config "$CONFIG"); [ "$DO_RECORD" -eq 1 ] || _args+=(--no-record)
  exec tmux new-session -s reloc "RELOC_TEST_IN_TMUX=1 $(printf '%q ' "$DIR/$(basename "${BASH_SOURCE[0]}")" "${_args[@]}")"
fi

MAP="$(cd "$MAP" 2>/dev/null && pwd)" || { echo "地图目录不存在" >&2; exit 2; }
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT="$HOME/test_data/reloc_test/$STAMP"
mkdir -p "$OUT"
LOG="$OUT/reloc_test.log"
# 本次运行专属的临时目录（pid 文件、显示脚本），收尾时删掉
RUN_DIR="$(mktemp -d /tmp/reloc_test.XXXXXX)"
PF_LIO="$RUN_DIR/fastlio.pid"; PF_BAG="$RUN_DIR/bag.pid"; PF_RELOC="$RUN_DIR/reloc.pid"

log()  { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
ok()   { printf '[%s] [ OK ] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
warn() { printf '[%s] [WARN] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG"; }
err()  { printf '[%s] [FAIL] %s\n' "$(date +%H:%M:%S)" "$*" | tee -a "$LOG" >&2; }
hr()   { printf '%s\n' "----------------------------------------------------------------" | tee -a "$LOG"; }

# ---------------------------------------------------------------- 子进程 ----
# 同 start_handheld_test.sh：子进程各开一个会话（终端的 Ctrl+C 绕不到它们，由本脚本统一转发），
# pid 文件里记「PID + 启动时刻」，收尾时核对身份，PID 被复用也不会误停别的进程。
STARTED=" "
proc_starttime() {
  local _s; _s="$(cat "/proc/$1/stat" 2>/dev/null)" || return 1
  _s="${_s##*)}"
  # shellcheck disable=SC2086
  set -- $_s; [ "$#" -ge 20 ] || return 1; printf '%s\n' "${20}"
}
child_pid() { # child_pid <pidfile>：核得上身份才输出 PID
  case "$STARTED" in *" $1 "*) ;; *) return 1 ;; esac
  local _pid _st; read -r _pid _st <"$1" 2>/dev/null || return 1
  [ -n "$_pid" ] && kill -0 "$_pid" 2>/dev/null || return 1
  [ "$(proc_starttime "$_pid")" = "$_st" ] || return 1
  printf '%s\n' "$_pid"
}
start_child() { # start_child <pidfile> <logfile> <命令...>
  local _pf="$1" _lf="$2"; shift 2
  STARTED="$STARTED$_pf "
  setsid --wait bash -c '
    _pf="$1"; shift; _cmd=("$@")
    _s="$(cat /proc/$$/stat 2>/dev/null)"; _s="${_s##*)}"; set -- $_s
    printf "%s %s\n" "$$" "${20:-}" >"$_pf"
    exec "${_cmd[@]}"
  ' _ "$_pf" "$@" </dev/null >"$_lf" 2>&1 &
  local _i=0
  while [ "$_i" -lt 50 ] && [ ! -s "$_pf" ]; do sleep 0.1; _i=$((_i + 1)); done
}
# 后台子进程继承的 SIGINT 是「忽略」，bash 脚本改不回来：start_mid360s_fastlio.sh 要发 TERM（它对 TERM
# 装了同一个收尾 trap，会转 SIGINT 给 roslaunch、再恢复对时）；rospy 节点和 rosbag 自己注册了 SIGINT，发 INT。
stop_child() { # stop_child <pidfile> <名字> <INT|TERM> <最多等几秒>：每 5 s 补发一次；超时只提示，不强杀（免得打断恢复对时）
  local _pid _i=0
  _pid="$(child_pid "$1")" || return 0
  log "停止$2（pid $_pid）"; kill -"$3" "$_pid" 2>/dev/null
  while child_pid "$1" >/dev/null; do
    sleep 0.5; _i=$((_i + 1))
    [ $((_i % 10)) -eq 0 ] && kill -"$3" "$_pid" 2>/dev/null
    if [ "$_i" -ge $(($4 * 2)) ]; then warn "$2 ${4} s 内没退出（pid $_pid），请手动处理"; return 1; fi
  done
}

# ---------------------------------------------------------------- 收尾 ------
SHUTTING_DOWN=0; WATCH_PID=""
finish() {
  local _code="$1"
  [ "$SHUTTING_DOWN" -eq 1 ] && return; SHUTTING_DOWN=1
  hr; log "收尾（只停本次启动的进程）"
  [ -n "$WATCH_PID" ] && kill -TERM "$WATCH_PID" 2>/dev/null
  stop_child "$PF_RELOC" "重定位节点" INT 10
  stop_child "$PF_BAG" "录包" INT 15
  stop_child "$PF_LIO" "雷达/FAST-LIO/重力桥（会恢复对时）" TERM 40
  rm -rf "$RUN_DIR"
  # 测完常直接断电：包的索引还在页缓存里就会丢（11-05 第一次测试的包就是这样，要 rosbag reindex）
  log "写盘（sync）……"; sync
  hr
  local _res; _res="$(ls -t "$OUT"/drone_reloc_*.json 2>/dev/null | head -1)"
  if [ -n "$_res" ]; then
    ok "本次定位成功：$(grep -h '定位成功' "$OUT/relocalize.log" 2>/dev/null | tail -1)"
    log "结果：$_res"; _code=0
  else
    err "本次没有定位成功：$(grep -h '^搜索' "$OUT/relocalize.log" 2>/dev/null | tail -1)"
    log "重定位节点日志：$OUT/relocalize.log"; [ "$_code" -eq 0 ] && _code=1
  fi
  [ -f "$OUT/reloc.bag" ] && log "包：$OUT/reloc.bag（$(du -h "$OUT/reloc.bag" | cut -f1)）"
  log "全部记录：$OUT"
  if [ "${RELOC_TEST_IN_TMUX:-0}" = 1 ]; then read -r -p "按回车关闭这个 tmux 会话 " _ </dev/tty; fi
  exit "$_code"
}
trap 'finish 130' INT
trap 'finish 143' TERM
trap 'finish 129' HUP

# ---------------------------------------------------------------- 检查 ------
hr; log "重定位实机测试（记录：$OUT）"
log "地图 $MAP"; log "配置 $CONFIG"
[ -f "$ROS_SETUP" ] || { err "缺少 $ROS_SETUP"; finish 1; }
[ -f "$WS_SETUP" ] || { err "缺少 $WS_SETUP，先在板端 catkin build"; finish 1; }
[ -x "$FASTLIO_LAUNCHER" ] || { err "缺少 $FASTLIO_LAUNCHER"; finish 1; }
[ -f "$MAP/map_world.pcd" ] && [ -f "$MAP/meta.json" ] || { err "$MAP 里没有 map_world.pcd / meta.json"; finish 1; }
[ -f "$CONFIG" ] || { err "配置不存在：$CONFIG"; finish 1; }
source "$ROS_SETUP"; source "$WS_SETUP"
source "$DIR/ros_net.sh" || { err "ROS 核心地址不在（nmcli con up ros-core）"; finish 1; }
if timeout 5 rosnode list 2>/dev/null | grep -qx /drone_reloc; then
  err "已有 /drone_reloc 节点在跑（另一个重定位实例？），先停掉它"; finish 1
fi
timeout 5 rosnode list >/dev/null 2>&1 && ok "ROS master 可达（$ROS_MASTER_URI）" \
  || warn "常驻 roscore 不可达：传感器链路会自己起一个 master（systemctl status mid360s-roscore）"

# 查找表缓存：指纹含地图和整套配置，第一次或换配置要重建（室外图约 70 s）。先在这里建好，
# 在线节点载入只要 1～2 s，不占它 60 s 的定位预算。
hr; log "步骤 1/4：载入地图（缓存不对就重建，室外图约 70 s，只第一次）"
if ! python3 - "$MAP" "$CONFIG" 2>&1 <<'PY' | tee -a "$LOG"; then err "地图载入失败"; finish 1; fi
import sys
from drone_relocalization.settings import load
from drone_relocalization.matching import MapModel
m = MapModel(sys.argv[1], load(sys.argv[2]))
print('地图 %d 点，载入 %.1f s，%s' % (len(m.P), m.load_s, '缓存命中' if m.cache_hit else '已重建缓存'))
PY

# ---------------------------------------------------------------- 启动 ------
hr; log "步骤 2/4：启动雷达 + FAST-LIO + 重力桥（不送飞控）。现在开始保持静止"
start_child "$PF_LIO" "$OUT/fastlio.log" "$FASTLIO_LAUNCHER"
_t=0
until timeout 3 rostopic echo -n1 /Odometry >/dev/null 2>&1; do
  if ! child_pid "$PF_LIO" >/dev/null; then
    err "传感器链路退出了，最后几行："; tail -n 8 "$OUT/fastlio.log" | tee -a "$LOG"; finish 1
  fi
  _t=$((_t + 3)); [ "$_t" -ge 90 ] && { err "90 s 内没有 /Odometry，看 $OUT/fastlio.log"; finish 1; }
  sleep 1
done
ok "FAST-LIO 已输出 /Odometry"
rosparam dump "$OUT/params.yaml" 2>/dev/null || warn "参数没存下来（离线复现会用默认外参）"

if [ "$DO_RECORD" -eq 1 ]; then
  hr; log "步骤 3/4：录包 $OUT/reloc.bag（约 130 MB/min）"
  start_child "$PF_BAG" "$OUT/rosbag.log" rosbag record -O "$OUT/reloc.bag" /livox/lidar /livox/imu /Odometry /tf_static \
    /fastlio/gravity_init /drone_reloc/result /drone_reloc/status /drone_reloc/pose
else
  hr; log "步骤 3/4：按要求不录包"
fi

hr; log "步骤 4/4：启动重定位（第一个窗口算 6～12 s，第二个窗口确认 1～3 s）"
start_child "$PF_RELOC" "$OUT/relocalize.log" rosrun drone_relocalization relocalize --map "$MAP" --config "$CONFIG" --save "$OUT"

# 屏幕上只看结论：每次搜索的结果和原因、状态变化，定位成功后每 2 s 的当前位置（同时写进日志）
cat >"$RUN_DIR/watch.py" <<'PY'
import json, math, sys, time
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
logf = open(sys.argv[1], 'a')
def out(s):
    line = '[%s] %s' % (time.strftime('%H:%M:%S'), s)
    print(line, flush=True); logf.write(line + '\n'); logf.flush()
st = dict(key=None, valid=False, pose=None, t=0.)
def on_status(m):
    s = json.loads(m.data); st['valid'] = bool(s.get('valid'))
    key = (s.get('state'), s.get('why'))
    if key != st['key']:
        st['key'] = key
        out('状态 %s：%s（已搜索 %d 次，确认 %d 次）' % (s.get('state'), s.get('why'), s.get('attempts', 0), s.get('confirmations', 0)))
def on_result(m):
    r = json.loads(m.data)
    if 'drone_map' not in r:
        if 'secs' in r: out('搜索结果：拒绝｜原因：%s' % r.get('why', ''))
        return
    p = r['drone_map']
    tag = '已确认' if r.get('confirmed') else ('可信候选，等下一个窗口确认' if r.get('candidate_ok') else '拒绝')
    out('搜索结果：%s｜位置 x=%.3f y=%.3f z=%.3f m（离原点水平 %.2f m）｜内点率 %.3f 残差 %.1f cm 歧义 %.2f｜算了 %.1f s%s' % (
        tag, p[0], p[1], p[2], math.hypot(p[0], p[1]), r.get('fitness', 0), 100 * r.get('rmse', 0), r.get('amb3d', 0),
        r.get('secs', 0), '' if r.get('candidate_ok') else '｜原因：' + r.get('why', '')))
def on_pose(m):
    st['pose'] = m; st['t'] = time.monotonic()
rospy.init_node('reloc_test_watch', anonymous=True)
rospy.Subscriber('/drone_reloc/status', String, on_status, queue_size=10)
rospy.Subscriber('/drone_reloc/result', String, on_result, queue_size=10)
rospy.Subscriber('/drone_reloc/pose', PoseStamped, on_pose, queue_size=1)
while not rospy.is_shutdown():
    time.sleep(2)
    m = st['pose']
    if st['valid'] and m is not None and time.monotonic() - st['t'] < 1:
        p, q = m.pose.position, m.pose.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        out('当前（地图系）x=%.3f y=%.3f z=%.3f m，航向 %.1f°，离原点水平 %.2f m' % (p.x, p.y, p.z, yaw, math.hypot(p.x, p.y)))
PY
python3 "$RUN_DIR/watch.py" "$LOG" </dev/null &
WATCH_PID=$!

echo
echo "=================================================================="
echo "  保持静止，等「已确认」；之后可以搬动飞机看当前位置。Ctrl+C 结束"
echo "=================================================================="
# 重定位节点自己退出（3 次搜索都没确认、60 s 超时、输入断流）或传感器链路掉了，就收尾
while :; do
  sleep 1
  if ! child_pid "$PF_RELOC" >/dev/null; then
    sleep 1; err "重定位节点已退出：$(tail -n 1 "$OUT/relocalize.log" 2>/dev/null)"; finish 1
  fi
  if ! child_pid "$PF_LIO" >/dev/null; then
    err "传感器链路掉了，看 $OUT/fastlio.log"; finish 1
  fi
done
