# shellcheck shell=bash
# source 本文件：定位核心运行期间冻结板端系统时钟。
# MID-360S 没接 PTP，雷达 / IMU 时间戳就是板端系统时间：运行中时间往前跳 FAST-LIO 的 IMU 积分会出错，往回跳
# 重力桥进 fatal、只停发位姿不退出。chrony 的 makestep 1 3 是 chronyd 启动后前 3 次更新允许跳，没网启动、
# 运行中才联网，或者运行中 chronyd 被重启，都会再跳一次。所以：
#   clock_freeze   核心起来前调用：持有 CLOCK_FREEZE_LOCK（fd 201）并停掉 chronyd。之后系统时钟既不跳也不调速，
#                  内核保持 chrony 已校准的频率，漂移是 ppm 级。
#   clock_release  核心全部退出后调用：释放锁再启动 chronyd，有大偏差就在这时一次跳过去。
# 运行中防被拉起：chrony.service 的 drop-in（/etc/systemd/system/chrony.service.d/mid360s-core-guard.conf →
# /usr/local/sbin/mid360s-chrony-guard）在锁被占用时拒绝启动，换网、unattended-upgrades 重启 chrony 都挡住。
# fd 201 会被调用方拉起的 roslaunch 继承：进程全退（含 kill -9）锁自动释放，chronyd 不会被永久挡住。
# 锁放 /run/mid360s（/etc/tmpfiles.d/mid360s.conf 开机建、归 orangepi）：chrony.service 是 PrivateTmp，看不到 /tmp。
# 停 / 起 chronyd 靠 /etc/sudoers.d/mid360s-chrony 免密，只放行这两条命令。
#
# roslaunch 用 setsid 起节点、不传 fd：roslaunch 被强杀时节点成孤儿继续跑，锁却跟着 roslaunch 放掉了。所以冻结期间
# 起的进程都用 clock_env 打上环境变量标记（会一路继承到节点）：MID360S_RUN=<本次标记>:<组名> 用来收拢本次残留，
# MID360S_CLOCK_FROZEN=1 让 chrony 的防启动检查认出任何一次运行留下的孤儿；残留全退之前不恢复 chronyd。
CLOCK_FREEZE_LOCK="/run/mid360s/clock_freeze.lock"
CLOCK_FROZEN=0
CLOCK_RUN_TAG="mid360s-$$-$(date +%s)"

# clock_env 组名：输出给 env 用的标记，用法 env $(clock_env core) roslaunch …
clock_env() { printf 'MID360S_RUN=%s:%s MID360S_CLOCK_FROZEN=1' "$CLOCK_RUN_TAG" "$1"; }

# tagged_pids [组名]：环境里带本次标记的进程（不给组名就是本次全部）。标记在 exec 时进 /proc/<pid>/environ，
# 调用方自己的 export 不会出现在它自己的 environ 里，所以只认得到带标记起的进程。
tagged_pids() {
  grep -lzE "^MID360S_RUN=${CLOCK_RUN_TAG}:${1:-.*}\$" /proc/[0-9]*/environ 2>/dev/null \
    | sed 's#^/proc/\([0-9]*\)/environ$#\1#'
}

# reap_tagged 组名 名称：收拢该组残留进程（SIGINT 10 s → SIGTERM 5 s），收不完返回 1
reap_tagged() {
  local _p _i=0
  _p="$(tagged_pids "$1")"
  [ -n "$_p" ] || return 0
  echo "[clock_guard] $2 还有残留进程（$(echo $_p)），SIGINT 收拢" >&2
  # shellcheck disable=SC2086
  kill -INT $_p 2>/dev/null
  while [ "$_i" -lt 20 ]; do
    sleep 0.5; _i=$((_i + 1))
    _p="$(tagged_pids "$1")"
    [ -n "$_p" ] || return 0
  done
  echo "[clock_guard] $2 残留进程 10 s 没退，SIGTERM（$(echo $_p)）" >&2
  # shellcheck disable=SC2086
  kill -TERM $_p 2>/dev/null
  _i=0
  while [ "$_i" -lt 10 ]; do
    sleep 0.5; _i=$((_i + 1))
    _p="$(tagged_pids "$1")"
    [ -n "$_p" ] || return 0
  done
  echo "[clock_guard] $2 仍有残留进程（$(echo $_p)），需手动处理" >&2
  return 1
}

chrony_active() { systemctl is-active --quiet chrony.service; }

# 对时前确保 chronyd 在跑：上次异常退出、锁被残留进程占着时可能停着
clock_ensure_running() {
  chrony_active && return 0
  sudo -n /usr/bin/systemctl start chrony.service >/dev/null 2>&1
}

# 返回 1：锁被别的实例占着；2：停不掉 chronyd（sudoers 没装？）。失败时不持有锁
clock_freeze() {
  exec 201>>"$CLOCK_FREEZE_LOCK" || return 1
  if ! flock -n 201; then
    exec 201>&-
    return 1
  fi
  if ! sudo -n /usr/bin/systemctl stop chrony.service >/dev/null 2>&1 || chrony_active; then
    exec 201>&-
    return 2
  fi
  CLOCK_FROZEN=1
}

# 返回 1：chronyd 没起来（多半还有核心进程占着锁）；2：本次带标记的进程还有残留，chronyd 保持停止
# （防启动检查也会挡住它）。两种情况都等残留退出后手动 sudo systemctl start chrony。
clock_release() {
  [ "$CLOCK_FROZEN" -eq 1 ] || return 0
  CLOCK_FROZEN=0
  exec 201>&-
  [ -z "$(tagged_pids)" ] || return 2
  sudo -n /usr/bin/systemctl start chrony.service >/dev/null 2>&1 && chrony_active
}
