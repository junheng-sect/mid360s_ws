#!/usr/bin/env bash
# 安装 / 更新 / 卸载常驻 roscore 和 MAVROS 两个 systemd 服务（板端执行，要 sudo）。
#   ./install.sh              复制 .service 到 /etc/systemd/system，开机自启并立即（重新）启动
#   ./install.sh --uninstall  停掉、取消自启并删除这两个服务（之后 fusion.sh 等入口会报「常驻 roscore 没起来」）
# 只有 .service 文件改了才需要重跑；run_roscore.sh / run_mavros.sh / mavros.launch 改了只要
# sudo systemctl restart mid360s-roscore（MAVROS 跟着重启）。
set -euo pipefail
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS=(mid360s-roscore.service mid360s-mavros.service)

if [ "${1:-}" = "--uninstall" ]; then
  sudo systemctl disable --now "${UNITS[@]}" || true
  for u in "${UNITS[@]}"; do sudo rm -f "/etc/systemd/system/$u"; done
  sudo systemctl daemon-reload
  echo "已卸载：${UNITS[*]}"
  exit 0
fi

for u in "${UNITS[@]}"; do
  sudo install -m 644 "$DIR/$u" "/etc/systemd/system/$u"
done
sudo systemctl daemon-reload
sudo systemctl enable "${UNITS[@]}"
# restart 而不是 start：更新时让新单元文件马上生效（MAVROS 随 roscore 一起重启）
sudo systemctl restart mid360s-roscore.service mid360s-mavros.service
systemctl --no-pager --lines=0 status "${UNITS[@]}" || true
