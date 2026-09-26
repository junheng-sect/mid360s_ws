#!/usr/bin/env bash
# 构建 RViz 显示镜像。基础镜像已在本地，构建通常只需几十秒。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 与启动入口共用本机镜像标签设置。
source "$SCRIPT_DIR/common.sh"
BASE_DIGEST="sha256:7dbfb9576d8e6d226c31e06129a82aaab8702695f38eca2116918cb9b9308797"
ENV_FILE="$CONFIG_DIR/.image.env"

log() { printf '[build] %s\n' "$*"; }
die() { printf '[build][错误] %s\n' "$*" >&2; exit 1; }

docker info >/dev/null 2>&1 || die "Docker 守护进程不可用：确认 docker 服务已启动、当前用户在 docker 组。"

log "构建 $IMAGE_TAG（基础镜像摘要 $BASE_DIGEST）"
docker build -t "$IMAGE_TAG" "$CONFIG_DIR"

image_id="$(docker image inspect "$IMAGE_TAG" --format '{{.Id}}')"
{
  printf 'IMAGE_TAG=%s\n' "$IMAGE_TAG"
  printf 'IMAGE_ID=%s\n' "$image_id"
  printf 'BASE_IMAGE=osrf/ros:noetic-desktop-full\n'
  printf 'BASE_DIGEST=%s\n' "$BASE_DIGEST"
  printf 'BUILT_AT=%s\n' "$(date '+%F %T %z')"
} >"$ENV_FILE"

log "完成：$IMAGE_TAG"
log "镜像 ID：$image_id"
log "记录写入：$ENV_FILE"
