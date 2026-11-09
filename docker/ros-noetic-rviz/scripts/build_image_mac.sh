#!/usr/bin/env bash
# 构建 Mac ARM64 RViz 显示镜像；Ubuntu 继续使用 build_image.sh。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# 与启动入口共用本机镜像标签设置。
source "$SCRIPT_DIR/common_mac.sh"
BASE_DIGEST=sha256:c84e573d2f2beb679ea5b03015eb2876e9962657c23d0073e4ae9d7ba41c4700
BASE_IMAGE=ros:noetic-ros-base
build_args=(--platform linux/arm64 -f "$CONFIG_DIR/Dockerfile.macos")
ENV_FILE="$CONFIG_DIR/.image.mac.env"

log() { printf '[build] %s\n' "$*"; }
die() { printf '[build][错误] %s\n' "$*" >&2; exit 1; }

check_docker

log "构建 ${IMAGE_TAG}（基础镜像摘要 ${BASE_DIGEST}）"
docker build "${build_args[@]}" -t "$IMAGE_TAG" "$CONFIG_DIR"

image_id="$(docker image inspect "$IMAGE_TAG" --format '{{.Id}}')"
{
  printf 'IMAGE_TAG=%s\n' "$IMAGE_TAG"
  printf 'IMAGE_ID=%s\n' "$image_id"
  printf 'BASE_IMAGE=%s\n' "$BASE_IMAGE"
  printf 'BASE_DIGEST=%s\n' "$BASE_DIGEST"
  printf 'BUILT_AT=%s\n' "$(date '+%F %T %z')"
} >"$ENV_FILE"

log "完成：$IMAGE_TAG"
log "镜像 ID：$image_id"
log "记录写入：$ENV_FILE"
