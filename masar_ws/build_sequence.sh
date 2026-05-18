#!/usr/bin/env bash
#
# build_sequence.sh 使用说明
# ----------------------------------------------------------------------
# 作用:
#   清理并重新构建当前 ROS1 catkin 工作空间中的全部有效包。
#   包列表由 catkin_topological_order 自动扫描 src 目录获得，新增包后通常不需要手动修改本脚本。
#
# 常用命令:
#   ./build_sequence.sh
#     清理 build/devel/install/logs 后，重新构建全部包。
#
#   ./build_sequence.sh -j8
#     清理后使用 8 个并行任务构建，适合日常全量重编译。
#
#   ./build_sequence.sh --no-clean -j8
#     不清理旧构建产物，只重新配置并增量构建；脚本会优先叠加现有 devel/setup.bash。
#
#   ./build_sequence.sh --list-packages
#     只查看当前会参与构建的包列表，不执行构建。
#
#   ./build_sequence.sh --dry-run -j8
#     只打印将要执行的清理和构建命令，不实际执行。
#
#   ./build_sequence.sh --clean-only
#     只删除 build/devel/install/logs，不执行构建。
#
# 注意:
#   1. 默认使用 /opt/ros/noetic/setup.bash 初始化 ROS 环境。
#   2. 脚本会显式清空 CATKIN_WHITELIST_PACKAGES，避免只构建旧白名单中的部分包。
#   3. 如需传递其他 catkin_make 参数，可直接追加在脚本参数后面。

set -Eeuo pipefail

usage() {
  cat <<'EOF'
用法:
  ./build_sequence.sh [选项] [额外 catkin_make 参数...]

功能:
  清理当前 catkin 工作空间的构建产物，并重新构建当前项目中的全部有效包。
  包列表由 catkin_topological_order 自动发现，避免新增包后手动列表遗漏。

选项:
  --no-clean       不清理 build/devel/install/logs，直接重新配置并构建。
  --clean-only     只清理构建产物，不执行构建。
  --list-packages  只打印当前会被 catkin 构建的包列表。
  --dry-run        只打印将执行的清理和构建命令，不实际执行。
  -j, --jobs N     传给 catkin_make 的并行任务数，例如 -j8。
  -h, --help       显示本帮助。

示例:
  ./build_sequence.sh
  ./build_sequence.sh --no-clean -j8
  ./build_sequence.sh --list-packages
  ./build_sequence.sh --dry-run
EOF
}

die() {
  echo "[build_sequence][错误] $*" >&2
  exit 1
}

log() {
  echo "[build_sequence] $*"
}

resolve_workspace_dir() {
  local script_dir
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

  if [[ -d "$script_dir/src" ]]; then
    echo "$script_dir"
    return
  fi

  local dir="$script_dir"
  while [[ "$dir" != "/" ]]; do
    if [[ -d "$dir/src" ]]; then
      echo "$dir"
      return
    fi
    dir="$(dirname "$dir")"
  done

  die "无法定位 catkin 工作空间，请从工作空间根目录运行该脚本。"
}

source_ros_env() {
  if [[ -f /opt/ros/noetic/setup.bash ]]; then
    # shellcheck source=/opt/ros/noetic/setup.bash
    source /opt/ros/noetic/setup.bash
  else
    die "未找到 /opt/ros/noetic/setup.bash，请确认 ROS1 Noetic 已安装。"
  fi

  unset CATKIN_WHITELIST_PACKAGES

  if [[ "$DO_CLEAN" == "0" && -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
    log "叠加现有工作空间环境: $WORKSPACE_DIR/devel/setup.bash"
    # shellcheck source=/dev/null
    source "$WORKSPACE_DIR/devel/setup.bash"
    unset CATKIN_WHITELIST_PACKAGES
  fi

  if [[ -d "$PYTHON_COMPAT_DIR" ]]; then
    export PYTHONPATH="$PYTHON_COMPAT_DIR${PYTHONPATH:+:$PYTHONPATH}"
  fi
}

discover_packages() {
  command -v catkin_topological_order >/dev/null 2>&1 || die "未找到 catkin_topological_order。"
  catkin_topological_order "$WORKSPACE_DIR/src"
}

print_packages() {
  local count
  count="$(discover_packages | wc -l | tr -d ' ')"
  log "当前有效 catkin 包数量: $count"
  discover_packages | awk '{printf "  %-36s %s\n", $1, $2}'
}

run_cmd() {
  if [[ "$DRY_RUN" == "1" ]]; then
    printf '[build_sequence][dry-run] '
    printf '%q ' "$@"
    printf '\n'
    return 0
  fi
  "$@"
}

cleanup_build_outputs() {
  log "清理旧构建产物: build devel install logs"
  run_cmd rm -rf "$WORKSPACE_DIR/build" "$WORKSPACE_DIR/devel" "$WORKSPACE_DIR/install" "$WORKSPACE_DIR/logs"
}

WORKSPACE_DIR="$(resolve_workspace_dir)"
PYTHON_COMPAT_DIR="$WORKSPACE_DIR/work/python_compat"
export ROS_PYTHON_VERSION="${ROS_PYTHON_VERSION:-3}"

DO_CLEAN=1
CLEAN_ONLY=0
LIST_ONLY=0
DRY_RUN=0
CATKIN_ARGS=()

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --no-clean)
      DO_CLEAN=0
      ;;
    --clean-only)
      CLEAN_ONLY=1
      ;;
    --list-packages)
      LIST_ONLY=1
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    -j|--jobs)
      shift || die "--jobs 需要一个数值。"
      [[ "${1:-}" =~ ^[0-9]+$ ]] || die "--jobs 参数必须是正整数。"
      CATKIN_ARGS+=("-j$1")
      ;;
    --)
      shift
      CATKIN_ARGS+=("$@")
      break
      ;;
    *)
      CATKIN_ARGS+=("$1")
      ;;
  esac
  shift
done

cd "$WORKSPACE_DIR"
source_ros_env

log "工作空间: $WORKSPACE_DIR"
print_packages

if [[ "$LIST_ONLY" == "1" ]]; then
  exit 0
fi

if [[ "$DO_CLEAN" == "1" ]]; then
  cleanup_build_outputs
else
  log "跳过清理，保留现有构建产物。"
fi

if [[ "$CLEAN_ONLY" == "1" ]]; then
  log "仅清理模式完成。"
  exit 0
fi

BUILD_CMD=(
  catkin_make
  --force-cmake
  -DCATKIN_WHITELIST_PACKAGES=
  "${CATKIN_ARGS[@]}"
)

log "开始全量构建当前项目中的所有有效包。"
run_cmd "${BUILD_CMD[@]}"

if [[ "$DRY_RUN" == "1" ]]; then
  log "dry-run 完成，未执行实际构建。"
  exit 0
fi

if [[ -f "$WORKSPACE_DIR/devel/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WORKSPACE_DIR/devel/setup.bash"
fi

log "全量构建完成。"
