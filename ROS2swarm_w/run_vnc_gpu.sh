#!/usr/bin/env bash
# 作用：在 Docker 容器内对齐宿主机 TurboVNC/X11 + VirtualGL + GPU 渲染环境。
#
# 背景：
#   宿主机重启后，TurboVNC 的 DISPLAY 号和 Xauthority cookie 可能变化。
#   如果容器沿用旧的 DISPLAY（例如 :0），Webots 窗口会被发到不可见的 X server，
#   或者只使用 llvmpipe 软件渲染。这个脚本把容器内环境重新指向当前 VNC 桌面，
#   并让现有 start_simulation.sh / start_robosar_webots.sh 默认通过 vglrun 使用 GPU。
#
# 与宿主机对齐方式：
#   1. 宿主机启动 TurboVNC 后，确认 VNC 桌面标题或终端中的 DISPLAY，例如 :1。
#   2. 启动容器时挂载宿主机 X11 socket 和 xauth 文件：
#        -v /tmp/.X11-unix:/tmp/.X11-unix:rw
#        -v "$HOST_XAUTH:/tmp/.docker.xauth:ro"
#        -e XAUTHORITY=/tmp/.docker.xauth
#      如果宿主机重启后 cookie 变了，需要在宿主机重新生成 HOST_XAUTH 后重启容器。
#   3. 如果自动检测不符合当前 VNC，可显式覆盖：
#        export ROS2SWARM_VNC_DISPLAY=:1
#        export ROS2SWARM_XAUTHORITY=/tmp/.docker.xauth
#
# 常用方式：
#   ./run_vnc_gpu.sh --check
#   ./run_vnc_gpu.sh -- ./start_simulation.sh
#   source ./run_vnc_gpu.sh
#
# 项目启动脚本会自动 source 本文件，因此通常直接运行：
#   ./start_simulation.sh
#   ./start_robosar_webots.sh

_run_vnc_gpu_is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "$0" ]]
}

_run_vnc_gpu_normalize_display() {
  local value="${1:-}"
  if [[ -z "$value" ]]; then
    return 1
  fi

  if [[ "$value" == :* ]]; then
    printf '%s\n' "$value"
  else
    printf ':%s\n' "$value"
  fi
}

_run_vnc_gpu_display_number() {
  local display="${1:-}"
  display="${display#*:}"
  display="${display%%.*}"
  printf '%s\n' "$display"
}

_run_vnc_gpu_detect_xauthority() {
  if [[ -n "${ROS2SWARM_XAUTHORITY:-}" ]]; then
    printf '%s\n' "$ROS2SWARM_XAUTHORITY"
  elif [[ -n "${XAUTHORITY:-}" && -r "${XAUTHORITY}" ]]; then
    printf '%s\n' "$XAUTHORITY"
  elif [[ -r /tmp/.docker.xauth ]]; then
    printf '%s\n' /tmp/.docker.xauth
  elif [[ -r "${HOME:-/root}/.Xauthority" ]]; then
    printf '%s\n' "${HOME:-/root}/.Xauthority"
  else
    printf '%s\n' /tmp/.docker.xauth
  fi
}

_run_vnc_gpu_display_from_xauth() {
  local xauth_file="$1"
  if [[ ! -r "$xauth_file" ]] || ! command -v xauth >/dev/null 2>&1; then
    return 1
  fi

  local displays
  displays="$(
    xauth -f "$xauth_file" list 2>/dev/null |
      sed -nE 's/^[^[:space:]]*(\/unix)?:([0-9]+)(\.[0-9]+)?[[:space:]].*/:\2/p' |
      sort -u
  )"

  local display
  while IFS= read -r display; do
    if [[ -n "$display" && "$display" != ":0" ]]; then
      printf '%s\n' "$display"
      return 0
    fi
  done <<< "$displays"

  while IFS= read -r display; do
    if [[ -n "$display" ]]; then
      printf '%s\n' "$display"
      return 0
    fi
  done <<< "$displays"

  return 1
}

_run_vnc_gpu_display_from_socket_dir() {
  local socket_dir="${ROS2SWARM_X11_SOCKET_DIR:-/tmp/.X11-unix}"
  local socket

  if [[ ! -d "$socket_dir" ]]; then
    return 1
  fi

  for socket in "$socket_dir"/X*; do
    [[ -S "$socket" ]] || continue
    local number="${socket##*/X}"
    if [[ "$number" != "0" ]]; then
      printf ':%s\n' "$number"
      return 0
    fi
  done

  for socket in "$socket_dir"/X*; do
    [[ -S "$socket" ]] || continue
    printf ':%s\n' "${socket##*/X}"
    return 0
  done

  return 1
}

_run_vnc_gpu_detect_display() {
  local xauth_file="$1"

  if [[ -n "${ROS2SWARM_VNC_DISPLAY:-}" ]]; then
    _run_vnc_gpu_normalize_display "$ROS2SWARM_VNC_DISPLAY"
  elif [[ -n "${VNC_DISPLAY:-}" ]]; then
    _run_vnc_gpu_normalize_display "$VNC_DISPLAY"
  elif _run_vnc_gpu_display_from_xauth "$xauth_file"; then
    return 0
  elif _run_vnc_gpu_display_from_socket_dir; then
    return 0
  elif [[ -n "${DISPLAY:-}" ]]; then
    _run_vnc_gpu_normalize_display "$DISPLAY"
  else
    printf '%s\n' ':0'
  fi
}

_run_vnc_gpu_configure() {
  local xauth_file
  xauth_file="$(_run_vnc_gpu_detect_xauthority)"

  local display
  display="$(_run_vnc_gpu_detect_display "$xauth_file")"

  export DISPLAY="$display"
  export XAUTHORITY="$xauth_file"
  export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
  export NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"

  # 使用 VirtualGL 时不应强制 indirect GL，否则 Webots 可能退回软件渲染。
  unset LIBGL_ALWAYS_INDIRECT

  export ROS2SWARM_USE_VGLRUN="${ROS2SWARM_USE_VGLRUN:-true}"
  export ROBOSAR_USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-true}"
  export ROS2SWARM_VGLRUN_DEVICE="${ROS2SWARM_VGLRUN_DEVICE:-/dev/dri/card2}"
  export ROBOSAR_VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-${ROS2SWARM_VGLRUN_DEVICE}}"
  export ROS2SWARM_VNC_GPU_DISPLAY="$DISPLAY"
  export ROS2SWARM_VNC_GPU_XAUTHORITY="$XAUTHORITY"
  export ROS2SWARM_VNC_GPU_LOADED=1
}

_run_vnc_gpu_print_env() {
  printf '[run_vnc_gpu] DISPLAY=%s XAUTHORITY=%s ROS2SWARM_VGLRUN_DEVICE=%s\n' \
    "${DISPLAY:-}" "${XAUTHORITY:-}" "${ROS2SWARM_VGLRUN_DEVICE:-}"
}

_run_vnc_gpu_check() {
  local rc=0
  local display_number
  display_number="$(_run_vnc_gpu_display_number "${DISPLAY:-}")"

  printf '[run_vnc_gpu][check] DISPLAY=%s\n' "${DISPLAY:-}"
  printf '[run_vnc_gpu][check] XAUTHORITY=%s\n' "${XAUTHORITY:-}"
  printf '[run_vnc_gpu][check] ROS2SWARM_VGLRUN_DEVICE=%s\n' "${ROS2SWARM_VGLRUN_DEVICE:-}"

  if [[ -z "${DISPLAY:-}" ]]; then
    printf '[run_vnc_gpu][error] DISPLAY is empty\n' >&2
    rc=1
  elif [[ ! -S "${ROS2SWARM_X11_SOCKET_DIR:-/tmp/.X11-unix}/X${display_number}" ]]; then
    printf '[run_vnc_gpu][warn] X11 socket not found: %s\n' \
      "${ROS2SWARM_X11_SOCKET_DIR:-/tmp/.X11-unix}/X${display_number}" >&2
  fi

  if [[ ! -r "${XAUTHORITY:-}" ]]; then
    printf '[run_vnc_gpu][warn] Xauthority file is not readable: %s\n' "${XAUTHORITY:-}" >&2
  fi

  if ! command -v vglrun >/dev/null 2>&1; then
    printf '[run_vnc_gpu][error] vglrun not found; VirtualGL GPU rendering is unavailable\n' >&2
    rc=1
  fi

  if command -v glxinfo >/dev/null 2>&1 && command -v vglrun >/dev/null 2>&1; then
    printf '[run_vnc_gpu][check] glxinfo through VirtualGL:\n'
    if ! timeout 15 vglrun -d "${ROS2SWARM_VGLRUN_DEVICE}" glxinfo -B 2>&1 |
      sed -nE '/OpenGL vendor|OpenGL renderer|OpenGL version/p'; then
      printf '[run_vnc_gpu][error] glxinfo failed through vglrun\n' >&2
      rc=1
    fi
  else
    printf '[run_vnc_gpu][warn] glxinfo not found; skipping OpenGL renderer check\n' >&2
  fi

  if command -v webots >/dev/null 2>&1 && command -v vglrun >/dev/null 2>&1; then
    printf '[run_vnc_gpu][check] Webots through VirtualGL:\n'
    if ! timeout 30 vglrun -d "${ROS2SWARM_VGLRUN_DEVICE}" webots --sysinfo 2>&1 |
      sed -nE '/OpenGL vendor|OpenGL renderer|OpenGL version/p'; then
      printf '[run_vnc_gpu][error] webots --sysinfo failed through vglrun\n' >&2
      rc=1
    fi
  else
    printf '[run_vnc_gpu][warn] webots not found; skipping Webots renderer check\n' >&2
  fi

  return "$rc"
}

_run_vnc_gpu_usage() {
  cat <<'EOF'
作用：
  对齐容器内 TurboVNC/X11 + VirtualGL + GPU 渲染环境。

用法：
  source ./run_vnc_gpu.sh [--quiet]
  ./run_vnc_gpu.sh --check
  ./run_vnc_gpu.sh -- <项目启动命令> [参数...]

环境变量覆盖：
  ROS2SWARM_VNC_DISPLAY     指定 VNC DISPLAY，例如 :1。
  ROS2SWARM_XAUTHORITY      指定 xauth 文件，默认优先 /tmp/.docker.xauth。
  ROS2SWARM_VGLRUN_DEVICE   指定 VirtualGL 3D 设备，默认 /dev/dri/card2。
  ROBOSAR_VGLRUN_DEVICE     RoboSAR 启动脚本使用的 VirtualGL 3D 设备。
  ROS2SWARM_X11_SOCKET_DIR  X11 socket 目录，默认 /tmp/.X11-unix。
EOF
}

_run_vnc_gpu_main() {
  local quiet="${ROS2SWARM_VNC_GPU_QUIET:-0}"
  local check=0
  local print=0
  local command_mode=0
  local command_args=()

  while (($#)); do
    case "$1" in
      --quiet)
        quiet=1
        ;;
      --check)
        check=1
        ;;
      --print)
        print=1
        ;;
      -h|--help)
        _run_vnc_gpu_usage
        return 0
        ;;
      --)
        command_mode=1
        shift
        command_args=("$@")
        break
        ;;
      *)
        printf '[run_vnc_gpu][error] unknown option: %s\n' "$1" >&2
        _run_vnc_gpu_usage >&2
        return 2
        ;;
    esac
    shift
  done

  _run_vnc_gpu_configure

  if [[ "$quiet" != "1" || "$print" == "1" ]]; then
    _run_vnc_gpu_print_env
  fi

  if [[ "$check" == "1" ]]; then
    _run_vnc_gpu_check
    return $?
  fi

  if [[ "$command_mode" == "1" ]]; then
    if ((${#command_args[@]} == 0)); then
      printf '[run_vnc_gpu][error] missing command after --\n' >&2
      return 2
    fi
    "${command_args[@]}"
    return $?
  fi

  if ! _run_vnc_gpu_is_sourced; then
    cat <<'EOF'
[run_vnc_gpu] 环境已在当前子进程中配置。
[run_vnc_gpu] 通常直接运行 ./start_simulation.sh 或 ./start_robosar_webots.sh 即可，它们会自动加载本脚本。
[run_vnc_gpu] 如需让当前交互式终端保留这些 export，请使用：source ./run_vnc_gpu.sh
EOF
  fi
}

if _run_vnc_gpu_is_sourced; then
  _run_vnc_gpu_main "$@"
else
  _run_vnc_gpu_main "$@"
  exit $?
fi
