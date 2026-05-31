#!/usr/bin/env bash
# Configure the container after the host has rebooted so GUI windows are shown
# in TurboVNC while OpenGL rendering is performed by the NVIDIA-backed X server.
#
# Host/container alignment:
#   1. On the host, identify the TurboVNC display. The TurboVNC window title
#      usually shows it, for example "hostname:1"; the matching socket in the
#      container is /tmp/.X11-unix/X1. Override with ROBOSAR_VNC_DISPLAY=:2 if
#      your VNC session moved to :2 after a reboot.
#   2. Keep the GPU X server as the VirtualGL render target. On this machine it
#      is :0; verify with `DISPLAY=:0 glxinfo -B` on the host/container.
#      Select a GPU/X screen by setting ROBOSAR_VGLRUN_DEVICE directly, for
#      example :0.0, :0.1, :0.2, :0.3, or /dev/dri/card0.
#   3. /tmp/.docker.xauth must contain the cookie for the VNC display and be
#      mounted into the container as XAUTHORITY. If the host was rebooted and
#      VNC was restarted, regenerate the host-side xauth file before starting
#      the container, or refresh the mounted file.
#
# Usage:
#   source ./run_vnc_gpu.sh
#   source ./run_vnc_gpu.sh --env-only
#   ./run_vnc_gpu.sh ./launch_robosar_g.sh --vglrun
#   ./run_vnc_gpu.sh ./run_response_deform_demo.sh --vglrun
#   ./run_vnc_gpu.sh --check
#
# With no command and when not sourced, this script opens a configured
# interactive shell. Run normal project scripts from that shell.

is_sourced() {
  [[ "${BASH_SOURCE[0]}" != "$0" ]]
}

if ! is_sourced; then
  set -euo pipefail
fi

detect_vnc_display() {
  local candidate="${ROBOSAR_VNC_DISPLAY:-${DISPLAY:-:1}}"
  local socket

  if [[ -n "${ROBOSAR_VNC_DISPLAY+x}" ]]; then
    echo "$candidate"
    return 0
  fi

  if [[ "$candidate" =~ ^:([0-9]+)(\.|$) ]]; then
    socket="/tmp/.X11-unix/X${BASH_REMATCH[1]}"
    if [[ "$candidate" != ":0" && -S "$socket" ]]; then
      echo "$candidate"
      return 0
    fi
  fi

  if [[ "$candidate" == ":0" || "$candidate" =~ ^:[0-9]+(\.|$) ]]; then
    for socket in /tmp/.X11-unix/X[1-9]*; do
      [[ -S "$socket" ]] || continue
      candidate=":${socket##*X}"
      break
    done
  fi

  echo "$candidate"
}

configure_vnc_gpu_env() {
  local vnc_display
  vnc_display="$(detect_vnc_display)"

  export DISPLAY="$vnc_display"
  export XAUTHORITY="${XAUTHORITY:-/tmp/.docker.xauth}"
  export ROBOSAR_USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-1}"
  export ROBOSAR_VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-:0}"
  export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
  unset LIBGL_ALWAYS_INDIRECT
}

print_status() {
  echo "[vnc-gpu] DISPLAY=$DISPLAY"
  echo "[vnc-gpu] XAUTHORITY=$XAUTHORITY"
  echo "[vnc-gpu] ROBOSAR_USE_VGLRUN=$ROBOSAR_USE_VGLRUN"
  echo "[vnc-gpu] ROBOSAR_VGLRUN_DEVICE=$ROBOSAR_VGLRUN_DEVICE"
}

run_check() {
  print_status

  if [[ ! -S "/tmp/.X11-unix/X${DISPLAY#:}" ]]; then
    echo "[vnc-gpu][warn] X socket not found for DISPLAY=$DISPLAY"
  fi

  if [[ ! -r "$XAUTHORITY" ]]; then
    echo "[vnc-gpu][warn] XAUTHORITY is not readable: $XAUTHORITY"
  fi

  command -v glxinfo >/dev/null 2>&1 || {
    echo "[vnc-gpu][warn] glxinfo not found"
    return 0
  }

  if command -v nvidia-smi >/dev/null 2>&1; then
    echo "[vnc-gpu] visible NVIDIA GPUs:"
    nvidia-smi --query-gpu=index,name,pci.bus_id,memory.total --format=csv,noheader 2>/dev/null || true
  fi

  echo "[vnc-gpu] VNC display renderer:"
  glxinfo -B 2>&1 | grep -E 'name of display|OpenGL vendor|OpenGL renderer|Accelerated' || true

  echo "[vnc-gpu] VNC display + VirtualGL renderer:"
  vglrun -d "$ROBOSAR_VGLRUN_DEVICE" glxinfo -B 2>&1 \
    | grep -E 'name of display|OpenGL vendor|OpenGL renderer|Accelerated|VGL' || true
}

usage() {
  cat <<'EOF'
Usage:
  source ./run_vnc_gpu.sh
  ROBOSAR_VGLRUN_DEVICE=:0.2 ./run_tepper_response_demo.sh
  ROBOSAR_VNC_DISPLAY=:2 ROBOSAR_VGLRUN_DEVICE=:0.1 ./launch_robosar_g.sh
  ./run_vnc_gpu.sh ./launch_robosar_g.sh --vglrun
  ./run_vnc_gpu.sh ./run_response_deform_demo.sh --vglrun
  ./run_vnc_gpu.sh --check

What it does:
  - sets DISPLAY to the TurboVNC X server, usually :1 after reboot
  - keeps XAUTHORITY on /tmp/.docker.xauth
  - points VirtualGL at ROBOSAR_VGLRUN_DEVICE, usually :0
  - unsets LIBGL_ALWAYS_INDIRECT and enables QT_X11_NO_MITSHM=1
EOF
}

configure_vnc_gpu_env

case "${1:-}" in
  --env-only)
    if [[ "${ROBOSAR_VNC_GPU_QUIET:-1}" != "1" ]]; then
      print_status
    fi
    if is_sourced; then
      return 0
    fi
    exit 0
    ;;
  -h|--help)
    usage
    if is_sourced; then
      return 0
    fi
    exit 0
    ;;
  --check)
    run_check
    if is_sourced; then
      return 0
    fi
    exit 0
    ;;
esac

if [[ "${ROBOSAR_VNC_GPU_QUIET:-0}" != "1" ]]; then
  print_status
fi

if is_sourced; then
  if [[ "${ROBOSAR_VNC_GPU_QUIET:-0}" != "1" ]]; then
    echo "[vnc-gpu] current shell configured; run project scripts normally, e.g. ./launch_robosar_g.sh"
  fi
  return 0
fi

if (($# == 0)); then
  echo "[vnc-gpu] opening configured shell; run project scripts normally, then exit to return."
  exec "${SHELL:-/bin/bash}" -i
fi

exec "$@"
