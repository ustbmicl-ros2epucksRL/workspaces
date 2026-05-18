#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_stage0_response_deform_demo.sh [options] [roslaunch_arg:=value ...]

Starts the ROS1 + Gazebo phase-0 emergency response DEFORM demo:
  roslaunch robosar_emergency_response stage0_response_deform_demo.launch

Options:
  --headless      Set gz_gui:=false and rviz:=false.
  --gz-gui        Set gz_gui:=true.
  --no-gz-gui     Set gz_gui:=false.
  --rviz          Set rviz:=true.
  --no-rviz       Set rviz:=false.
  --vglrun        Force launch through vglrun -d ${ROBOSAR_VGLRUN_DEVICE}.
  --no-vglrun     Disable vglrun.
  --isolated      Use an isolated ROS/Gazebo master. Default.
  --shared-master Reuse the caller's ROS/Gazebo master environment.
  --cleanup-only  Stop leftover phase-0 processes for the configured isolated ports.
  --dry-run       Print the resolved command without executing it.
  -h, --help      Show this help.

Environment:
  WORKSPACE_DIR              Catkin workspace root. Auto-detected when unset.
  ROBOSAR_USE_VGLRUN         auto, 1/true, or 0/false. Default: auto.
  ROBOSAR_VGLRUN_DEVICE      DRM device for vglrun. Default: /dev/dri/card0.
  ROBOSAR_STAGE0_WORLD       Gazebo world. Default: deform_navigation_demo.world.
  ROBOSAR_GZ_GUI             Default gz_gui launch arg. Default: true.
  ROBOSAR_RVIZ               Default rviz launch arg. Default: true.
  TURTLEBOT3_MODEL           TurtleBot3 model. Default: burger.
  ROBOSAR_STAGE0_ISOLATED    1/true or 0/false. Default: true.
  ROBOSAR_STAGE0_ROS_PORT    Isolated ROS master port. Default: 11411.
  ROBOSAR_STAGE0_GAZEBO_PORT Isolated Gazebo master port. Default: 11445.
  ROBOSAR_SKIP_SETUP         Set to 1 for dry-run tests that should not source ROS.

Examples:
  ./src/robosar/robosar_emergency_response/scripts/run_stage0_response_deform_demo.sh
  ./src/robosar/robosar_emergency_response/scripts/run_stage0_response_deform_demo.sh --headless
  ./src/robosar/robosar_emergency_response/scripts/run_stage0_response_deform_demo.sh --vglrun gz_gui:=false rviz:=false
EOF
}

die() {
  echo "[stage0][error] $*" >&2
  exit 1
}

resolve_workspace_dir() {
  if [[ -n "${WORKSPACE_DIR:-}" ]]; then
    cd "$WORKSPACE_DIR" && pwd
    return
  fi

  local dir="$SCRIPT_DIR"
  while [[ "$dir" != "/" ]]; do
    if [[ -f "$dir/devel/setup.bash" && -d "$dir/src" ]]; then
      echo "$dir"
      return
    fi
    dir="$(dirname "$dir")"
  done

  if [[ -f "$PWD/devel/setup.bash" && -d "$PWD/src" ]]; then
    pwd
    return
  fi

  cd "$SCRIPT_DIR/../../../.." && pwd
}

has_roslaunch_arg() {
  local name="$1"
  shift || true

  local arg
  for arg in "$@"; do
    if [[ "$arg" == "${name}:="* ]]; then
      return 0
    fi
  done
  return 1
}

source_ros_env() {
  local workspace_setup="$WORKSPACE_DIR/devel/setup.bash"

  if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
    # shellcheck source=/opt/ros/noetic/setup.bash
    source "/opt/ros/noetic/setup.bash"
  fi

  if [[ ! -f "$workspace_setup" ]]; then
    die "catkin setup file not found: $workspace_setup. Run catkin_make first or set WORKSPACE_DIR."
  fi

  # shellcheck source=/dev/null
  source "$workspace_setup"
}

kill_process_group() {
  local signal="$1"
  local pid="${2:-}"

  if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
    echo "[stage0][cleanup] SIG${signal} process group $pid"
    kill "-$signal" "-$pid" 2>/dev/null || true
  fi
}

kill_pid_list() {
  local signal="$1"
  shift || true

  local pid
  local self_pgid
  self_pgid="$(ps -o pgid= -p "$$" 2>/dev/null | tr -d ' ')"
  for pid in "$@"; do
    local pid_pgid
    pid_pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
    if [[ -n "$self_pgid" && "$pid_pgid" == "$self_pgid" ]]; then
      continue
    fi
    if [[ -n "$pid" && "$pid" != "$$" && "$pid" != "${ROSLAUNCH_PID:-}" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "[stage0][cleanup] SIG${signal} pid $pid ($(ps -p "$pid" -o comm= 2>/dev/null | tr -d ' '))"
      kill "-$signal" "$pid" 2>/dev/null || true
    fi
  done
  # kill_pid_list always succeeds for cleanup callers; processes may exit while scanned.
  return 0
}

pids_with_env_value() {
  local expected="$1"
  local proc env_file pid

  for proc in /proc/[0-9]*; do
    pid="${proc##*/}"
    env_file="$proc/environ"
    [[ -r "$env_file" ]] || continue
    local env_lines
    env_lines="$(tr '\0' '\n' < "$env_file" 2>/dev/null || continue)"
    if grep -Fxq "$expected" <<< "$env_lines"; then
      echo "$pid"
    fi
  done
  # pids_with_env_value always succeeds for cleanup callers; /proc is inherently racy.
  return 0
}

cleanup_stage0_processes() {
  local include_orphans="${1:-0}"
  local pids=()

  kill_process_group "INT" "${ROSLAUNCH_PID:-}"
  sleep 2
  kill_process_group "TERM" "${ROSLAUNCH_PID:-}"
  sleep 1

  if [[ "$include_orphans" == "1" ]]; then
    case "${STAGE0_ISOLATED,,}" in
      1|true|yes|on)
        ;;
      *)
        echo "[stage0][cleanup] shared-master mode: skipping orphan ROS/Gazebo cleanup"
        return
        ;;
    esac

    echo "[stage0][cleanup] targeted ROS master pattern: rosmaster --core -p ${STAGE0_ROS_PORT}"
    mapfile -t pids < <(pgrep -f "rosmaster --core -p ${STAGE0_ROS_PORT}" 2>/dev/null || true)
    kill_pid_list "INT" "${pids[@]}"

    echo "[stage0][cleanup] targeted Gazebo env: GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<unset>} (gzserver/gzclient)"
    if [[ -n "${GAZEBO_MASTER_URI:-}" ]]; then
      mapfile -t pids < <(pids_with_env_value "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}" || true)
      kill_pid_list "INT" "${pids[@]}"
    fi

    echo "[stage0][cleanup] targeted ROS env: ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"
    if [[ -n "${ROS_MASTER_URI:-}" ]]; then
      mapfile -t pids < <(pids_with_env_value "ROS_MASTER_URI=${ROS_MASTER_URI}" || true)
      kill_pid_list "INT" "${pids[@]}"
    fi

    sleep 2
    mapfile -t pids < <(pgrep -f "rosmaster --core -p ${STAGE0_ROS_PORT}" 2>/dev/null || true)
    kill_pid_list "TERM" "${pids[@]}"
    if [[ -n "${GAZEBO_MASTER_URI:-}" ]]; then
      mapfile -t pids < <(pids_with_env_value "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}" || true)
      kill_pid_list "TERM" "${pids[@]}"
    fi
    if [[ -n "${ROS_MASTER_URI:-}" ]]; then
      mapfile -t pids < <(pids_with_env_value "ROS_MASTER_URI=${ROS_MASTER_URI}" || true)
      kill_pid_list "TERM" "${pids[@]}"
    fi
  fi
}

handle_interrupt() {
  echo
  echo "[stage0][cleanup] Ctrl+C received; stopping phase-0 launch tree..."
  cleanup_stage0_processes 1
  exit 130
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(resolve_workspace_dir)"

DRY_RUN=0
CLEANUP_ONLY=0
GZ_GUI="${ROBOSAR_GZ_GUI:-true}"
RVIZ="${ROBOSAR_RVIZ:-true}"
MODEL="${TURTLEBOT3_MODEL:-burger}"
DEFAULT_STAGE0_WORLD="$WORKSPACE_DIR/src/robosar/robosar_emergency_response/worlds/deform_navigation_demo.world"
WORLD_NAME="${ROBOSAR_STAGE0_WORLD:-$DEFAULT_STAGE0_WORLD}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
STAGE0_ISOLATED="${ROBOSAR_STAGE0_ISOLATED:-1}"
STAGE0_ROS_PORT="${ROBOSAR_STAGE0_ROS_PORT:-11411}"
STAGE0_GAZEBO_PORT="${ROBOSAR_STAGE0_GAZEBO_PORT:-11445}"
PASSTHROUGH_ARGS=()

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    --headless)
      GZ_GUI=false
      RVIZ=false
      ;;
    --gz-gui)
      GZ_GUI=true
      ;;
    --no-gz-gui)
      GZ_GUI=false
      ;;
    --rviz)
      RVIZ=true
      ;;
    --no-rviz)
      RVIZ=false
      ;;
    --vglrun)
      USE_VGLRUN=1
      ;;
    --no-vglrun)
      USE_VGLRUN=0
      ;;
    --isolated)
      STAGE0_ISOLATED=1
      ;;
    --shared-master)
      STAGE0_ISOLATED=0
      ;;
    --cleanup-only)
      CLEANUP_ONLY=1
      ;;
    --)
      shift
      PASSTHROUGH_ARGS+=("$@")
      break
      ;;
    *)
      PASSTHROUGH_ARGS+=("$1")
      ;;
  esac
  shift
done

ROSLAUNCH_ARGS=()
if ! has_roslaunch_arg "gz_gui" "${PASSTHROUGH_ARGS[@]}"; then
  ROSLAUNCH_ARGS+=("gz_gui:=$GZ_GUI")
fi
if ! has_roslaunch_arg "rviz" "${PASSTHROUGH_ARGS[@]}"; then
  ROSLAUNCH_ARGS+=("rviz:=$RVIZ")
fi
if ! has_roslaunch_arg "model" "${PASSTHROUGH_ARGS[@]}"; then
  ROSLAUNCH_ARGS+=("model:=$MODEL")
fi
if ! has_roslaunch_arg "world_name" "${PASSTHROUGH_ARGS[@]}"; then
  ROSLAUNCH_ARGS+=("world_name:=$WORLD_NAME")
fi
ROSLAUNCH_ARGS+=("${PASSTHROUGH_ARGS[@]}")

WRAPPER=()
case "${USE_VGLRUN,,}" in
  auto)
    if command -v vglrun >/dev/null 2>&1 && [[ -e "$VGLRUN_DEVICE" ]]; then
      WRAPPER=(vglrun -d "$VGLRUN_DEVICE")
    fi
    ;;
  1|true|yes|on)
    WRAPPER=(vglrun -d "$VGLRUN_DEVICE")
    ;;
  0|false|no|off)
    ;;
  *)
    die "ROBOSAR_USE_VGLRUN must be auto, 1/true, or 0/false; got '$USE_VGLRUN'."
    ;;
esac

case "${STAGE0_ISOLATED,,}" in
  1|true|yes|on)
    export ROS_MASTER_URI="${ROBOSAR_STAGE0_ROS_MASTER_URI:-http://127.0.0.1:${STAGE0_ROS_PORT}}"
    export GAZEBO_MASTER_URI="${ROBOSAR_STAGE0_GAZEBO_MASTER_URI:-http://127.0.0.1:${STAGE0_GAZEBO_PORT}}"
    ;;
  0|false|no|off)
    ;;
  *)
    die "ROBOSAR_STAGE0_ISOLATED must be 1/true or 0/false; got '$STAGE0_ISOLATED'."
    ;;
esac

COMMAND=(
  "${WRAPPER[@]}"
  roslaunch
  robosar_emergency_response
  stage0_response_deform_demo.launch
  "${ROSLAUNCH_ARGS[@]}"
)

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[stage0][dry-run] workspace=$WORKSPACE_DIR"
  echo "[stage0][dry-run] TURTLEBOT3_MODEL=$MODEL"
  echo "[stage0][dry-run] ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>}"
  echo "[stage0][dry-run] GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<caller-default>}"
  if [[ "$CLEANUP_ONLY" == "1" ]]; then
    echo "[stage0][dry-run] cleanup-only"
    echo "[stage0][dry-run] would target: rosmaster --core -p ${STAGE0_ROS_PORT}"
    echo "[stage0][dry-run] would target: GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<caller-default>} (gzserver/gzclient)"
    exit 0
  fi
  printf '[stage0][dry-run] command='
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

trap 'handle_interrupt' INT TERM

if [[ "$CLEANUP_ONLY" == "1" ]]; then
  cleanup_stage0_processes 1
  exit 0
fi

if [[ "${ROBOSAR_SKIP_SETUP:-0}" != "1" ]]; then
  source_ros_env
fi

if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  command -v vglrun >/dev/null 2>&1 || die "vglrun requested but not found in PATH."
  [[ -e "$VGLRUN_DEVICE" ]] || die "vglrun device not found: $VGLRUN_DEVICE"
fi

export TURTLEBOT3_MODEL="$MODEL"
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"

echo "[stage0] workspace=$WORKSPACE_DIR"
echo "[stage0] TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "[stage0] ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>}"
echo "[stage0] GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<caller-default>}"
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  echo "[stage0] using vglrun device=$VGLRUN_DEVICE"
else
  echo "[stage0] vglrun disabled or unavailable"
fi
printf '[stage0] command='
printf '%q ' "${COMMAND[@]}"
printf '\n'

ROSLAUNCH_PID=""
setsid "${COMMAND[@]}" &
ROSLAUNCH_PID=$!

set +e
wait "$ROSLAUNCH_PID"
STATUS=$?
set -e

cleanup_stage0_processes 0
exit "$STATUS"
