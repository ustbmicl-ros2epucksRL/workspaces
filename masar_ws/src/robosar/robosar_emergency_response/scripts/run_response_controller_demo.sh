#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_response_controller_demo.sh [options] {roundflocking|roundflocking-checkpoint|roundflocking-heuristic|transport|transport-checkpoint|transport-heuristic} [launch_arg:=value ...]

Starts a standalone Gazebo controller validation demo:
  roundflocking        Checkpoint roundflocking controller around a fixed target.
  roundflocking-checkpoint Checkpoint roundflocking controller for the roundflocking demo.
  roundflocking-heuristic VMAS heuristic roundflocking controller for the roundflocking demo.
  transport           Checkpoint transport pushing a box to a target zone.
  transport-checkpoint Checkpoint transport controller for the transport demo.
  transport-heuristic Heuristic transport controller for the transport demo.

Options:
  --headless   Set gz_gui:=false and rviz:=false.
  --gz-gui     Set gz_gui:=true.
  --no-gz-gui  Set gz_gui:=false.
  --rviz       Set rviz:=true.
  --no-rviz    Set rviz:=false.
  --collision-debug    Show collision-envelope debug markers in RViz.
  --no-collision-debug Disable collision-envelope debug markers.
  --vglrun     Force launch through vglrun -d ${ROBOSAR_VGLRUN_DEVICE}.
  --no-vglrun  Disable vglrun.
  --isolated   Use isolated ROS/Gazebo masters. Default.
  --shared-master Reuse the caller's ROS/Gazebo masters.
  --dry-run    Print the resolved command without executing it.
  -h, --help   Show this help.
EOF
}

die() {
  echo "[controller-demo][error] $*" >&2
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

  cd "$SCRIPT_DIR/../../../.." && pwd
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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(resolve_workspace_dir)"

DRY_RUN=0
GZ_GUI="${ROBOSAR_GZ_GUI:-true}"
RVIZ="${ROBOSAR_RVIZ:-true}"
MODEL="${TURTLEBOT3_MODEL:-burger}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
DEMO_ISOLATED="${ROBOSAR_CONTROLLER_DEMO_ISOLATED:-1}"
DEMO_ROS_PORT="${ROBOSAR_CONTROLLER_DEMO_ROS_PORT:-11611}"
DEMO_GAZEBO_PORT="${ROBOSAR_CONTROLLER_DEMO_GAZEBO_PORT:-11645}"
COLLISION_DEBUG="${ROBOSAR_COLLISION_DEBUG:-auto}"
TASK=""
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
    --collision-debug)
      COLLISION_DEBUG=true
      ;;
    --no-collision-debug)
      COLLISION_DEBUG=false
      ;;
    --vglrun)
      USE_VGLRUN=1
      ;;
    --no-vglrun)
      USE_VGLRUN=0
      ;;
    --isolated)
      DEMO_ISOLATED=1
      ;;
    --shared-master)
      DEMO_ISOLATED=0
      ;;
    --)
      shift
      PASSTHROUGH_ARGS+=("$@")
      break
      ;;
    roundflocking|roundflocking-checkpoint|roundflocking-heuristic|transport|transport-checkpoint|transport-heuristic)
      [[ -z "$TASK" ]] || die "only one demo task can be selected"
      TASK="$1"
      ;;
    *:=*)
      PASSTHROUGH_ARGS+=("$1")
      ;;
    *)
      die "unknown option or demo task: $1"
      ;;
  esac
  shift
done

TASK="${TASK:-roundflocking}"
CONTROLLER=""
case "$TASK" in
  roundflocking)
    LAUNCH_FILE="roundflocking_controller_demo.launch"
    CONTROLLER="checkpoint"
    ;;
  roundflocking-heuristic)
    LAUNCH_FILE="roundflocking_controller_demo.launch"
    CONTROLLER="heuristic"
    ;;
  roundflocking-checkpoint)
    LAUNCH_FILE="roundflocking_controller_demo.launch"
    CONTROLLER="checkpoint"
    ;;
  transport)
    LAUNCH_FILE="transport_controller_demo.launch"
    CONTROLLER="checkpoint"
    [[ "$COLLISION_DEBUG" == "auto" ]] && COLLISION_DEBUG=true
    ;;
  transport-checkpoint)
    LAUNCH_FILE="transport_controller_demo.launch"
    CONTROLLER="checkpoint"
    [[ "$COLLISION_DEBUG" == "auto" ]] && COLLISION_DEBUG=true
    ;;
  transport-heuristic)
    LAUNCH_FILE="transport_controller_demo.launch"
    CONTROLLER="heuristic"
    [[ "$COLLISION_DEBUG" == "auto" ]] && COLLISION_DEBUG=true
    ;;
  *)
    die "unsupported demo task: $TASK"
    ;;
esac
[[ "$COLLISION_DEBUG" == "auto" ]] && COLLISION_DEBUG=false

ROSLAUNCH_ARGS=()
has_roslaunch_arg "gz_gui" "${PASSTHROUGH_ARGS[@]}" || ROSLAUNCH_ARGS+=("gz_gui:=$GZ_GUI")
has_roslaunch_arg "rviz" "${PASSTHROUGH_ARGS[@]}" || ROSLAUNCH_ARGS+=("rviz:=$RVIZ")
has_roslaunch_arg "model" "${PASSTHROUGH_ARGS[@]}" || ROSLAUNCH_ARGS+=("model:=$MODEL")
if [[ -n "$CONTROLLER" ]] && ! has_roslaunch_arg "controller" "${PASSTHROUGH_ARGS[@]}"; then
  ROSLAUNCH_ARGS+=("controller:=$CONTROLLER")
fi
if ! has_roslaunch_arg "collision_debug" "${PASSTHROUGH_ARGS[@]}"; then
  ROSLAUNCH_ARGS+=("collision_debug:=$COLLISION_DEBUG")
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

case "${DEMO_ISOLATED,,}" in
  1|true|yes|on)
    export ROS_MASTER_URI="${ROBOSAR_CONTROLLER_DEMO_ROS_MASTER_URI:-http://127.0.0.1:${DEMO_ROS_PORT}}"
    export GAZEBO_MASTER_URI="${ROBOSAR_CONTROLLER_DEMO_GAZEBO_MASTER_URI:-http://127.0.0.1:${DEMO_GAZEBO_PORT}}"
    ;;
  0|false|no|off)
    ;;
  *)
    die "ROBOSAR_CONTROLLER_DEMO_ISOLATED must be 1/true or 0/false; got '$DEMO_ISOLATED'."
    ;;
esac

COMMAND=(
  "${WRAPPER[@]}"
  roslaunch
  robosar_emergency_response
  "$LAUNCH_FILE"
  "${ROSLAUNCH_ARGS[@]}"
)

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[controller-demo][dry-run] workspace=$WORKSPACE_DIR"
  echo "[controller-demo][dry-run] task=$TASK"
  echo "[controller-demo][dry-run] TURTLEBOT3_MODEL=$MODEL"
  echo "[controller-demo][dry-run] ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>}"
  echo "[controller-demo][dry-run] GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<caller-default>}"
  echo "[controller-demo][dry-run] GAZEBO_MODEL_DATABASE_URI="
  printf '[controller-demo][dry-run] command='
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

if [[ "${ROBOSAR_SKIP_SETUP:-0}" != "1" ]]; then
  source_ros_env
fi

export GAZEBO_MODEL_DATABASE_URI=""

if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  command -v vglrun >/dev/null 2>&1 || die "vglrun requested but not found in PATH."
  [[ -e "$VGLRUN_DEVICE" ]] || die "vglrun device not found: $VGLRUN_DEVICE"
fi

export TURTLEBOT3_MODEL="$MODEL"
export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"

echo "[controller-demo] workspace=$WORKSPACE_DIR"
echo "[controller-demo] task=$TASK"
echo "[controller-demo] TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "[controller-demo] ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>}"
echo "[controller-demo] GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<caller-default>}"
echo "[controller-demo] GAZEBO_MODEL_DATABASE_URI=${GAZEBO_MODEL_DATABASE_URI:-<unset>}"
printf '[controller-demo] command='
printf '%q ' "${COMMAND[@]}"
printf '\n'

exec "${COMMAND[@]}"
