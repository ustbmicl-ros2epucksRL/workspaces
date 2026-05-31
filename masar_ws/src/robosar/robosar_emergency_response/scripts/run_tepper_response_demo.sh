#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_tepper_response_demo.sh [options] [response_overlay_arg:=value ...]

Starts the ROS1 + Gazebo Tepper emergency-response demo in the same order as
the original RoboSAR Gazebo launcher, then adds the response-group overlay.

Options:
  --headless   Set gz_gui:=false and rviz:=false for the Tepper simulator.
  --gz-gui     Set gz_gui:=true.
  --no-gz-gui  Set gz_gui:=false.
  --rviz       Set rviz:=true.
  --no-rviz    Set rviz:=false.
  --vglrun     Run the Tepper simulator launch through vglrun -d ${ROBOSAR_VGLRUN_DEVICE} (DRM device or X display).
  --no-vglrun  Disable vglrun.
  --dry-run    Print the startup sequence without executing it.
  -h, --help   Show this help.

Use the GUI START/STOP/HOME buttons to control the search and response loops.
EOF
}

die() {
  echo "[tepper-response][error] $*" >&2
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
  local caller_ros_master_uri="${ROS_MASTER_URI:-}"
  local caller_gazebo_master_uri="${GAZEBO_MASTER_URI:-}"

  [[ -f "/opt/ros/noetic/setup.bash" ]] && source "/opt/ros/noetic/setup.bash"

  if [[ -f "$WORKSPACE_SETUP" ]]; then
    source "$WORKSPACE_SETUP"
  elif [[ -f "$LEGACY_WORKSPACE_SETUP" ]]; then
    source "$LEGACY_WORKSPACE_SETUP"
  else
    die "catkin setup file not found: $WORKSPACE_SETUP"
  fi

  if [[ -n "$caller_ros_master_uri" ]]; then
    export ROS_MASTER_URI="$caller_ros_master_uri"
  fi
  if [[ -n "$caller_gazebo_master_uri" ]]; then
    export GAZEBO_MASTER_URI="$caller_gazebo_master_uri"
  fi
}

has_roslaunch_arg() {
  local name="$1"
  shift || true
  local arg
  for arg in "$@"; do
    [[ "$arg" == "${name}:="* ]] && return 0
  done
  return 1
}

quote_args() {
  printf "%q " "$@"
}

vglrun_target_is_available() {
  local target="$1"

  if [[ -z "$target" ]]; then
    return 1
  fi

  if [[ "$target" == /* ]]; then
    [[ -e "$target" ]]
    return
  fi

  # VirtualGL also accepts X display targets such as :0 or host:0.
  return 0
}

ros_master_port() {
  local uri="${ROS_MASTER_URI:-http://127.0.0.1:11311}"
  local host_port="${uri#*://}"
  host_port="${host_port%%/*}"
  local port="${host_port##*:}"

  if [[ "$port" =~ ^[0-9]+$ ]]; then
    echo "$port"
  else
    echo "11311"
  fi
}

start_process_group() {
  local label="$1"
  local cmd="$2"
  local wait_sec="${3:-1}"

  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] ${label}: ${cmd}"
    return
  fi

  setsid bash -lc "
    set -e
    WORKSPACE_SETUP=$(printf "%q" "$WORKSPACE_SETUP")
    LEGACY_WORKSPACE_SETUP=$(printf "%q" "$LEGACY_WORKSPACE_SETUP")
    source_ros_env() {
      caller_ros_master_uri=\"\${ROS_MASTER_URI:-}\"
      caller_gazebo_master_uri=\"\${GAZEBO_MASTER_URI:-}\"
      [ -f /opt/ros/noetic/setup.bash ] && . /opt/ros/noetic/setup.bash
      if [ -f \"\$WORKSPACE_SETUP\" ]; then
        . \"\$WORKSPACE_SETUP\"
      elif [ -f \"\$LEGACY_WORKSPACE_SETUP\" ]; then
        . \"\$LEGACY_WORKSPACE_SETUP\"
      else
        echo '[tepper-response][error] no catkin setup found' >&2
        exit 1
      fi
      if [ -n \"\$caller_ros_master_uri\" ]; then
        export ROS_MASTER_URI=\"\$caller_ros_master_uri\"
      fi
      if [ -n \"\$caller_gazebo_master_uri\" ]; then
        export GAZEBO_MASTER_URI=\"\$caller_gazebo_master_uri\"
      fi
    }
    source_ros_env
    ${cmd}
  " &

  local pid=$!
  PIDS+=("$pid")
  echo "[tepper-response][start] ($pid) ${label}: ${cmd}"
  sleep "$wait_sec"
}

wait_for_roscore() {
  local i
  for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then
      echo "[tepper-response] roscore is ready"
      return
    fi
    sleep 1
  done
  die "roscore did not become ready within 30s"
}

ensure_roscore() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] roscore: roscore"
    return
  fi

  if rostopic list >/dev/null 2>&1; then
    echo "[tepper-response] using existing roscore"
    return
  fi

  echo "[tepper-response][start] launching roscore..."
  setsid roscore -p "$(ros_master_port)" &
  local pid=$!
  PIDS+=("$pid")
  wait_for_roscore
}

cleanup_gui_processes() {
  local signal="$1"
  local pid

  if [[ -n "${GUI_SCRIPT:-}" ]]; then
    while read -r pid; do
      [[ -z "$pid" || "$pid" == "$$" ]] && continue
      echo "  -> SIG${signal} GUI pid $pid"
      kill "-${signal}" "$pid" 2>/dev/null || true
    done < <(pgrep -f "$GUI_SCRIPT" 2>/dev/null || true)
  fi

  case "$signal" in
    INT) pkill -INT -x rviz 2>/dev/null || true ;;
    TERM) pkill -TERM -x rviz 2>/dev/null || true ;;
    KILL) pkill -KILL -x rviz 2>/dev/null || true ;;
  esac
}

cleanup() {
  local pid
  echo
  echo "[tepper-response][cleanup] stopping launched processes..."

  for pid in "${PIDS[@]}"; do
    echo "  -> SIGINT to PGID $pid"
    kill -INT -"${pid}" 2>/dev/null || true
  done
  cleanup_gui_processes "INT"
  sleep 2

  for pid in "${PIDS[@]}"; do
    echo "  -> SIGTERM to PGID $pid"
    kill -TERM -"${pid}" 2>/dev/null || true
  done
  cleanup_gui_processes "TERM"
  sleep 2

  # Gazebo occasionally ignores process-group shutdown; keep the same simple
  # targeted fallback style used by the original RoboSAR Gazebo launcher.
  pkill -INT -x gzserver 2>/dev/null || true
  pkill -INT -x gzclient 2>/dev/null || true
  sleep 1
  pkill -TERM -x gzserver 2>/dev/null || true
  pkill -TERM -x gzclient 2>/dev/null || true
  sleep 1

  for pid in "${PIDS[@]}"; do
    echo "  -> SIGKILL to PGID $pid"
    kill -KILL -"${pid}" 2>/dev/null || true
  done

  pkill -KILL -x gzserver 2>/dev/null || true
  pkill -KILL -x gzclient 2>/dev/null || true
  cleanup_gui_processes "KILL"
}

handle_interrupt() {
  cleanup
  exit 130
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(resolve_workspace_dir)"
WORKSPACE_SETUP="$WORKSPACE_DIR/devel/setup.bash"
LEGACY_WORKSPACE_SETUP="$HOME/catkin_ws2/devel/setup.bash"
GUI_SCRIPT="$WORKSPACE_DIR/src/robosar/robosar_gui/src/robosar_gui.py"

DRY_RUN=0
GZ_GUI="${ROBOSAR_GZ_GUI:-true}"
RVIZ="${ROBOSAR_RVIZ:-true}"
MODEL="${TURTLEBOT3_MODEL:-burger}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
SLAM_NODE_TYPE="${SLAM_NODE_TYPE:-sync_slam_toolbox_node}"
PASSTHROUGH_ARGS=()
PIDS=()

while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --dry-run) DRY_RUN=1 ;;
    --headless) GZ_GUI=false; RVIZ=false ;;
    --gz-gui) GZ_GUI=true ;;
    --no-gz-gui) GZ_GUI=false ;;
    --rviz) RVIZ=true ;;
    --no-rviz) RVIZ=false ;;
    --vglrun) USE_VGLRUN=1 ;;
    --no-vglrun) USE_VGLRUN=0 ;;
    --) shift; PASSTHROUGH_ARGS+=("$@"); break ;;
    *) PASSTHROUGH_ARGS+=("$1") ;;
  esac
  shift
done

WRAPPER=()
case "${USE_VGLRUN,,}" in
  auto)
    if command -v vglrun >/dev/null 2>&1 && vglrun_target_is_available "$VGLRUN_DEVICE"; then
      WRAPPER=(vglrun -d "$VGLRUN_DEVICE")
    fi
    ;;
  1|true|yes|on) WRAPPER=(vglrun -d "$VGLRUN_DEVICE") ;;
  0|false|no|off) ;;
  *) die "ROBOSAR_USE_VGLRUN must be auto, 1/true, or 0/false; got '$USE_VGLRUN'." ;;
esac

OVERLAY_ARGS=()
has_roslaunch_arg "model" "${PASSTHROUGH_ARGS[@]}" || OVERLAY_ARGS+=("model:=$MODEL")
OVERLAY_ARGS+=("${PASSTHROUGH_ARGS[@]}")

VGLRUN_CMD=""
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  VGLRUN_CMD="$(quote_args "${WRAPPER[@]}")"
fi

GAZEBO_CMD="GAZEBO_MODEL_DATABASE_URI='' ${VGLRUN_CMD}roslaunch robosar_sim_bringup robosar_sim_tepper.launch gz_gui:=${GZ_GUI} rviz:=${RVIZ}"
OVERLAY_CMD="roslaunch robosar_emergency_response tepper_response_demo.launch $(quote_args "${OVERLAY_ARGS[@]}")"
TRAFFIC_CMD="rosrun robosar_controller robosar_traffic_controller_node"

echo "[tepper-response] workspace=$WORKSPACE_DIR"
echo "[tepper-response] TURTLEBOT3_MODEL=$MODEL"
echo "[tepper-response] gz_gui=$GZ_GUI rviz=$RVIZ"

if [[ "$DRY_RUN" == "0" ]]; then
  source_ros_env
  export TURTLEBOT3_MODEL="$MODEL"
  export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"
  export MALLOC_ARENA_MAX="${MALLOC_ARENA_MAX:-${ROBOSAR_MALLOC_ARENA_MAX:-2}}"
  export MALLOC_TRIM_THRESHOLD_="${MALLOC_TRIM_THRESHOLD_:-${ROBOSAR_MALLOC_TRIM_THRESHOLD:-131072}}"
  export MALLOC_MMAP_THRESHOLD_="${MALLOC_MMAP_THRESHOLD_:-${ROBOSAR_MALLOC_MMAP_THRESHOLD:-131072}}"
  echo "[tepper-response] MALLOC_ARENA_MAX=$MALLOC_ARENA_MAX MALLOC_TRIM_THRESHOLD_=$MALLOC_TRIM_THRESHOLD_ MALLOC_MMAP_THRESHOLD_=$MALLOC_MMAP_THRESHOLD_"

  if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
    command -v vglrun >/dev/null 2>&1 || die "vglrun requested but not found in PATH."
    vglrun_target_is_available "$VGLRUN_DEVICE" || die "vglrun target not found: $VGLRUN_DEVICE"
  fi

  trap 'handle_interrupt' INT TERM
fi

ensure_roscore

if [[ "$DRY_RUN" == "0" ]]; then
  rosparam set /use_sim_time true
else
  echo "[tepper-response][dry-run] rosparam: rosparam set /use_sim_time true"
fi

start_process_group "agent_bringup" "roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch" 1
start_process_group "tepper_sim" "$GAZEBO_CMD" 1
start_process_group "slam" "roslaunch slam_toolbox robosar_mapping_sim_g.launch slam_node_type:=${SLAM_NODE_TYPE}" 1
start_process_group "task_generator" "roslaunch robosar_task_generator task_generator.launch" 1
start_process_group "mission_executive" "roslaunch robosar_navigation mission_executive.launch" 1
start_process_group "traffic_controller" "$TRAFFIC_CMD" 1
start_process_group "mission_commander" "rosrun robosar_task_allocator mission_commander.py -s=true -t='high'" 1
start_process_group "gui" "python3 $(printf "%q" "$GUI_SCRIPT")" 1
start_process_group "response_overlay" "$OVERLAY_CMD" 1

if [[ "$DRY_RUN" == "1" ]]; then
  exit 0
fi

wait
