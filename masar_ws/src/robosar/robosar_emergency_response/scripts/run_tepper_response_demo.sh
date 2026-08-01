#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  run_tepper_response_demo.sh [options] [response_overlay_arg:=value ...]

Starts the ROS1 + Gazebo Tepper emergency-response demo in the same order as
the original RoboSAR Gazebo launcher, then adds the response-group overlay.

Options:
  --headless   Set gz_gui:=false, rviz:=false, and gui:=false for automated runs.
  --gz-gui     Set gz_gui:=true.
  --no-gz-gui  Set gz_gui:=false.
  --rviz       Set rviz:=true.
  --no-rviz    Set rviz:=false.
  --gui        Start the RoboSAR Qt GUI.
  --no-gui     Do not start the RoboSAR Qt GUI.
  --vglrun     Run the Tepper simulator launch through vglrun -d ${ROBOSAR_VGLRUN_DEVICE} (DRM device or X display).
  --no-vglrun  Disable vglrun.
  --shared-master Reuse an externally managed ROS graph without cleaning it; rejects an existing /clock publisher.
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
  (($#)) || return 0
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

configure_master_mode() {
  case "${MASTER_MODE,,}" in
    clean|shared)
      export ROS_MASTER_URI="${ROBOSAR_TEPPER_ROS_MASTER_URI:-${ROS_MASTER_URI:-http://127.0.0.1:11311}}"
      export GAZEBO_MASTER_URI="${ROBOSAR_TEPPER_GAZEBO_MASTER_URI:-${GAZEBO_MASTER_URI:-http://127.0.0.1:11345}}"
      ;;
    *)
      die "ROBOSAR_TEPPER_MASTER_MODE must be clean or shared; got '$MASTER_MODE'."
      ;;
  esac
}

platform_mode_enabled() {
  case "${PLATFORM_MODE,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
}

start_process_group() {
  local label="$1"
  local cmd="$2"
  local wait_sec="${3:-1}"

  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] ${label}: ${cmd}"
    return
  fi

  local launcher=()
  if ! platform_mode_enabled; then
    launcher=(setsid)
  fi

  "${launcher[@]}" bash -lc "
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

clock_publishers() {
  rostopic info /clock 2>/dev/null | awk '
    /^Publishers:/ { in_publishers=1; next }
    /^Subscribers:/ { in_publishers=0 }
    in_publishers && $1 == "*" { print $2 }
  '
}

require_unclaimed_clock() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] clock_preflight: require no existing /clock publisher"
    return
  fi

  local publishers
  publishers="$(clock_publishers || true)"
  if [[ -n "$publishers" ]]; then
    die "existing /clock publisher(s) detected on ${ROS_MASTER_URI}: ${publishers//$'\n'/, }. Stop the competing simulator, or use the default clean mode before starting an external bridge."
  fi
  echo "[tepper-response] /clock is unclaimed before Gazebo startup"
}

managed_ros_nodes() {
  rosnode list 2>/dev/null | awk '$0 != "/rosout"'
}

prepare_clean_master() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] master_preflight: stop existing ROS nodes except /rosout, remove stale registrations and parameters"
    require_unclaimed_clock
    return
  fi

  local node remaining i
  local -a nodes=()
  mapfile -t nodes < <(managed_ros_nodes || true)

  if ((${#nodes[@]})); then
    echo "[tepper-response][preflight] cleaning existing ROS graph on ${ROS_MASTER_URI}"
    for node in "${nodes[@]}"; do
      echo "  -> scheduled shutdown: $node"
    done
    # One bounded rosnode invocation keeps cleanup time independent of the
    # number of registered robots. A later graph check is the source of truth.
    timeout 15 rosnode kill "${nodes[@]}" >/dev/null 2>&1 || true
  else
    echo "[tepper-response][preflight] ROS graph has no previous application nodes"
  fi

  # Remove unreachable registrations left by processes that did not unregister.
  timeout 10 rosnode cleanup <<<"y" >/dev/null 2>&1 || true

  remaining=""
  for i in $(seq 1 60); do
    remaining="$(managed_ros_nodes || true)"
    [[ -z "$remaining" ]] && break
    sleep 0.25
  done
  if [[ -n "$remaining" ]]; then
    die "could not create a clean ROS graph; nodes still registered: ${remaining//$'\n'/, }. Stop their supervising launcher and retry."
  fi

  # A top-level full-system launcher owns its parameter namespace. Removing the
  # old tree prevents stale launch/run_id/use_sim_time values leaking into a run.
  rosparam delete / >/dev/null 2>&1 || true
  require_unclaimed_clock
  echo "[tepper-response][preflight] clean ROS graph is ready on ${ROS_MASTER_URI}"
}

prepare_master() {
  case "${MASTER_MODE,,}" in
    clean) prepare_clean_master ;;
    shared) require_unclaimed_clock ;;
  esac
}

wait_for_gazebo_clock() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] gazebo_clock: require /gazebo as the sole /clock publisher"
    return
  fi

  local i publishers publisher_count
  for i in $(seq 1 60); do
    publishers="$(clock_publishers || true)"
    publisher_count="$(grep -c . <<<"$publishers" || true)"
    if [[ "$publisher_count" == "1" && "$publishers" == "/gazebo" ]] \
        && timeout 2 rostopic echo -n 1 /clock >/dev/null 2>&1; then
      echo "[tepper-response] Gazebo is the sole live /clock publisher"
      return
    fi
    sleep 1
  done
  die "Gazebo clock validation failed; expected sole publisher /gazebo, got: ${publishers:-none}"
}

wait_for_response_gate_closed() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[tepper-response][dry-run] response_gate: wait for latched /response_group/motion_enabled=false"
    return
  fi

  local i output
  for i in $(seq 1 30); do
    output="$(timeout 2 rostopic echo -n 1 /response_group/motion_enabled 2>/dev/null || true)"
    if grep -Eqi 'data:[[:space:]]*(false|0)' <<<"$output"; then
      echo "[tepper-response] response motion gate is closed"
      return
    fi
    sleep 1
  done
  die "response motion gate did not publish its default closed state within 30s"
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
  local launcher=()
  if ! platform_mode_enabled; then
    launcher=(setsid)
  fi
  "${launcher[@]}" roscore -p "$(ros_master_port)" &
  local pid=$!
  PIDS+=("$pid")
  wait_for_roscore
}

cleanup() {
  local pid
  if [[ "$CLEANUP_DONE" == "1" ]]; then
    return
  fi
  CLEANUP_DONE=1

  echo
  echo "[tepper-response][cleanup] stopping launched processes..."
  if platform_mode_enabled; then
    echo "  -> platform mode: Runtime Agent owns and signals the complete process group"
    return
  fi
  if ((${#PIDS[@]} == 0)); then
    echo "  -> no launched process groups"
    return
  fi

  for pid in "${PIDS[@]}"; do
    echo "  -> SIGINT to PGID $pid"
    kill -INT -"${pid}" 2>/dev/null || true
  done
  sleep 2

  for pid in "${PIDS[@]}"; do
    echo "  -> SIGTERM to PGID $pid"
    kill -TERM -"${pid}" 2>/dev/null || true
  done
  sleep 2

  for pid in "${PIDS[@]}"; do
    echo "  -> SIGKILL to PGID $pid"
    kill -KILL -"${pid}" 2>/dev/null || true
  done
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
PLATFORM_MODE="${MASAR_PLATFORM_MODE:-0}"
GZ_GUI="${ROBOSAR_GZ_GUI:-true}"
RVIZ="${ROBOSAR_RVIZ:-true}"
GUI="${ROBOSAR_GUI:-true}"
MODEL="${TURTLEBOT3_MODEL:-burger}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
SLAM_NODE_TYPE="${SLAM_NODE_TYPE:-sync_slam_toolbox_node}"
MASTER_MODE="${ROBOSAR_TEPPER_MASTER_MODE:-clean}"
PASSTHROUGH_ARGS=()
SIM_PASSTHROUGH_ARGS=()
OVERLAY_PASSTHROUGH_ARGS=()
PIDS=()
CLEANUP_DONE=0

while (($#)); do
  case "$1" in
    -h|--help) usage; exit 0 ;;
    --dry-run) DRY_RUN=1 ;;
    --headless) GZ_GUI=false; RVIZ=false; GUI=false ;;
    --gz-gui) GZ_GUI=true ;;
    --no-gz-gui) GZ_GUI=false ;;
    --rviz) RVIZ=true ;;
    --no-rviz) RVIZ=false ;;
    --gui) GUI=true ;;
    --no-gui) GUI=false ;;
    --vglrun) USE_VGLRUN=1 ;;
    --no-vglrun) USE_VGLRUN=0 ;;
    --shared-master) MASTER_MODE=shared ;;
    --) shift; PASSTHROUGH_ARGS+=("$@"); break ;;
    --*) die "unknown option: $1" ;;
    *) PASSTHROUGH_ARGS+=("$1") ;;
  esac
  shift
done

configure_master_mode

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

for arg in "${PASSTHROUGH_ARGS[@]}"; do
  case "$arg" in
    spawn_search_robots:=*|search_robot_names:=*) SIM_PASSTHROUGH_ARGS+=("$arg") ;;
    *) OVERLAY_PASSTHROUGH_ARGS+=("$arg") ;;
  esac
done

SIM_ARGS=()
SIM_ARGS+=("${SIM_PASSTHROUGH_ARGS[@]}")

OVERLAY_ARGS=()
has_roslaunch_arg "model" "${OVERLAY_PASSTHROUGH_ARGS[@]}" || OVERLAY_ARGS+=("model:=$MODEL")
OVERLAY_ARGS+=("${OVERLAY_PASSTHROUGH_ARGS[@]}")

VGLRUN_CMD=""
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  VGLRUN_CMD="$(quote_args "${WRAPPER[@]}")"
fi

GAZEBO_CMD="GAZEBO_MODEL_DATABASE_URI='' ${VGLRUN_CMD}roslaunch robosar_sim_bringup robosar_sim_tepper.launch gz_gui:=${GZ_GUI} rviz:=${RVIZ} $(quote_args "${SIM_ARGS[@]}")"
OVERLAY_CMD="roslaunch robosar_emergency_response tepper_response_demo.launch $(quote_args "${OVERLAY_ARGS[@]}")"
TRAFFIC_CMD="rosrun robosar_controller robosar_traffic_controller_node"

echo "[tepper-response] workspace=$WORKSPACE_DIR"
echo "[tepper-response] TURTLEBOT3_MODEL=$MODEL"
echo "[tepper-response] gz_gui=$GZ_GUI rviz=$RVIZ gui=$GUI"
echo "[tepper-response] master_mode=$MASTER_MODE platform_mode=$PLATFORM_MODE ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>} GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI:-<caller-default>}"

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

  trap 'cleanup' EXIT
  trap 'handle_interrupt' INT TERM
fi

ensure_roscore
prepare_master

if [[ "$DRY_RUN" == "0" ]]; then
  rosparam set /use_sim_time true
else
  echo "[tepper-response][dry-run] rosparam: rosparam set /use_sim_time true"
fi

start_process_group "tepper_sim" "$GAZEBO_CMD" 1
wait_for_gazebo_clock
start_process_group "agent_bringup" "roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch" 1
start_process_group "slam" "roslaunch slam_toolbox robosar_mapping_sim_g.launch slam_node_type:=${SLAM_NODE_TYPE}" 1
start_process_group "task_generator" "roslaunch robosar_task_generator task_generator.launch" 1
start_process_group "mission_executive" "roslaunch robosar_navigation mission_executive.launch" 1
start_process_group "traffic_controller" "$TRAFFIC_CMD" 1
start_process_group "mission_commander" "rosrun robosar_task_allocator mission_commander.py -s=true -t='high'" 1
start_process_group "response_overlay" "$OVERLAY_CMD" 1
wait_for_response_gate_closed
if [[ "${GUI,,}" == "1" || "${GUI,,}" == "true" || "${GUI,,}" == "yes" || "${GUI,,}" == "on" ]]; then
  start_process_group "gui" "python3 $(printf "%q" "$GUI_SCRIPT")" 1
elif [[ "$DRY_RUN" == "1" ]]; then
  echo "[tepper-response][dry-run] gui: disabled"
fi

if [[ "$DRY_RUN" == "1" ]]; then
  exit 0
fi

wait
