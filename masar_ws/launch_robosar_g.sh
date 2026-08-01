#!/usr/bin/env bash
# 作用：启动 RoboSAR 原有 Gazebo/Tepper 搜索任务闭环栈，包括仿真、SLAM、任务生成、导航调度和控制 GUI。
# 用法：./launch_robosar_g.sh [选项] [robosar_sim_tepper.launch_arg:=value ...]
# 常用示例：./launch_robosar_g.sh
# 常用示例：./launch_robosar_g.sh --headless --no-vglrun
# 常用示例：./launch_robosar_g.sh --dry-run --vglrun
# 说明：会自动加载 run_vnc_gpu.sh 对齐 VNC/GPU 环境；运行 ./launch_robosar_g.sh --help 可查看完整参数；按 Ctrl+C 会关闭本脚本拉起的进程组。

#指定运行 GPU/X screen命令：
#ROBOSAR_VGLRUN_DEVICE=:0.0 ./launch_robosar_g.sh --vglrun  # GPU 0
#ROBOSAR_VGLRUN_DEVICE=:0.1 ./launch_robosar_g.sh --vglrun  # GPU 1

set -euo pipefail

usage() {
  cat <<'EOF'
作用：
  启动 RoboSAR ROS1 + Gazebo Tepper 搜索任务闭环栈。
  启动顺序保持原始工程逻辑：roscore -> agent -> Gazebo -> SLAM -> 任务生成 -> 导航/控制 -> GUI。

用法：
  ./launch_robosar_g.sh [选项] [robosar_sim_tepper.launch_arg:=value ...]

选项：
  --headless          同时设置 gz_gui:=false 和 rviz:=false，适合无显示环境。
  --gz-gui            设置 gz_gui:=true，显示 Gazebo 图形界面。
  --no-gz-gui         设置 gz_gui:=false，不显示 Gazebo 图形界面。
  --rviz              设置 rviz:=true，显示 RViz。
  --no-rviz           设置 rviz:=false，不显示 RViz。
  --vglrun            使用 vglrun -d ${ROBOSAR_VGLRUN_DEVICE} 包装 Gazebo 启动命令。
  --no-vglrun         禁用 vglrun。
  --slam-node-type    设置 SLAM 节点类型，默认读取 SLAM_NODE_TYPE 或 sync_slam_toolbox_node。
  --dry-run           只打印启动顺序和命令，不真正启动 ROS/Gazebo。
  -h, --help          显示本说明。

环境变量：
  WORKSPACE_DIR          masar_ws 工作空间路径；默认是脚本所在目录。
  ROBOSAR_USE_VGLRUN     auto、1/true 或 0/false；默认 auto。
  ROBOSAR_VGLRUN_DEVICE  vglrun 使用的 DRM 设备或 X display；默认 /dev/dri/card0。
  ROBOSAR_GZ_GUI         Gazebo GUI 默认开关；默认 true。
  ROBOSAR_RVIZ           RViz 默认开关；默认 true。
  MASAR_PLATFORM_MODE    设为 1 时禁用原 Qt GUI、二次 setsid 和全局进程清理。
  SLAM_NODE_TYPE         SLAM 节点类型；默认 sync_slam_toolbox_node。
EOF
}

die() {
  echo "[robosar-g][error] $*" >&2
  exit 1
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

platform_mode_enabled() {
  case "${PLATFORM_MODE,,}" in
    1|true|yes|on) return 0 ;;
    *) return 1 ;;
  esac
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

source_ros_env() {
  [[ -f "/opt/ros/noetic/setup.bash" ]] && source "/opt/ros/noetic/setup.bash"

  if [[ -f "$WORKSPACE_SETUP" ]]; then
    source "$WORKSPACE_SETUP"
  elif [[ -f "$LEGACY_WORKSPACE_SETUP" ]]; then
    source "$LEGACY_WORKSPACE_SETUP"
  else
    die "no catkin workspace setup found. looked for: $WORKSPACE_SETUP; fallback: $LEGACY_WORKSPACE_SETUP"
  fi
}

start_process_group() {
  local label="$1"
  local cmd="$2"
  local wait_sec="${3:-1}"

  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[robosar-g][dry-run] ${label}: ${cmd}"
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
      [ -f /opt/ros/noetic/setup.bash ] && . /opt/ros/noetic/setup.bash
      if [ -f \"\$WORKSPACE_SETUP\" ]; then
        . \"\$WORKSPACE_SETUP\"
      elif [ -f \"\$LEGACY_WORKSPACE_SETUP\" ]; then
        . \"\$LEGACY_WORKSPACE_SETUP\"
      else
        echo '[robosar-g][error] no catkin setup found' >&2
        exit 1
      fi
    }
    source_ros_env
    ${cmd}
  " &

  local pid=$!
  PIDS+=("$pid")
  echo "[robosar-g][start] ($pid) ${label}: ${cmd}"
  sleep "$wait_sec"
}

wait_for_roscore() {
  local i
  for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then
      echo "[robosar-g] roscore is ready (took ~${i}s)"
      return
    fi
    sleep 1
  done
  die "roscore did not become ready within 30s"
}

start_roscore() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[robosar-g][dry-run] roscore: roscore"
    return
  fi

  echo "[robosar-g][start] launching roscore..."
  local launcher=()
  if ! platform_mode_enabled; then
    launcher=(setsid)
  fi
  "${launcher[@]}" roscore &
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
  echo "[robosar-g][cleanup] stopping all launched processes..."

  if platform_mode_enabled; then
    echo "  -> platform mode: Runtime Agent owns and signals the complete process group"
    return
  fi

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

  echo "  -> Gazebo targeted shutdown (gzserver/gzclient)"
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
if [[ "${ROBOSAR_AUTO_VNC_GPU:-1}" != "0" && -f "$SCRIPT_DIR/run_vnc_gpu.sh" ]]; then
  source "$SCRIPT_DIR/run_vnc_gpu.sh" --env-only
fi
WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"
WORKSPACE_SETUP="$WORKSPACE_DIR/devel/setup.bash"
LEGACY_WORKSPACE_SETUP="$HOME/catkin_ws2/devel/setup.bash"
GUI_SCRIPT="$WORKSPACE_DIR/src/robosar/robosar_gui/src/robosar_gui.py"

DRY_RUN=0
PLATFORM_MODE="${MASAR_PLATFORM_MODE:-0}"
GZ_GUI="${ROBOSAR_GZ_GUI:-true}"
RVIZ="${ROBOSAR_RVIZ:-true}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
SLAM_NODE_TYPE="${SLAM_NODE_TYPE:-sync_slam_toolbox_node}"
ENABLE_APRILTAG_SIM="${ENABLE_APRILTAG_SIM:-0}"
SIM_ARGS=()
PIDS=()

if platform_mode_enabled; then
  RVIZ=false
fi

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
    --slam-node-type)
      shift
      [[ $# -gt 0 ]] || die "--slam-node-type requires a value"
      SLAM_NODE_TYPE="$1"
      ;;
    --)
      shift
      SIM_ARGS+=("$@")
      break
      ;;
    -*)
      die "unknown option: $1"
      ;;
    *)
      SIM_ARGS+=("$1")
      ;;
  esac
  shift
done

if ! has_roslaunch_arg "gz_gui" "${SIM_ARGS[@]}"; then
  SIM_ARGS=("gz_gui:=$GZ_GUI" "${SIM_ARGS[@]}")
fi
if ! has_roslaunch_arg "rviz" "${SIM_ARGS[@]}"; then
  SIM_ARGS=("rviz:=$RVIZ" "${SIM_ARGS[@]}")
fi

WRAPPER=()
case "${USE_VGLRUN,,}" in
  auto)
    if command -v vglrun >/dev/null 2>&1 && vglrun_target_is_available "$VGLRUN_DEVICE"; then
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

VGLRUN_CMD=""
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  VGLRUN_CMD="$(quote_args "${WRAPPER[@]}")"
fi

GAZEBO_CMD="GAZEBO_MODEL_DATABASE_URI='' ${VGLRUN_CMD}roslaunch robosar_sim_bringup robosar_sim_tepper.launch $(quote_args "${SIM_ARGS[@]}")"

echo "[robosar-g] workspace=$WORKSPACE_DIR"
echo "[robosar-g] gz_gui=$GZ_GUI rviz=$RVIZ slam_node_type=$SLAM_NODE_TYPE platform_mode=$PLATFORM_MODE"
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  echo "[robosar-g] using vglrun device=$VGLRUN_DEVICE"
else
  echo "[robosar-g] vglrun disabled or unavailable"
fi

if [[ "$DRY_RUN" == "0" ]]; then
  source_ros_env
  export GAZEBO_MODEL_DATABASE_URI="${GAZEBO_MODEL_DATABASE_URI:-}"
  export ENABLE_APRILTAG_SIM

  if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
    command -v vglrun >/dev/null 2>&1 || die "vglrun requested but not found in PATH."
    vglrun_target_is_available "$VGLRUN_DEVICE" || die "vglrun target not found: $VGLRUN_DEVICE"
  fi

  trap 'handle_interrupt' INT TERM
fi

start_roscore

if [[ "$DRY_RUN" == "0" ]]; then
  echo "[robosar-g] setting /use_sim_time=true before launching nodes..."
  rosparam set /use_sim_time true
else
  echo "[robosar-g][dry-run] rosparam: rosparam set /use_sim_time true"
fi

start_process_group "agent_bringup" "roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch" 1
start_process_group "tepper_sim" "$GAZEBO_CMD" 1
start_process_group "slam" "roslaunch slam_toolbox robosar_mapping_sim_g.launch slam_node_type:=${SLAM_NODE_TYPE}" 1
start_process_group "task_generator" "roslaunch robosar_task_generator task_generator.launch" 1
start_process_group "mission_executive" "roslaunch robosar_navigation mission_executive.launch" 1
start_process_group "traffic_controller" "rosrun robosar_controller robosar_traffic_controller_node" 1
start_process_group "mission_commander" "rosrun robosar_task_allocator mission_commander.py -s=true -t='high'" 1
if platform_mode_enabled; then
  echo "[robosar-g] platform mode: original Qt GUI disabled"
else
  start_process_group "gui" "python3 $(printf "%q" "$GUI_SCRIPT")" 1
fi

if [[ "$DRY_RUN" == "1" ]]; then
  exit 0
fi

wait
