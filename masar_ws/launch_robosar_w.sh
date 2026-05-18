#!/usr/bin/env bash
# 作用：启动 RoboSAR 原有 Webots 桥接/搜索任务闭环栈，包括地图上下文、SLAM、任务生成、导航调度和控制 GUI。
# 用法：./launch_robosar_w.sh [选项]
# 常用示例：./launch_robosar_w.sh --vglrun
# 常用示例：./launch_robosar_w.sh --no-rviz --no-vglrun
# 常用示例：./launch_robosar_w.sh --dry-run --sim-scene tepper
# 说明：运行 ./launch_robosar_w.sh --help 可查看完整参数；按 Ctrl+C 会关闭本脚本拉起的进程组。

set -euo pipefail

usage() {
  cat <<'EOF'
作用：
  启动 RoboSAR ROS1 Webots 桥接/搜索任务闭环栈。
  启动顺序保持原始工程逻辑：roscore -> agent -> Webots ROS 侧桥接 -> SLAM -> 任务生成 -> 导航/控制 -> GUI。

用法：
  ./launch_robosar_w.sh [选项]

选项：
  --rviz              设置 rviz:=true，显示 RViz。
  --no-rviz           设置 rviz:=false，不显示 RViz。
  --vglrun            使用 vglrun -d ${ROBOSAR_VGLRUN_DEVICE} 包装 Webots ROS 侧桥接 launch。
  --no-vglrun         禁用 vglrun。
  --sim-scene         设置仿真场景名，覆盖 ROBOSAR_SIM_SCENE。
  --sim-map-yaml      设置静态地图 yaml，覆盖 ROBOSAR_SIM_MAP_YAML。
  --task-geofence     设置任务生成器 geofence，默认读取 WEBOTS_TASK_GEOFENCE。
  --slam-node-type    设置 SLAM 节点类型，默认读取 SLAM_NODE_TYPE 或 sync_slam_toolbox_node。
  --dry-run           只打印启动顺序和命令，不真正启动 ROS/Webots 桥接。
  -h, --help          显示本说明。

环境变量：
  WORKSPACE_DIR             masar_ws 工作空间路径；默认是脚本所在目录。
  ROBOSAR_USE_VGLRUN        auto、1/true 或 0/false；默认 auto。
  ROBOSAR_VGLRUN_DEVICE     vglrun 使用的 DRM 设备；默认 /dev/dri/card0。
  ROBOSAR_RVIZ              RViz 默认开关；默认 true。
  ROBOSAR_SIM_SCENE         Webots/Gazebo 共享场景名覆盖。
  ROBOSAR_SIM_MAP_YAML      静态地图 yaml 覆盖。
  ROBOSAR_SCENE_STATE_FILE  GUI 写入的共享场景状态文件。
  WEBOTS_TASK_GEOFENCE      Webots 任务生成 geofence；默认 [-20.0,20.0,-20.0,20.0]。
  SLAM_NODE_TYPE            SLAM 节点类型；默认 sync_slam_toolbox_node。
EOF
}

die() {
  echo "[robosar-w][error] $*" >&2
  exit 1
}

quote_args() {
  printf "%q " "$@"
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

resolve_scene_context() {
  local scene_lines
  mapfile -t scene_lines < <(
    SCENE_CONFIG_PATH="$SCENE_CONFIG_PATH" \
    SHARED_SCENE_STATE_FILE="$SHARED_SCENE_STATE_FILE" \
    ROBOSAR_SIM_SCENE="${ROBOSAR_SIM_SCENE:-}" \
    ROBOSAR_SIM_MAP_YAML="${ROBOSAR_SIM_MAP_YAML:-}" \
    python3 - <<'PY'
import json
import os

config_path = os.environ["SCENE_CONFIG_PATH"]
state_path = os.environ["SHARED_SCENE_STATE_FILE"]
override_scene = os.environ.get("ROBOSAR_SIM_SCENE", "").strip()
override_map = os.environ.get("ROBOSAR_SIM_MAP_YAML", "").strip()

with open(config_path, "r", encoding="utf-8") as f:
    config = json.load(f)

scenes = config["scenes"]
default_scene = config.get("default_scene", next(iter(scenes)))

scene_name = default_scene
map_yaml = scenes[default_scene].get("map_yaml", "")
source = "default"

if override_scene or override_map:
    candidate_scene = override_scene or default_scene
    if candidate_scene in scenes:
        scene_name = candidate_scene
        map_yaml = override_map or scenes[scene_name].get("map_yaml", "")
        source = "override"
elif os.path.isfile(state_path):
    with open(state_path, "r", encoding="utf-8") as f:
        state = json.load(f)
    candidate_scene = state.get("map_name", "").strip()
    candidate_map = state.get("map_yaml", "").strip()
    if candidate_scene in scenes:
        scene_name = candidate_scene
        map_yaml = candidate_map or scenes[scene_name].get("map_yaml", "")
        source = "shared_state"

if not os.path.isfile(map_yaml):
    scene_name = default_scene
    map_yaml = scenes[default_scene].get("map_yaml", "")
    source = "default"

print(scene_name)
print(map_yaml)
print(source)
PY
  )

  ROBOSAR_SIM_SCENE_NAME="${scene_lines[0]}"
  ROBOSAR_SIM_MAP_PATH="${scene_lines[1]}"
  ROBOSAR_SIM_SCENE_SOURCE="${scene_lines[2]}"
}

start_process_group() {
  local label="$1"
  local cmd="$2"
  local wait_sec="${3:-1}"

  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[robosar-w][dry-run] ${label}: ${cmd}"
    return
  fi

  setsid bash -lc "
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
        echo '[robosar-w][error] no catkin setup found' >&2
        exit 1
      fi
    }
    source_ros_env
    ${cmd}
  " &

  local pid=$!
  PIDS+=("$pid")
  echo "[robosar-w][start] ($pid) ${label}: ${cmd}"
  sleep "$wait_sec"
}

wait_for_roscore() {
  local i
  for i in $(seq 1 30); do
    if rostopic list >/dev/null 2>&1; then
      echo "[robosar-w] roscore is ready (took ~${i}s)"
      return
    fi
    sleep 1
  done
  die "roscore did not become ready within 30s"
}

start_roscore() {
  if [[ "$DRY_RUN" == "1" ]]; then
    echo "[robosar-w][dry-run] roscore: roscore"
    return
  fi

  echo "[robosar-w][start] launching roscore..."
  setsid roscore &
  local pid=$!
  PIDS+=("$pid")
  wait_for_roscore
}

cleanup() {
  local pid
  echo
  echo "[robosar-w][cleanup] stopping all launched processes..."

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
WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"
WORKSPACE_SETUP="$WORKSPACE_DIR/devel/setup.bash"
LEGACY_WORKSPACE_SETUP="$HOME/catkin_ws2/devel/setup.bash"
GUI_SCRIPT="$WORKSPACE_DIR/src/robosar/robosar_gui/src/robosar_gui.py"
SCENE_CONFIG_PATH="$WORKSPACE_DIR/src/robosar/robosar_sim_bringup/config/sim_scenes.json"
SHARED_SCENE_STATE_FILE="${ROBOSAR_SCENE_STATE_FILE:-$(cd "$WORKSPACE_DIR/.." && pwd)/.robosar_scene_state.json}"

DRY_RUN=0
RVIZ="${ROBOSAR_RVIZ:-true}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
SLAM_NODE_TYPE="${SLAM_NODE_TYPE:-sync_slam_toolbox_node}"
WEBOTS_TASK_GEOFENCE="${WEBOTS_TASK_GEOFENCE:-[-20.0,20.0,-20.0,20.0]}"
PIDS=()

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --dry-run)
      DRY_RUN=1
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
    --sim-scene)
      shift
      [[ $# -gt 0 ]] || die "--sim-scene requires a value"
      ROBOSAR_SIM_SCENE="$1"
      ;;
    --sim-map-yaml)
      shift
      [[ $# -gt 0 ]] || die "--sim-map-yaml requires a value"
      ROBOSAR_SIM_MAP_YAML="$1"
      ;;
    --task-geofence)
      shift
      [[ $# -gt 0 ]] || die "--task-geofence requires a value"
      WEBOTS_TASK_GEOFENCE="$1"
      ;;
    --slam-node-type)
      shift
      [[ $# -gt 0 ]] || die "--slam-node-type requires a value"
      SLAM_NODE_TYPE="$1"
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
  shift
done

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

resolve_scene_context

VGLRUN_CMD=""
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  VGLRUN_CMD="$(quote_args "${WRAPPER[@]}")"
fi

WEBOTS_SIM_CMD="${VGLRUN_CMD}roslaunch robosar_sim_bringup robosar_sim_w.launch sim_scene:=${ROBOSAR_SIM_SCENE_NAME} sim_map_yaml:=${ROBOSAR_SIM_MAP_PATH} rviz:=${RVIZ}"

echo "[robosar-w] workspace=$WORKSPACE_DIR"
echo "[robosar-w] scene=${ROBOSAR_SIM_SCENE_NAME} source=${ROBOSAR_SIM_SCENE_SOURCE}"
echo "[robosar-w] map=${ROBOSAR_SIM_MAP_PATH}"
echo "[robosar-w] rviz=$RVIZ slam_node_type=$SLAM_NODE_TYPE geofence=$WEBOTS_TASK_GEOFENCE"
if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
  echo "[robosar-w] using vglrun device=$VGLRUN_DEVICE"
else
  echo "[robosar-w] vglrun disabled or unavailable"
fi

if [[ "$DRY_RUN" == "0" ]]; then
  source_ros_env

  if [[ "${#WRAPPER[@]}" -gt 0 ]]; then
    command -v vglrun >/dev/null 2>&1 || die "vglrun requested but not found in PATH."
    [[ -e "$VGLRUN_DEVICE" ]] || die "vglrun device not found: $VGLRUN_DEVICE"
  fi

  trap 'handle_interrupt' INT TERM
fi

start_roscore

if [[ "$DRY_RUN" == "0" ]]; then
  echo "[robosar-w] setting /use_sim_time=true before launching nodes..."
  rosparam set /use_sim_time true
else
  echo "[robosar-w][dry-run] rosparam: rosparam set /use_sim_time true"
fi

start_process_group "agent_bringup" "roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch" 1
start_process_group "webots_ros_bridge" "$WEBOTS_SIM_CMD" 1
start_process_group "slam" "roslaunch slam_toolbox robosar_mapping_sim_w.launch slam_node_type:=${SLAM_NODE_TYPE}" 1
start_process_group "task_generator" "roslaunch robosar_task_generator task_generator.launch geofence:='${WEBOTS_TASK_GEOFENCE}'" 1
start_process_group "mission_executive" "roslaunch robosar_navigation mission_executive.launch" 1
start_process_group "traffic_controller" "rosrun robosar_controller robosar_traffic_controller_node" 1
start_process_group "mission_commander" "rosrun robosar_task_allocator mission_commander.py -s=true -t='high'" 1
start_process_group "gui" "python3 $(printf "%q" "$GUI_SCRIPT")" 1

if [[ "$DRY_RUN" == "1" ]]; then
  exit 0
fi

wait
