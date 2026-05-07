#!/usr/bin/env bash

set -e
(set -o pipefail) 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"
WORKSPACE_SETUP="$WORKSPACE_DIR/devel/setup.bash"
LEGACY_WORKSPACE_SETUP="$HOME/catkin_ws2/devel/setup.bash"
GUI_SCRIPT="$WORKSPACE_DIR/src/robosar/robosar_gui/src/robosar_gui.py"
SCENE_CONFIG_PATH="$WORKSPACE_DIR/src/robosar/robosar_sim_bringup/config/sim_scenes.json"
SHARED_SCENE_STATE_FILE="${ROBOSAR_SCENE_STATE_FILE:-$(cd "$WORKSPACE_DIR/.." && pwd)/.robosar_scene_state.json}"

pids=()
SLAM_NODE_TYPE="${SLAM_NODE_TYPE:-sync_slam_toolbox_node}"
# Gazebo task-generator geofence is [2,19,0,18] in the original static-map frame.
# Webots + bridge + SLAM currently shift that frame into the negative quadrant, so
# use the translated bounds here while keeping Gazebo's shared launch defaults intact.
WEBOTS_TASK_GEOFENCE="${WEBOTS_TASK_GEOFENCE:-[-20.0,20.0,-20.0,20.0]}"

source_ros_env() {
  [ -f "/opt/ros/noetic/setup.bash" ] && . /opt/ros/noetic/setup.bash

  if [ -f "$WORKSPACE_SETUP" ]; then
    . "$WORKSPACE_SETUP"
  elif [ -f "$LEGACY_WORKSPACE_SETUP" ]; then
    . "$LEGACY_WORKSPACE_SETUP"
  else
    echo "[error] no catkin workspace setup found."
    echo "  looked for: $WORKSPACE_SETUP"
    echo "  fallback:   $LEGACY_WORKSPACE_SETUP"
    exit 1
  fi
}

source_ros_env

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

resolve_scene_context

echo "[scene] name=${ROBOSAR_SIM_SCENE_NAME} source=${ROBOSAR_SIM_SCENE_SOURCE}"
echo "[scene] map=${ROBOSAR_SIM_MAP_PATH}"

start() {
  local cmd="$1"
  local wait_sec="${2:-1}"
  setsid bash -lc "source_ros_env() { [ -f \"/opt/ros/noetic/setup.bash\" ] && . /opt/ros/noetic/setup.bash; if [ -f \"$WORKSPACE_SETUP\" ]; then . \"$WORKSPACE_SETUP\"; elif [ -f \"$LEGACY_WORKSPACE_SETUP\" ]; then . \"$LEGACY_WORKSPACE_SETUP\"; else echo \"[error] no catkin workspace setup found.\"; exit 1; fi; }; source_ros_env; $cmd" &
  local pid=$!
  pids+=("$pid")
  echo "[start] ($pid) $cmd"
  sleep "$wait_sec"
}

cleanup() {                            # 定义清理函数：当按 Ctrl+C 或收到 TERM 时执行
  echo -e "\n[cleanup] stopping all launched processes..."

  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then            # kill -0 只是探测进程是否存在
      echo "  -> SIGINT to PGID $pid"
      kill -INT -"${pid}" 2>/dev/null || true      # 对“负 PID”发信号 = 发给该 PID 的整个进程组
    fi
  done
  sleep 2                                          # 等 2 秒，给程序收尾时间

  # 2) 还活着的发 SIGTERM（更强硬一些的正常退出信号）
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo "  -> SIGTERM to PGID $pid"
      kill -TERM -"${pid}" 2>/dev/null || true
    fi
  done
  sleep 2

  # 3) 仍不退出就 KILL 兜底（不可恢复）
  for pid in "${pids[@]}"; do
    if kill -0 "$pid" 2>/dev/null; then
      echo "  -> SIGKILL to PGID $pid (force)"
      kill -KILL -"${pid}" 2>/dev/null || true
    fi
  done

}

trap 'cleanup; exit 130' INT TERM

# Step 1: start roscore explicitly and wait until it's ready
echo "[start] launching roscore..."
setsid roscore &
ROSCORE_PID=$!
pids+=("$ROSCORE_PID")

for i in $(seq 1 30); do
  if rostopic list >/dev/null 2>&1; then
    echo "[start] roscore is ready (took ~${i}s)"
    break
  fi
  sleep 1
done

# IMPORTANT: set sim-time BEFORE starting any ROS nodes.
# Otherwise early-started nodes (especially roscpp SLAM) may latch wall-time
# and publish TF with epoch timestamps, causing TF extrapolation errors.
echo "[start] setting /use_sim_time=true before launching nodes..."
rosparam set /use_sim_time true
sleep 1

# —— 逐条启动（按需改命令）——
start "roslaunch robosar_agent_bringup robosar_agent_bringup_node.launch"     # 1 启动机器人 agent
start "roslaunch robosar_sim_bringup robosar_sim_w.launch sim_scene:=${ROBOSAR_SIM_SCENE_NAME} sim_map_yaml:=${ROBOSAR_SIM_MAP_PATH}"               # 2 启动仿真服务节点
start "roslaunch slam_toolbox robosar_mapping_sim_w.launch slam_node_type:=${SLAM_NODE_TYPE}"  # 3 启动 SLAM/mapping
start "roslaunch robosar_task_generator task_generator.launch geofence:='${WEBOTS_TASK_GEOFENCE}'"  # 4 Webots 侧任务生成器参数覆盖
start "roslaunch robosar_navigation mission_executive.launch"                  # 5 导航/任务执行
start "rosrun robosar_controller robosar_traffic_controller_node"              # 6 控制器
start "rosrun robosar_task_allocator mission_commander.py -s=true -t='high'"   # 7 任务分配/调度器
start "python3 \"$GUI_SCRIPT\""                                                # 8 GUI

wait                                      # 等待所有后台子进程退出；当你 Ctrl+C 时，会触发 trap -> cleanup -> 再退出
