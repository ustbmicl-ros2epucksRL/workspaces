#!/usr/bin/env bash

set -e
(set -o pipefail) 2>/dev/null || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"
WORKSPACE_SETUP="$WORKSPACE_DIR/devel/setup.bash"
LEGACY_WORKSPACE_SETUP="$HOME/catkin_ws2/devel/setup.bash"
GUI_SCRIPT="$WORKSPACE_DIR/src/robosar/robosar_gui/src/robosar_gui.py"

pids=()
SLAM_NODE_TYPE="${SLAM_NODE_TYPE:-sync_slam_toolbox_node}"

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
start "roslaunch robosar_sim_bringup robosar_sim_w.launch"               # 2 启动 Webots 侧仿真服务节点
start "roslaunch slam_toolbox robosar_mapping_sim_w.launch slam_node_type:=${SLAM_NODE_TYPE}"  # 3 启动 SLAM/mapping
start "roslaunch robosar_task_generator task_generator.launch"                 # 4 任务生成器
start "roslaunch robosar_navigation mission_executive.launch"                  # 5 导航/任务执行
start "rosrun robosar_controller robosar_traffic_controller_node"              # 6 控制器
start "rosrun robosar_task_allocator mission_commander.py -s=true -t='high'"   # 7 任务分配/调度器
start "python3 \"$GUI_SCRIPT\""                                                # 8 GUI

wait                                      # 等待所有后台子进程退出；当你 Ctrl+C 时，会触发 trap -> cleanup -> 再退出
