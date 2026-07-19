#!/usr/bin/env bash
# 作用：构建 ROS2swarm Webots 仿真相关包，并启动默认的 transport 仿真场景。
# 用法：./start_simulation.sh [选项]
# 可直接修改的全局变量：
#   WEBOTS_WORLD / PATTERN / NUMBER_ROBOTS / TOTAL_ROBOTS
#   LOG_LEVEL / ROBOT / SENSOR_TYPE
#   WEBOTS_MODE / WEBOTS_GUI
#   X_MIN / X_MAX / Y_MIN / Y_MAX / MIN_DISTANCE / Z_HEIGHT
#   DRIVING_SWARM / LOGGING / NUM_OBSTACLES / TRANSPORT_OBJECT_SHAPE
#     这些变量会直接透传给 create_enviroment.launch.py。
# 常用示例：
#   ./start_simulation.sh --vglrun
#   ./start_simulation.sh --no-vglrun
#   ./start_simulation.sh --dry-run

set -euo pipefail

# Webots 世界模板文件名。
# Webots 世界模板文件名。
WEBOTS_WORLD="command_world.wbt"                # 例如：command_world.wbt
# 行为模式 launch 名称。
PATTERN="roundflocking_pattern"
# 例如：transport_pattern、roundflocking_pattern、navigation_pattern、aggregation_pattern、attraction_pattern、dispersion_pattern、drive_pattern。
#transport_pattern_neat、roundflocking_pattern_neat、navigation_pattern_neat
# 当前实例中实际生成并启动的机器人数量。
NUMBER_ROBOTS="5"                               # 正整数
# 实验总机器人数量，主要用于测量/日志相关流程。
TOTAL_ROBOTS="10"                               # 正整数
# ROS 日志级别。
LOG_LEVEL="info"                                # debug、info、warn、error、fatal
# Webots 运行模式和渲染开关。
WEBOTS_MODE="realtime"                          # pause、realtime、fast、rendering
WEBOTS_GUI="true"                               # true、false
# 机器人类型。
ROBOT="burger"                                  # e_puck、burger
# 传感器类型。
SENSOR_TYPE="lidar"                             # lidar、ir
# 随机生成区域 X 最小值。
X_MIN="-1.0"                                    # 浮点数
# 随机生成区域 X 最大值。
X_MAX="1.0"                                     # 浮点数
# 随机生成区域 Y 最小值。
Y_MIN="-1.0"                                    # 浮点数
# 随机生成区域 Y 最大值。
Y_MAX="1.0"                                     # 浮点数
# 初始生成时机器人/任务物体之间的最小间距。
MIN_DISTANCE="0.2"                              # 正浮点数
# 机器人生成高度。
Z_HEIGHT="0.0"                                  # 浮点数
# 是否启用 driving swarm 流程。
DRIVING_SWARM="False"                           # True、False
# 是否启用 rosbag 记录。
LOGGING="False"                                 # True、False
# 随机障碍物数量。
NUM_OBSTACLES="0"                               # 非负整数
# transport 任务中被推动物体的形状。
TRANSPORT_OBJECT_SHAPE="cube"                   # cube、cylinder、triangular_prism

usage() {
  cat <<'EOF'
作用：
  构建 communication_interfaces、launch_webots、ros2swarm，并启动默认 Webots 群体仿真。

用法：
  ./start_simulation.sh [选项]

选项：
  --vglrun
    使用 vglrun -d ${ROS2SWARM_VGLRUN_DEVICE} 包装 ros2 launch。
  --no-vglrun
    禁用 vglrun。
  --dry-run
    只打印将执行的命令，不真正构建或启动仿真。
  -h, --help
    显示本说明。

环境变量：
  WORKSPACE_DIR             工作空间路径，默认脚本所在目录。
  ROS2SWARM_USE_VGLRUN      auto、1/true 或 0/false；默认 auto。
  ROS2SWARM_VGLRUN_DEVICE   vglrun 使用的设备，默认 /dev/dri/card2。
  ROS2SWARM_VNC_DISPLAY     容器应连接的 TurboVNC DISPLAY，例如 :1；默认由 run_vnc_gpu.sh 自动检测。
  ROS2SWARM_XAUTHORITY      容器内 xauth 文件路径，默认优先 /tmp/.docker.xauth。

脚本内可直接修改的配置变量：
  WEBOTS_WORLD
    Webots 世界模板文件名。
  PATTERN
    行为模式 launch 名称；例如 transport_pattern、roundflocking_pattern、navigation_pattern、drive_pattern。
  NUMBER_ROBOTS
    当前实例中实际生成并启动的机器人数量；正整数。
  TOTAL_ROBOTS
    实验总机器人数量，主要给测量/日志流程使用；正整数。
  LOG_LEVEL
    ROS 日志级别；可选 debug、info、warn、error、fatal。
  WEBOTS_MODE
    Webots 运行模式；可选 pause、realtime、fast、rendering。
  WEBOTS_GUI
    是否启用 Webots GUI/rendering；可选 true、false。
  ROBOT
    机器人类型；可选 e_puck、burger。
  SENSOR_TYPE
    传感器类型；可选 lidar、ir。
  X_MIN / X_MAX / Y_MIN / Y_MAX
    机器人与任务物体的随机生成区域边界；浮点数。
  MIN_DISTANCE
    初始生成时各物体/机器人之间的最小间距；正浮点数。
  Z_HEIGHT
    机器人生成高度；浮点数。
  DRIVING_SWARM
    是否启用 driving swarm 流程；可选 True、False。
  LOGGING
    是否启用 rosbag 记录；可选 True、False。
  NUM_OBSTACLES
    随机障碍物数量；非负整数。
  TRANSPORT_OBJECT_SHAPE
    transport 任务中被推动物体的形状；可选 cube、cylinder、triangular_prism。
EOF
}

quote_args() {
  printf "%q " "$@"
}

source_workspace_setup() {
  local had_nounset=0
  if [[ $- == *u* ]]; then
    had_nounset=1
    set +u
  fi

  # colcon-generated setup.bash reads COLCON_TRACE without guarding unset vars.
  source "$WORKSPACE_DIR/install/setup.bash"

  if (( had_nounset )); then
    set -u
  fi
}

resolve_use_vglrun() {
  local value="${1,,}"
  case "$value" in
    1|true|yes|on)
      return 0
      ;;
    0|false|no|off)
      return 1
      ;;
    auto)
      command -v vglrun >/dev/null 2>&1
      return
      ;;
    *)
      echo "[start_simulation][error] invalid ROS2SWARM_USE_VGLRUN value: $1" >&2
      exit 2
      ;;
  esac
}

validate_transport_object_shape() {
  case "$1" in
    cube|cylinder|triangular_prism)
      ;;
    *)
      echo "[start_simulation][error] invalid transport object shape: $1" >&2
      echo "[start_simulation][error] expected one of: cube, cylinder, triangular_prism" >&2
      exit 2
      ;;
  esac
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-${SCRIPT_DIR}}"

# 宿主机重启后 TurboVNC 的 DISPLAY/xauth 可能变化；自动对齐容器内 VNC + GPU 环境。
if [[ -f "${SCRIPT_DIR}/run_vnc_gpu.sh" ]]; then
  source "${SCRIPT_DIR}/run_vnc_gpu.sh" --quiet
fi

USE_VGLRUN="${ROS2SWARM_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROS2SWARM_VGLRUN_DEVICE:-/dev/dri/card2}"
DRY_RUN=0

while (($#)); do
  case "$1" in
    --vglrun)
      USE_VGLRUN=true
      ;;
    --no-vglrun)
      USE_VGLRUN=false
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[start_simulation][error] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
  shift
done

validate_transport_object_shape "$TRANSPORT_OBJECT_SHAPE"

launch_cmd=(
  ros2 launch launch_webots create_enviroment.launch.py
  webots_world:=${WEBOTS_WORLD}
  pattern:=${PATTERN}
  number_robots:=${NUMBER_ROBOTS}
  total_robots:=${TOTAL_ROBOTS}
  log_level:=${LOG_LEVEL}
  webots_mode:=${WEBOTS_MODE}
  webots_gui:=${WEBOTS_GUI}
  robot:=${ROBOT}
  sensor_type:=${SENSOR_TYPE}
  x_min:=${X_MIN}
  x_max:=${X_MAX}
  y_min:=${Y_MIN}
  y_max:=${Y_MAX}
  min_distance:=${MIN_DISTANCE}
  z_height:=${Z_HEIGHT}
  driving_swarm:=${DRIVING_SWARM}
  logging:=${LOGGING}
  num_obstacles:=${NUM_OBSTACLES}
  transport_object_shape:=${TRANSPORT_OBJECT_SHAPE}
)

if resolve_use_vglrun "$USE_VGLRUN"; then
  launch_cmd=(vglrun -d "$VGLRUN_DEVICE" "${launch_cmd[@]}")
fi

launch_cmd=(env ROS_DOMAIN_ID=42 "${launch_cmd[@]}")
build_cmd=(colcon build --symlink-install --allow-overriding communication_interfaces launch_webots ros2swarm)

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[start_simulation][dry-run] workdir: $WORKSPACE_DIR"
  if [[ "${ROS2SWARM_VNC_GPU_LOADED:-}" == "1" ]]; then
    echo "[start_simulation][dry-run] vnc-gpu: DISPLAY=${DISPLAY:-} XAUTHORITY=${XAUTHORITY:-} VGLRUN_DEVICE=${VGLRUN_DEVICE}"
  fi
  echo "[start_simulation][dry-run] build: $(quote_args "${build_cmd[@]}")"
  echo "[start_simulation][dry-run] source: source ./install/setup.bash"
  echo "[start_simulation][dry-run] launch: $(quote_args "${launch_cmd[@]}")"
  exit 0
fi

cd "$WORKSPACE_DIR"
"${build_cmd[@]}"
source_workspace_setup
"${launch_cmd[@]}"
