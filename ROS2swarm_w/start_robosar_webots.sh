#!/usr/bin/env bash
# 作用：构建 launch_webots，并启动 RoboSAR 使用的 Webots 仿真与 ros1_bridge。
# 用法：./start_robosar_webots.sh [选项]
# 常用示例：./start_robosar_webots.sh --vglrun
# 常用示例：./start_robosar_webots.sh --no-vglrun
# 常用示例：./start_robosar_webots.sh --dry-run --vglrun

set -euo pipefail

usage() {
  cat <<'EOF'
作用：
  构建 launch_webots，并启动 RoboSAR Webots 仿真场景和 ros1_bridge。

用法：
  ./start_robosar_webots.sh [选项]

选项：
  --vglrun            使用 vglrun -d ${ROBOSAR_VGLRUN_DEVICE} 包装 Webots launch。
  --no-vglrun         禁用 vglrun。
  --dry-run           只打印将执行的命令，不真正构建或启动仿真/bridge。
  -h, --help          显示本说明。

环境变量：
  WORKSPACE_DIR             工作空间路径，默认脚本所在目录。
  NUM_ROBOTS                机器人数量，默认 5。
  ROBOT_TYPE                机器人类型，默认 e_puck。
  WEBOTS_WORLD              世界模板名，默认 robosar_apartment_smallv1.wbt。
  STATIC_MAP                静态地图 yaml；默认按 WEBOTS_WORLD 自动推导。
  PUBLISH_STATIC_MAP        是否发布 /sim_map，默认 false。
  ROS1_MASTER_IP            ROS1 master 地址，默认 localhost。
  SPAWN_X_MIN/MAX           随机出生区域 X 范围。
  SPAWN_Y_MIN/MAX           随机出生区域 Y 范围。
  SPAWN_MIN_DISTANCE        最小出生间距。
  ROBOSAR_USE_VGLRUN        auto、1/true 或 0/false；默认 auto。
  ROBOSAR_VGLRUN_DEVICE     vglrun 使用的设备，默认 /dev/dri/card0。
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
      echo "[start_robosar_webots][error] invalid ROBOSAR_USE_VGLRUN value: $1" >&2
      exit 2
      ;;
  esac
}

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-${SCRIPT_DIR}}"
HOST_WORKSPACE_DIR="$(cd -- "${WORKSPACE_DIR}/.." && pwd)"
SCENE_STATE_FILE="${SCENE_STATE_FILE:-${HOST_WORKSPACE_DIR}/.robosar_scene_state.json}"
SCENE_CONFIG="${SCENE_CONFIG:-${HOST_WORKSPACE_DIR}/masar_ws/src/robosar/robosar_sim_bringup/config/sim_scenes.json}"

NUM_ROBOTS="${NUM_ROBOTS:-5}"
ROBOT_TYPE="${ROBOT_TYPE:-e_puck}"
WEBOTS_WORLD="${WEBOTS_WORLD:-robosar_apartment_smallv1.wbt}"
STATIC_MAP="${STATIC_MAP:-}"
PUBLISH_STATIC_MAP="${PUBLISH_STATIC_MAP:-false}"
ROS1_MASTER_IP="${ROS1_MASTER_IP:-localhost}"
SPAWN_X_MIN="${SPAWN_X_MIN:--0.8}"
SPAWN_X_MAX="${SPAWN_X_MAX:--0.2}"
SPAWN_Y_MIN="${SPAWN_Y_MIN:--8.0}"
SPAWN_Y_MAX="${SPAWN_Y_MAX:--7.0}"
SPAWN_MIN_DISTANCE="${SPAWN_MIN_DISTANCE:-0.2}"
USE_VGLRUN="${ROBOSAR_USE_VGLRUN:-auto}"
VGLRUN_DEVICE="${ROBOSAR_VGLRUN_DEVICE:-/dev/dri/card0}"
DRY_RUN=0
WEBOTS_PID=""
BRIDGE_PID=""

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
      echo "[start_robosar_webots][error] unknown option: $1" >&2
      usage
      exit 2
      ;;
  esac
  shift
done

resolve_static_map_path() {
  local requested_map="$1"
  local webots_world="$2"
  local package_maps_dir="${WORKSPACE_DIR}/src/launch_webots/maps"
  local gazebo_maps_dir="${HOST_WORKSPACE_DIR}/masar_ws/src/robosar/robosar_sim_bringup/worlds/gazebo_config/maps"

  if [[ -n "${requested_map}" ]]; then
    if [[ "${requested_map}" = /* ]]; then
      echo "${requested_map}"
    elif [[ -f "${package_maps_dir}/${requested_map}" ]]; then
      echo "${package_maps_dir}/${requested_map}"
    elif [[ -f "${gazebo_maps_dir}/${requested_map}" ]]; then
      echo "${gazebo_maps_dir}/${requested_map}"
    else
      echo "${package_maps_dir}/${requested_map}"
    fi
    return
  fi

  local world_base="${webots_world%.wbt}"
  if [[ -f "${package_maps_dir}/${world_base}.yaml" ]]; then
    echo "${package_maps_dir}/${world_base}.yaml"
  else
    echo "${gazebo_maps_dir}/tepper.yaml"
  fi
}

write_scene_state() {
  local map_yaml="$1"
  local map_name
  map_name="$(basename "${map_yaml}")"
  map_name="${map_name%.yaml}"

  python3 - "${SCENE_STATE_FILE}" "${WEBOTS_WORLD}" "${map_name}" "${map_yaml}" <<'PY'
import json
import os
import sys

state_path, webots_world, map_name, map_yaml = sys.argv[1:5]
os.makedirs(os.path.dirname(state_path), exist_ok=True)
with open(state_path, "w", encoding="utf-8") as f:
    json.dump(
        {
            "source": "start_robosar_webots.sh",
            "webots_world": webots_world,
            "map_name": map_name,
            "map_yaml": map_yaml,
        },
        f,
        indent=2,
        sort_keys=True,
    )
PY
}

cleanup() {
  rm -f "${SCENE_STATE_FILE}"

  if [[ -n "${BRIDGE_PID}" ]] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
    kill "${BRIDGE_PID}" 2>/dev/null || true
  fi

  if [[ -n "${WEBOTS_PID}" ]] && kill -0 "${WEBOTS_PID}" 2>/dev/null; then
    kill "${WEBOTS_PID}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

STATIC_MAP="$(resolve_static_map_path "${STATIC_MAP}" "${WEBOTS_WORLD}")"
SIM_SCENE="${SIM_SCENE:-$(basename "${STATIC_MAP}")}"
SIM_SCENE="${SIM_SCENE%.yaml}"

echo "============================================================"
echo "  RoboSAR + Webots  (ROS2 side)"
echo "============================================================"
echo "  NUM_ROBOTS   = ${NUM_ROBOTS}"
echo "  ROBOT_TYPE   = ${ROBOT_TYPE}"
echo "  WEBOTS_WORLD = ${WEBOTS_WORLD}"
echo "  STATIC_MAP   = ${STATIC_MAP}"
echo "  SIM_SCENE    = ${SIM_SCENE}"
echo "  SCENE_CONFIG = ${SCENE_CONFIG}"
echo "  PUBLISH_MAP  = ${PUBLISH_STATIC_MAP}"
echo "  ROS1_MASTER  = ${ROS1_MASTER_IP}"
echo "  SPAWN_X      = [${SPAWN_X_MIN}, ${SPAWN_X_MAX}]"
echo "  SPAWN_Y      = [${SPAWN_Y_MIN}, ${SPAWN_Y_MAX}]"
echo "  MIN_DISTANCE = ${SPAWN_MIN_DISTANCE}"
echo "  SCENE_STATE  = ${SCENE_STATE_FILE}"
echo "============================================================"

build_cmd=(colcon build --symlink-install --packages-select launch_webots)
webots_launch_cmd=(
  ros2 launch launch_webots robosar_webots_sim.launch.py
  webots_world:=${WEBOTS_WORLD}
  number_robots:=${NUM_ROBOTS}
  robot:=${ROBOT_TYPE}
  static_map:=${STATIC_MAP}
  sim_scene:=${SIM_SCENE}
  scene_config:=${SCENE_CONFIG}
  publish_static_map:=${PUBLISH_STATIC_MAP}
  x_min:=${SPAWN_X_MIN}
  x_max:=${SPAWN_X_MAX}
  y_min:=${SPAWN_Y_MIN}
  y_max:=${SPAWN_Y_MAX}
  min_distance:=${SPAWN_MIN_DISTANCE}
)

if resolve_use_vglrun "$USE_VGLRUN"; then
  webots_launch_cmd=(vglrun -d "$VGLRUN_DEVICE" "${webots_launch_cmd[@]}")
fi

bridge_cmd=(ros2 run ros1_bridge dynamic_bridge --bridge-all-topics)

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[start_robosar_webots][dry-run] workdir: $WORKSPACE_DIR"
  echo "[start_robosar_webots][dry-run] build: $(quote_args "${build_cmd[@]}")"
  echo "[start_robosar_webots][dry-run] source: source ./install/setup.bash"
  echo "[start_robosar_webots][dry-run] webots: $(quote_args "${webots_launch_cmd[@]}")"
  echo "[start_robosar_webots][dry-run] env: export ROS_MASTER_URI=http://${ROS1_MASTER_IP}:11311"
  echo "[start_robosar_webots][dry-run] bridge: $(quote_args "${bridge_cmd[@]}")"
  exit 0
fi

write_scene_state "${STATIC_MAP}"

echo "[1/3] Building launch_webots..."
if [[ ! -f "${WORKSPACE_DIR}/src/launch_webots/package.xml" ]]; then
  echo "ERROR: launch_webots package not found under WORKSPACE_DIR=${WORKSPACE_DIR}" >&2
  exit 1
fi

cd "${WORKSPACE_DIR}"
"${build_cmd[@]}" 2>&1 | tail -5
source_workspace_setup

echo "[2/3] Starting Webots simulation..."
"${webots_launch_cmd[@]}" &
WEBOTS_PID=$!
sleep 15

echo "[3/3] Starting ros1_bridge (ROS_MASTER_URI=http://${ROS1_MASTER_IP}:11311)..."
export ROS_MASTER_URI="http://${ROS1_MASTER_IP}:11311"
"${bridge_cmd[@]}" &
BRIDGE_PID=$!

echo "============================================================"
echo "  ROS2 side ready!"
echo "  Webots PID  : ${WEBOTS_PID}"
echo "  Bridge PID  : ${BRIDGE_PID}"
echo ""
echo "  Robot type   : ${ROBOT_TYPE}"
echo "  Num robots   : ${NUM_ROBOTS}"
echo ""
echo "  Topic mapping (per robot N):"
echo "    /robot_N/scan       -> /robosar_.../agent_N/feedback/scan"
echo "    /robot_N/odom       -> /robosar_.../agent_N/odom"
echo "    /robosar_.../agent_N/cmd_vel -> /robot_N/cmd_vel"
echo ""
echo "  ROS1 side: run launch_robosar.sh (without simulation line)"
echo "============================================================"

wait "${WEBOTS_PID}"
