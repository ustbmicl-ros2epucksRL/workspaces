#!/bin/bash
# ============================================================
#  RoboSAR + Webots 仿真启动脚本（ROS2 侧）
# ============================================================
#
#  Usage:
#    ./start_robosar_webots.sh [options]
#
#  Options (env vars or positional defaults):
#    NUM_ROBOTS     : number of robots (default 5)
#    ROBOT_TYPE     : e_puck | burger  (default e_puck)
#    WEBOTS_WORLD   : world template   (default robosar_apartment_smallv1.wbt)
#    STATIC_MAP     : map yaml file    (default auto-derived from WEBOTS_WORLD)
#    PUBLISH_STATIC_MAP
#                   : publish ROS2 local /sim_map (default false)
#    ROS1_MASTER_IP : ROS1 master IP   (default localhost for --network=host)
#    SPAWN_X_MIN/MAX, SPAWN_Y_MIN/MAX, SPAWN_MIN_DISTANCE
#                   : random spawn area controls
#
#  ROS1 容器中运行（去掉仿真行的 launch_robosar.sh）:
#    ./launch_robosar.sh
#
# ============================================================

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="${WORKSPACE_DIR:-${SCRIPT_DIR}}"
HOST_WORKSPACE_DIR="$(cd -- "${WORKSPACE_DIR}/.." && pwd)"
SCENE_STATE_FILE="${SCENE_STATE_FILE:-${HOST_WORKSPACE_DIR}/.robosar_scene_state.json}"
SCENE_CONFIG="${SCENE_CONFIG:-${HOST_WORKSPACE_DIR}/masar_ws/src/robosar/robosar_sim_bringup/config/sim_scenes.json}"

# --- 配置（可通过环境变量覆盖） ---
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
WEBOTS_PID=""
BRIDGE_PID=""

resolve_static_map_path() {
    local requested_map="$1"
    local webots_world="$2"
    local package_maps_dir="${WORKSPACE_DIR}/src/launch_webots/maps"
    local gazebo_maps_dir="${HOST_WORKSPACE_DIR}/masar_ws/src/robosar/robosar_sim_bringup/worlds/gazebo_config/maps"

    if [ -n "${requested_map}" ]; then
        if [[ "${requested_map}" = /* ]]; then
            echo "${requested_map}"
        elif [ -f "${package_maps_dir}/${requested_map}" ]; then
            echo "${package_maps_dir}/${requested_map}"
        elif [ -f "${gazebo_maps_dir}/${requested_map}" ]; then
            echo "${gazebo_maps_dir}/${requested_map}"
        else
            echo "${package_maps_dir}/${requested_map}"
        fi
        return
    fi

    local world_base="${webots_world%.wbt}"
    if [ -f "${package_maps_dir}/${world_base}.yaml" ]; then
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

    if [ -n "${BRIDGE_PID}" ] && kill -0 "${BRIDGE_PID}" 2>/dev/null; then
        kill "${BRIDGE_PID}" 2>/dev/null || true
    fi

    if [ -n "${WEBOTS_PID}" ] && kill -0 "${WEBOTS_PID}" 2>/dev/null; then
        kill "${WEBOTS_PID}" 2>/dev/null || true
    fi
}

trap cleanup EXIT INT TERM

STATIC_MAP="$(resolve_static_map_path "${STATIC_MAP}" "${WEBOTS_WORLD}")"
SIM_SCENE="${SIM_SCENE:-$(basename "${STATIC_MAP}")}"
SIM_SCENE="${SIM_SCENE%.yaml}"
write_scene_state "${STATIC_MAP}"

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

# --- 构建 ---
echo "[1/3] Building launch_webots..."
if [ ! -f "${WORKSPACE_DIR}/src/launch_webots/package.xml" ]; then
    echo "ERROR: launch_webots package not found under WORKSPACE_DIR=${WORKSPACE_DIR}" >&2
    exit 1
fi

cd "${WORKSPACE_DIR}"
colcon build --symlink-install --packages-select launch_webots 2>&1 | tail -5
source ./install/setup.bash

# NOTE: use_sim_time is now handled correctly:
#   - ROS2 bridge adapter runs with use_sim_time=True (Webots sim clock)
#   - ROS1 mission_commander -s=true sets /use_sim_time=true
#   - /clock from Webots is bridged to ROS1 via ros1_bridge
#   - All TF/topic timestamps use the same sim-time domain

# --- 启动 Webots 仿真 ---
echo "[2/3] Starting Webots simulation..."
ros2 launch launch_webots robosar_webots_sim.launch.py \
    webots_world:=${WEBOTS_WORLD} \
    number_robots:=${NUM_ROBOTS} \
    robot:=${ROBOT_TYPE} \
    static_map:=${STATIC_MAP} \
    sim_scene:=${SIM_SCENE} \
    scene_config:=${SCENE_CONFIG} \
    publish_static_map:=${PUBLISH_STATIC_MAP} \
    x_min:=${SPAWN_X_MIN} x_max:=${SPAWN_X_MAX} \
    y_min:=${SPAWN_Y_MIN} y_max:=${SPAWN_Y_MAX} \
    min_distance:=${SPAWN_MIN_DISTANCE} &

WEBOTS_PID=$!
sleep 15

# --- 启动 ros1_bridge ---
echo "[3/3] Starting ros1_bridge (ROS_MASTER_URI=http://${ROS1_MASTER_IP}:11311)..."
export ROS_MASTER_URI=http://${ROS1_MASTER_IP}:11311

# --bridge-all-topics 确保所有话题被桥接（纯 dynamic_bridge 无法自动发现所有匹配）。
# /clock 被桥接是必须的：ROS1 mission_commander 设 use_sim_time=true 后需要它。
# /tf 被桥接以传递 adapter 发布的 map->odom 和 odom->base_link 变换。
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics &

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

wait ${WEBOTS_PID}
