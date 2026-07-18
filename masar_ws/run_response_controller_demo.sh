#!/usr/bin/env bash
# 作用：根目录快捷入口；普通任务名验证 Tepper 当前生产控制器，显式后缀用于对照旧控制器。
# 用法：./run_response_controller_demo.sh [选项] {roundflocking|roundflocking-checkpoint|roundflocking-heuristic|transport|transport-checkpoint|transport-heuristic} [launch_arg:=value ...]
# 常用示例：./run_response_controller_demo.sh roundflocking
# 常用示例：./run_response_controller_demo.sh --headless transport
# 说明：会自动加载 run_vnc_gpu.sh 对齐 VNC/GPU 环境；实际实现位于 src/robosar/robosar_emergency_response/scripts/run_response_controller_demo.sh。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${ROBOSAR_AUTO_VNC_GPU:-1}" != "0" && -f "$SCRIPT_DIR/run_vnc_gpu.sh" ]]; then
  source "$SCRIPT_DIR/run_vnc_gpu.sh" --env-only
fi
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/run_response_controller_demo.sh" "$@"
