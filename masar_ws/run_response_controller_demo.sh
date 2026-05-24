#!/usr/bin/env bash
# 作用：根目录快捷入口，启动应急组 checkpoint/heuristic 控制器的独立 Gazebo 验证 demo。
# 用法：./run_response_controller_demo.sh [选项] {roundflocking|transport|transport-checkpoint|transport-heuristic} [launch_arg:=value ...]
# 常用示例：./run_response_controller_demo.sh --headless roundflocking
# 常用示例：./run_response_controller_demo.sh --headless transport
# 说明：实际实现位于 src/robosar/robosar_emergency_response/scripts/run_response_controller_demo.sh。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/run_response_controller_demo.sh" "$@"
