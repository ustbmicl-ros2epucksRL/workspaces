#!/usr/bin/env bash
# 作用：根目录快捷入口，启动 Tepper 大环境搜索组基础栈，并叠加响应组可变编队闭环。
# 用法：./run_tepper_response_demo.sh [选项] [tepper_response_demo.launch_arg:=value ...]
# 常用示例：./run_tepper_response_demo.sh
# 常用示例：./run_tepper_response_demo.sh --headless --dry-run
# 说明：会自动加载 run_vnc_gpu.sh 对齐 VNC/GPU 环境；实际实现位于 src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${ROBOSAR_AUTO_VNC_GPU:-1}" != "0" && -f "$SCRIPT_DIR/run_vnc_gpu.sh" ]]; then
  source "$SCRIPT_DIR/run_vnc_gpu.sh" --env-only
fi
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh" "$@"
