#!/usr/bin/env bash
# 作用：根目录快捷入口，启动 Tepper 大环境搜索组基础栈，并叠加响应组可变编队闭环。
# 用法：./run_tepper_response_demo.sh [选项] [tepper_response_demo.launch_arg:=value ...]
# 常用示例：./run_tepper_response_demo.sh --vglrun
# 常用示例：./run_tepper_response_demo.sh --headless --dry-run
# 说明：实际实现位于 src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh" "$@"
