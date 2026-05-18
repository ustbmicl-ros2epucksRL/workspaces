#!/usr/bin/env bash
# 作用：根目录快捷入口，向响应组发布开始信号，用于手动触发响应组任务执行。
# 用法：./start_response_tasks.sh [选项]
# 常用示例：./start_response_tasks.sh
# 常用示例：./start_response_tasks.sh --dry-run
# 说明：实际实现位于 src/robosar/robosar_emergency_response/scripts/start_response_tasks.sh。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/start_response_tasks.sh" "$@"
