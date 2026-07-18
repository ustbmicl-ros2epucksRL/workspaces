#!/usr/bin/env bash
# 作用：根目录快捷入口，向响应组发布开始信号，用于手动触发响应组任务执行。
# 用法：./start_response_tasks.sh [选项]
# 常用示例：./start_response_tasks.sh
# 常用示例：./start_response_tasks.sh --dry-run
# 说明：会自动加载 run_vnc_gpu.sh 对齐 VNC/GPU 环境；实际实现位于 src/robosar/robosar_emergency_response/scripts/start_response_tasks.sh。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${ROBOSAR_AUTO_VNC_GPU:-1}" != "0" && -f "$SCRIPT_DIR/run_vnc_gpu.sh" ]]; then
  source "$SCRIPT_DIR/run_vnc_gpu.sh" --env-only
fi
export WORKSPACE_DIR="${WORKSPACE_DIR:-$SCRIPT_DIR}"

exec "$WORKSPACE_DIR/src/robosar/robosar_emergency_response/scripts/start_response_tasks.sh" "$@"

#回航命令:
#cd /root/workspace/masar_ws
#source devel/setup.bash
#export ROS_MASTER_URI=http://127.0.0.1:11411

#rostopic pub -1 /response_group/return_home std_msgs/Bool "data: true"