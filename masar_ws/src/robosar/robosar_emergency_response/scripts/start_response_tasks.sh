#!/usr/bin/env bash

set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  start_response_tasks.sh [options]

Publishes the response-demo task-start signal consumed by response_goal_sequence_node:
  rostopic pub -1 /response_group/start std_msgs/Bool "data: true"

Options:
  --isolated      Use the isolated ROS master used by the response demo launcher. Default.
  --shared-master Reuse the caller's ROS_MASTER_URI.
  --dry-run       Print the resolved command without executing it.
  -h, --help      Show this help.

Environment:
  WORKSPACE_DIR           Catkin workspace root. Auto-detected when unset.
  ROBOSAR_RESPONSE_DEMO_ROS_PORT Isolated ROS master port. Default: 11411.
  ROBOSAR_SKIP_SETUP      Set to 1 for dry-run tests that should not source ROS.
EOF
}

die() {
  echo "[response-start][error] $*" >&2
  exit 1
}

resolve_workspace_dir() {
  if [[ -n "${WORKSPACE_DIR:-}" ]]; then
    cd "$WORKSPACE_DIR" && pwd
    return
  fi

  local dir="$SCRIPT_DIR"
  while [[ "$dir" != "/" ]]; do
    if [[ -f "$dir/devel/setup.bash" && -d "$dir/src" ]]; then
      echo "$dir"
      return
    fi
    dir="$(dirname "$dir")"
  done

  cd "$SCRIPT_DIR/../../../.." && pwd
}

source_ros_env() {
  local workspace_setup="$WORKSPACE_DIR/devel/setup.bash"

  if [[ -f "/opt/ros/noetic/setup.bash" ]]; then
    # shellcheck source=/opt/ros/noetic/setup.bash
    source "/opt/ros/noetic/setup.bash"
  fi

  if [[ ! -f "$workspace_setup" ]]; then
    die "catkin setup file not found: $workspace_setup. Run catkin_make first or set WORKSPACE_DIR."
  fi

  # shellcheck source=/dev/null
  source "$workspace_setup"
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(resolve_workspace_dir)"
DRY_RUN=0
RESPONSE_DEMO_ISOLATED="${ROBOSAR_RESPONSE_DEMO_ISOLATED:-1}"
RESPONSE_DEMO_ROS_PORT="${ROBOSAR_RESPONSE_DEMO_ROS_PORT:-11411}"

while (($#)); do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --dry-run)
      DRY_RUN=1
      ;;
    --isolated)
      RESPONSE_DEMO_ISOLATED=1
      ;;
    --shared-master)
      RESPONSE_DEMO_ISOLATED=0
      ;;
    *)
      die "unknown option: $1"
      ;;
  esac
  shift
done

case "${RESPONSE_DEMO_ISOLATED,,}" in
  1|true|yes|on)
    export ROS_MASTER_URI="${ROBOSAR_RESPONSE_DEMO_ROS_MASTER_URI:-http://127.0.0.1:${RESPONSE_DEMO_ROS_PORT}}"
    ;;
  0|false|no|off)
    ;;
  *)
    die "ROBOSAR_RESPONSE_DEMO_ISOLATED must be 1/true or 0/false; got '$RESPONSE_DEMO_ISOLATED'."
    ;;
esac

COMMAND=(rostopic pub -1 /response_group/start std_msgs/Bool "data: true")

if [[ "$DRY_RUN" == "1" ]]; then
  echo "[response-start][dry-run] workspace=$WORKSPACE_DIR"
  echo "[response-start][dry-run] ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>}"
  echo "[response-start][dry-run] payload=data: true"
  printf '[response-start][dry-run] command='
  printf '%q ' "${COMMAND[@]}"
  printf '\n'
  exit 0
fi

if [[ "${ROBOSAR_SKIP_SETUP:-0}" != "1" ]]; then
  source_ros_env
fi

echo "[response-start] ROS_MASTER_URI=${ROS_MASTER_URI:-<caller-default>}"
printf '[response-start] command='
printf '%q ' "${COMMAND[@]}"
printf '\n'
exec "${COMMAND[@]}"
