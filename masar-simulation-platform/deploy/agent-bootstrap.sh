#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: agent-bootstrap.sh <ros1|ros2> [runtime-agent options]

Starts the MASAR standard-library runtime Agent inside an existing ROS
container. The Agent binds only to 127.0.0.1 and does not modify the image.
EOF
}

if [[ ${1:-} == "--help" || ${1:-} == "-h" ]]; then
  usage
  exit 0
fi

agent=${1:-}
case "$agent" in
  ros1|ros2) ;;
  *)
    usage >&2
    exit 2
    ;;
esac
shift

platform_root=${MASAR_PLATFORM_ROOT:-/root/workspace/masar-simulation-platform}
exec python3 "$platform_root/agents/runtime_agent/main.py" --agent "$agent" "$@"
