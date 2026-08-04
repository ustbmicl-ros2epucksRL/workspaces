#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: agent-bootstrap.sh <ros1|ros2> [runtime-agent options]

Starts the MASAR standard-library Runtime Agent as the managed service inside
a ROS runtime container. The Agent binds only to 127.0.0.1 and does not modify
the image.
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
export NO_PROXY="${NO_PROXY:+${NO_PROXY},}127.0.0.1,localhost,::1"
export no_proxy="${no_proxy:+${no_proxy},}127.0.0.1,localhost,::1"

had_nounset=0
if [[ $- == *u* ]]; then
  had_nounset=1
  set +u
fi
if [[ "$agent" == "ros1" ]]; then
  [[ -f /opt/ros/noetic/setup.bash ]] && source /opt/ros/noetic/setup.bash
  [[ -f /root/workspace/masar_ws/devel/setup.bash ]] && source /root/workspace/masar_ws/devel/setup.bash
else
  [[ -f /opt/ros/humble/setup.bash ]] && source /opt/ros/humble/setup.bash
  [[ -f /root/ros2_ws/install/setup.bash ]] && source /root/ros2_ws/install/setup.bash
  [[ -f /root/workspace/ROS2swarm_w/install/setup.bash ]] && source /root/workspace/ROS2swarm_w/install/setup.bash
  [[ -f /root/ros-humble-ros1-bridge/install/setup.bash ]] && source /root/ros-humble-ros1-bridge/install/setup.bash
fi
if (( had_nounset )); then
  set -u
fi

exec python3 "$platform_root/agents/runtime_agent/main.py" --agent "$agent" "$@"
