#!/bin/bash
set -e

export ROS_PYTHON_VERSION=3

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$SCRIPT_DIR"
PYTHON_COMPAT_DIR="$WORKSPACE_DIR/work/python_compat"

cd "$WORKSPACE_DIR"

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

if [ -d "$PYTHON_COMPAT_DIR" ]; then
  export PYTHONPATH="$PYTHON_COMPAT_DIR${PYTHONPATH:+:$PYTHONPATH}"
fi

catkin_make
source devel/setup.bash

echo "DEFORM 编译完成。"
