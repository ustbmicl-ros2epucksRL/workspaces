#!/bin/bash
export ROS_PYTHON_VERSION=3

# 根据脚本位置定位 catkin 工作空间，避免写死绝对路径
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PYTHON_COMPAT_DIR="$WORKSPACE_DIR/work/python_compat"
if [ ! -d "$WORKSPACE_DIR" ]; then
  echo "错误：工作空间目录 $WORKSPACE_DIR 不存在！"
  exit 1
fi

cd "$WORKSPACE_DIR" || exit 1

if [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
fi

if [ -d "$PYTHON_COMPAT_DIR" ]; then
  export PYTHONPATH="$PYTHON_COMPAT_DIR${PYTHONPATH:+:$PYTHONPATH}"
fi

# 清理旧编译产物
echo "正在清理旧的构建文件..."
rm -rf build devel
echo "清理完成"

# 定义要编译的包列表（按依赖顺序排列）
PACKAGES=(
  "robosar_messages"
  "robosar_agent_bringup"
  "stage_ros_add_pose_and_crash"
  "slam_toolbox"
  "slam_toolbox_rviz"
  "robosar_controller"
  "robosar_gui"
  "robosar_navigation"
  "robosar_sim_bringup"
  "robosar_task_allocator"
  "robosar_task_generator"
  "robosar_tf"
)

# 依次编译每个包
for pkg in "${PACKAGES[@]}"; do
  echo "=============================================="
  echo "正在编译包: $pkg"
  echo "=============================================="
  
  # 使用--only-pkg-with-deps进行针对性编译
  catkin_make --only-pkg-with-deps "$pkg"
  
  # 检查编译是否成功
  if [ $? -ne 0 ]; then
    echo "错误：编译 $pkg 失败！请检查日志"
    exit 1
  fi
done

# 最终环境配置
echo "所有包编译完成！正在更新环境变量..."
source devel/setup.bash

echo "编译流程成功完成！"
