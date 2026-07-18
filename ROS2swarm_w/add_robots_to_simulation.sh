#!/bin/bash

source ./install/setup.bash

ROS_DOMAIN_ID=42 ros2 launch launch_webots add_robot.launch.py \
    start_index:=4 \
    number_robots:=3 \
    pattern:=drive_pattern \
    log_level:=info \
    robot:=burger \
    sensor_type:=lidar \
    x_start:=0.5 \
    x_dist:=0.0 \
    y_start:=0.0 \
    y_dist:=0.4 \
    random_spawn:=True \
    driving_swarm:=False

# ============================================================
# 参数说明：
# ============================================================
#
#   start_index: 机器人编号起始值（避免与已有机器人重名）
#                例如已有 robot_0 ~ robot_3，则设为 4
#   number_robots: 本次添加的机器人数量
#   pattern: 行为模式（见 start_simulation.sh 中的完整列表）
#   robot: 机器人类型（e_puck | burger）
#   sensor_type: 传感器类型（lidar | ir）
#   x_start, y_start: 第一个新机器人的位置
#   x_dist, y_dist: 每个机器人之间的位置增量
#   random_spawn: 是否随机生成位置（True | False）；False 时使用 x/y 参数
#   driving_swarm: 是否启用 driving_swarm 框架（True | False）
#
# 前提：先通过 start_simulation.sh 启动仿真。动态追加依赖其启动的
#       /global_info，来避开已有机器人、任务物体和随机障碍物。
#       默认 command_world.wbt 的生成范围为 [-1.0, 1.0]，位于 arena 内部；
#       使用含内部墙体的自定义世界时，应自行配置不与墙体相交的生成范围。
#
# 示例用法（异构群体）：
#   1. 先启动 3 个 e_puck 执行 transport_pattern:
#      bash start_simulation.sh
#   2. 再追加 2 个 burger 执行 drive_pattern:
#      修改本脚本中 robot:=burger start_index:=3 number_robots:=2
#      bash add_robots_to_simulation.sh
