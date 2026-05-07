#!/bin/bash
#    快速启动仿真脚本
#    通过预生成包含机器人的世界文件，避免使用Ros2Supervisor服务动态添加的延迟
#
#    Copyright 2020 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0

# 构建包
colcon build --symlink-install --allow-overriding communication_interfaces launch_webots ros2swarm

# 设置环境
source ./install/setup.bash

# 启动快速仿真（使用随机位置生成）
ROS_DOMAIN_ID=42 ros2 launch launch_webots create_enviroment_fast.launch.py \
    webots_world:=command_world.wbt \
    pattern:=aggregation_pattern \
    number_robots:=5 \
    total_robots:=10 \
    log_level:=info \
    robot:=e_puck \
    sensor_type:=lidar \
    random:=True \
    x_min:=-1.0 \
    x_max:=1.0 \
    y_min:=-1.0 \
    y_max:=1.0 \
    min_distance:=0.2 \
    z_height:=0.0 \
    driving_swarm:=False \
    logging:=False \
    num_obstacles:=0

# ============================================================
# 参数说明：
# ============================================================
#
# 基本参数：
#   webots_world: 世界文件模板名称
#            可用场景：
#            - command_world.wbt: 空旷的方形场地
#            - obstacle_world.wbt: 含两堵墙的障碍物环境
#   pattern: 机器人行为模式（见下方列表）
#            根据pattern名称自动配置任务环境：
#            - transport_pattern: 自动生成运输任务场景（箱子+目标点）
#            - roundflocking_pattern: 自动生成环绕任务场景（圆柱目标）
#            - navigation_pattern: 自动生成导航任务场景（每个机器人有不同颜色的可进入目标点）
#            - 其他pattern: 基础环境
#   number_robots: 机器人数量
#   total_robots: 总机器人数量（用于异构群体）
#   log_level: 日志级别（info | DEBUG）
#   robot: 机器人类型（burger | e_puck）
#   sensor_type: 传感器类型（lidar | ir | ir_tf）
#   driving_swarm: 是否使用driving_swarm框架（True | False）
#   logging: 是否记录rosbag（True | False）
#
# 位置生成参数：
#   random: 是否使用随机位置（True | False）
#   random_seed: 随机种子（可选，用于可重复的随机位置）
#   x_min, x_max: 随机区域X轴范围
#   y_min, y_max: 随机区域Y轴范围
#   min_distance: 机器人/物体之间的最小距离
#   z_height: 机器人Z轴高度
#
# 障碍物参数（任意模式下可用）：
#   num_obstacles: 随机障碍物数量（0表示不添加）
#       - 灰色方块障碍物，位置随机
#       - 所有物体位置会自动避免与机器人重叠
#
# ============================================================
# E-puck 传感器话题说明：
# ============================================================
#
# sensor_type:=lidar 时：
#   - Webots内置Lidar设备发布 -> /robot_X/scan (LaserScan)
#   - epuck_node红外拼接发布 -> /robot_X/laser_scan (LaserScan)
#
# sensor_type:=ir 时：
#   - epuck_node红外拼接发布 -> /robot_X/laser_scan (LaserScan)
#   - lidar设备不工作（机器人上没有安装）
#
# ============================================================
# 行为模式列表：
# ============================================================
#
# 移动模式：
#   * drive_pattern
#   * dispersion_pattern 
#   * aggregation_pattern 
#   * attraction_pattern 
#   * attraction_pattern2
#   * magnetometer_pattern
#   * minimalist_flocking_pattern
#   * rat_search_pattern 
#   * flocking_pattern 
#   * flocking_pattern2 
#   * random_walk_pattern
#   * discussed_dispersion_pattern
#   * transport_pattern        <- 自动生成运输任务场景（箱子+目标点）
#   * roundflocking_pattern    <- 自动生成环绕任务场景（圆柱目标）
#   * navigation_pattern       <- 自动生成导航任务场景（每个机器人有不同颜色的目标点）
#
# 投票模式：
#   * voter_model_pattern
#   * voter_model_with_limiter_pattern
#   * majority_rule_pattern
#
# ============================================================
