#!/usr/bin/env python3
# flake8: noqa
"""
Navigation Pattern - 使用NEAT神经网络控制机器人进行导航任务

该模式使用预训练的NEAT神经网络来生成控制命令，实现机器人导航到各自目标点的行为。
网络输入通过订阅 /global_info 话题获取全局信息（机器人状态、目标位置）。
输出为线速度和角速度，映射到 u_range 范围。

观测结构（18维，基于VMAS navigation任务）：
- 智能体自身位置 (2维): x, y (归一化)
- 智能体自身速度 (2维): vx, vy
- 到目标的相对位置 (2维): 机器人到自己目标点的相对位置
- 激光雷达测量值 (12维): 12个方向的障碍物距离

每个机器人有自己的专属目标点（不同颜色），目标点是可进入的。
"""
import os
import pickle
import numpy as np
import json
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from ros2swarm.utils import setup_node
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, qos_profile_sensor_data

try:
    from neat.nn import RecurrentNetwork
    NEAT_AVAILABLE = True
except ImportError:
    NEAT_AVAILABLE = False


class NavigationPattern(MovementPattern):
    """
    使用NEAT神经网络进行导航(Navigation)任务的模式节点

    通过订阅 /global_info 话题获取全局信息，计算观测向量，
    使用NEAT网络计算控制命令，实现机器人导航到各自目标点。

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """初始化Navigation模式节点"""
        super().__init__('navigation_pattern_neat')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('genome_file', 'navigation_genome_package.pkl'),  # 基因包文件名
                ('max_linear_velocity', 0.12),  # 最大线速度 (m/s) - 差速驱动约束
                ('max_angular_velocity', 1.0),  # 最大角速度 (rad/s) - 差速驱动约束
                ('timer_period', 0.1),  # 控制周期（秒）
                ('target_def_prefix', 'navigation_target'),  # 目标点DEF名称前缀
                ('world_scale', 1.75),  # 世界坐标归一化因子
                ('goal_threshold', 0.1),  # 判断是否到达目标的距离阈值（米）
                ('collisions', True),  # 是否启用碰撞检测（添加激光雷达观测）
                ('n_lidar_rays', 12),  # 激光雷达采样区间数
                ('max_range', 0.2),  # 激光雷达最大范围
                ('real_robot_mode', False),
                ('data_timeout', 0.30),
            ])

        # 获取参数值
        self.genome_file = self.get_parameter(
            'genome_file').get_parameter_value().string_value
        self.max_linear_velocity = self.get_parameter(
            'max_linear_velocity').get_parameter_value().double_value
        self.max_angular_velocity = self.get_parameter(
            'max_angular_velocity').get_parameter_value().double_value
        self.timer_period = self.get_parameter(
            'timer_period').get_parameter_value().double_value
        self.target_def_prefix = self.get_parameter(
            'target_def_prefix').get_parameter_value().string_value
        self.world_scale = self.get_parameter(
            'world_scale').get_parameter_value().double_value
        self.goal_threshold = self.get_parameter(
            'goal_threshold').get_parameter_value().double_value
        self.collisions = self.get_parameter(
            'collisions').get_parameter_value().bool_value
        self.n_lidar_rays = self.get_parameter(
            'n_lidar_rays').get_parameter_value().integer_value
        self.max_range = self.get_parameter(
            'max_range').get_parameter_value().double_value
        self.real_robot_mode = self.get_parameter(
            'real_robot_mode').get_parameter_value().bool_value
        self.data_timeout = self.get_parameter(
            'data_timeout').get_parameter_value().double_value

        # NEAT网络
        self.network = None
        # 网络输入维度将在加载网络后确定
        self.network_input_dim = 6  # 初始估计：位置(2) + 速度(2) + 目标相对位置(2)

        # 数据缓存
        self.global_info = None
        self.lidar_data = None
        self.robot_name = self.get_namespace().strip('/')
        self._missing_data_warn_counts = {}
        self._global_info_received_at = None
        self._lidar_received_at = None
        self._distance_to_target = None
        self._goal_reached = False
        
        # 从机器人名称中提取ID
        self.robot_id = self._extract_robot_id(self.robot_name)
        self.target_def_name = f'{self.target_def_prefix}_{self.robot_id}'

        # 加载NEAT网络
        if not NEAT_AVAILABLE:
            self.get_logger().error('neat-python库未安装! 请运行: pip install neat-python')
        else:
            self._load_neat_network()

        # QoS 配置
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # 订阅全局信息话题
        self.global_info_sub = self.create_subscription(
            String,
            '/global_info',
            self._global_info_callback,
            qos
        )
        self.get_logger().debug(f'订阅全局信息话题: /global_info')

        # 如果启用碰撞检测，订阅激光雷达话题
        if self.collisions:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                self.get_namespace() + '/scan',
                self._lidar_callback,
                qos_profile_sensor_data
            )
            self.get_logger().debug(f'订阅激光雷达话题: {self.get_namespace()}/scan')

        # 定时器用于处理和发布控制命令
        self.timer = self.create_timer(
            self.timer_period,
            self.swarm_command_controlled_timer(self.control_callback)
        )

        self.get_logger().debug(
            f'Navigation模式初始化完成 [{self.robot_name}], 目标: {self.target_def_name}, '
            f'网络输入维度: {self.network_input_dim}, collisions: {self.collisions}, '
            f'速度范围: linear=[-{self.max_linear_velocity}, {self.max_linear_velocity}], '
            f'angular=[-{self.max_angular_velocity}, {self.max_angular_velocity}]'
        )

    def _extract_robot_id(self, robot_name: str) -> int:
        """从机器人名称中提取ID"""
        try:
            # 假设格式为 "robot_X"
            return int(robot_name.split('_')[-1])
        except (ValueError, IndexError):
            self.get_logger().warn(f'无法从 {robot_name} 提取ID，默认使用0')
            return 0

    def _load_neat_network(self):
        """加载NEAT神经网络"""
        # 构建基因包完整路径
        package_dir = get_package_share_directory('ros2swarm')
        checkpoint_dir = os.path.join(
            package_dir, 'launch', 'pattern', 'movement_pattern', 'basic',
            'checkpoints', 'navigation'
        )
        
        # 如果genome_file是完整路径则直接使用，否则在checkpoint目录下查找
        if os.path.isabs(self.genome_file):
            self.genome_package_path = self.genome_file
        else:
            self.genome_package_path = os.path.join(checkpoint_dir, self.genome_file)

        self.get_logger().debug(f'加载NEAT基因组包: {self.genome_package_path}')

        try:
            with open(self.genome_package_path, 'rb') as f:
                package = pickle.load(f)

            genome = package['genome']
            neat_config = package['neat_config']

            self.network = RecurrentNetwork.create(genome, neat_config)
            self.network.reset()

            # 更新网络输入维度
            self.network_input_dim = len(self.network.input_nodes)

            self.get_logger().debug(
                f'NEAT网络加载成功 - 任务: {package.get("task_name", "unknown")}, '
                f'适应度: {package.get("fitness", 0):.4f}, '
                f'输入维度: {self.network_input_dim}, '
                f'输出维度: {len(self.network.output_nodes)}'
            )
        except FileNotFoundError:
            self.get_logger().error(f'NEAT基因组包不存在: {self.genome_package_path}')
        except Exception as e:
            self.get_logger().error(f'加载NEAT网络失败: {str(e)}')

    def _global_info_callback(self, msg: String):
        """接收全局信息"""
        try:
            self.global_info = json.loads(msg.data)
            self._global_info_received_at = time.monotonic()
        except json.JSONDecodeError as e:
            self.get_logger().error(f'解析全局信息失败: {e}')

    def _warn_missing_data(self, message: str):
        count = self._missing_data_warn_counts.get(message, 0) + 1
        self._missing_data_warn_counts[message] = count
        if count == 1:
            self.get_logger().warn(message)
        else:
            self.get_logger().debug(message)

    def _lidar_callback(self, msg: LaserScan):
        """
        接收激光雷达数据，将360度分为n_lidar_rays个区间，
        取每个区间中间光束的测量值
        """
        ranges = np.array(msg.ranges, dtype=np.float32)
        num_beams = len(ranges)
        
        if num_beams == 0:
            return
        
        # 处理无效值
        ranges = np.where(np.isinf(ranges), self.max_range, ranges)
        ranges = np.where(np.isnan(ranges), self.max_range, ranges)
        ranges = np.clip(ranges, 0, self.max_range)
        
        # 将360度分为n_lidar_rays个区间，取每个区间中间光束的值
        sampled_ranges = []
        beams_per_sector = num_beams // self.n_lidar_rays
        
        for i in range(self.n_lidar_rays):
            start_idx = i * beams_per_sector
            mid_idx = start_idx + beams_per_sector // 2
            mid_idx = min(mid_idx, num_beams - 1)
            sampled_ranges.append(ranges[mid_idx])
        
        self.lidar_data = np.array(sampled_ranges)
        self._lidar_received_at = time.monotonic()

    def _get_observation(self):
        """
        获取观测向量（基于VMAS navigation任务的观测结构）
        
        观测结构（18维）：
        - 智能体位置 (2维): x, y (归一化)
        - 智能体速度 (2维): vx, vy
        - 到目标的相对位置 (2维)
        - 激光雷达测量值 (12维，当collisions=True时)
        
        Returns:
            list: 观测向量
        """
        if self.global_info is None:
            self.get_logger().debug('等待全局信息...')
            if self.real_robot_mode:
                return None
            return self._get_random_observation()
        
        if self.collisions and self.lidar_data is None:
            self.get_logger().debug('等待激光雷达数据...')
            if self.real_robot_mode:
                return None
            return self._get_random_observation()
        if self.real_robot_mode:
            now = time.monotonic()
            if (
                self._global_info_received_at is None
                or now - self._global_info_received_at > self.data_timeout
                or (
                    self.collisions
                    and (
                        self._lidar_received_at is None
                        or now - self._lidar_received_at > self.data_timeout
                    )
                )
            ):
                self._warn_missing_data('导航输入已过期')
                return None
        
        try:
            robots = self.global_info.get('robots', {})
            objects = self.global_info.get('objects', {})
            
            # 获取自身信息
            robot_info = robots.get(self.robot_name, None)
            if robot_info is None:
                self._warn_missing_data(f'未找到机器人 {self.robot_name} 的信息')
                if self.real_robot_mode:
                    return None
                return self._get_random_observation()
            
            robot_x = robot_info['position']['x']
            robot_y = robot_info['position']['y']
            robot_vx = robot_info['velocity']['x']
            robot_vy = robot_info['velocity']['y']
            
            # 获取机器人朝向（从3x3旋转矩阵提取yaw角）
            orientation = robot_info.get('orientation', [1, 0, 0, 0, 1, 0, 0, 0, 1])
            robot_yaw = np.arctan2(orientation[3], orientation[0])
            
            # 归一化因子
            scale = self.world_scale
            
            # 将速度转换到机器人本体坐标系
            cos_yaw = np.cos(-robot_yaw)
            sin_yaw = np.sin(-robot_yaw)
            robot_vx_body = cos_yaw * robot_vx - sin_yaw * robot_vy
            robot_vy_body = sin_yaw * robot_vx + cos_yaw * robot_vy
            
            # 获取到目标的相对位置
            goal_poses = []
            target_info = objects.get(self.target_def_name, None)
            if target_info:
                # 计算到目标的相对位置（全局坐标系）
                rel_x_global = robot_x - target_info['position']['x']
                rel_y_global = robot_y - target_info['position']['y']
                
                # 转换到本体坐标系
                rel_x = cos_yaw * rel_x_global - sin_yaw * rel_y_global
                rel_y = sin_yaw * rel_x_global + cos_yaw * rel_y_global
                
                goal_poses.extend([rel_x / scale, rel_y / scale])
                self._distance_to_target = float(np.hypot(
                    rel_x_global,
                    rel_y_global,
                ))
            else:
                self._warn_missing_data(f'未找到目标 {self.target_def_name}')
                self._distance_to_target = None
                if self.real_robot_mode:
                    return None
                goal_poses.extend([0.0, 0.0])
            
            # 构建观测向量（匹配VMAS navigation任务的观测结构）
            observation = [
                robot_x / scale,      # 智能体全局位置 x (归一化)
                robot_y / scale,      # 智能体全局位置 y (归一化)
                robot_vx_body,        # 智能体本体速度 vx
                robot_vy_body,        # 智能体本体速度 vy
            ]
            
            # 添加目标位置信息
            observation.extend(goal_poses)
            
            # 如果启用碰撞检测，添加激光雷达数据
            if self.collisions and self.lidar_data is not None:
                # VMAS中的观测是 max_range - measure()，即距离障碍物的"空闲空间"
                lidar_obs = self.max_range - self.lidar_data
                observation.extend(lidar_obs.tolist())
            
            # 调试输出
            self.get_logger().debug(
                f'[{self.robot_name}] yaw={np.degrees(robot_yaw):.1f}°, '
                f'target_rel=({goal_poses[0] if goal_poses else 0:.3f},{goal_poses[1] if len(goal_poses) > 1 else 0:.3f}), '
                f'obs_dim={len(observation)}'
            )
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'获取观测失败: {str(e)}')
            if self.real_robot_mode:
                return None
            return self._get_random_observation()

    def _get_random_observation(self):
        """生成随机观测向量用于测试"""
        return np.random.uniform(-1.0, 1.0, self.network_input_dim).tolist()

    def control_callback(self):
        """控制回调函数，使用NEAT网络计算并发布控制命令"""
        if self.network is None:
            self._warn_missing_data('NEAT网络未初始化，无法控制')
            if self.real_robot_mode:
                self.command_publisher.publish(Twist())
            return

        try:
            # 获取观测
            observation = self._get_observation()
            if observation is None:
                self.command_publisher.publish(Twist())
                return
            if self.real_robot_mode and (
                self._goal_reached
                or (
                    self._distance_to_target is not None
                    and self._distance_to_target <= self.goal_threshold
                )
            ):
                self._goal_reached = True
                self.command_publisher.publish(Twist())
                return
            
            # 通过NEAT网络计算输出
            output = self.network.activate(observation)

            # VMAS navigation任务的网络输出是全向移动的 [vx, vy]
            # 需要转换为差速驱动的 [linear, angular]
            # 
            # NEAT输出在[0, 1]范围（sigmoid激活），映射到[-1, 1]
            vx = output[0] * 2 - 1  # 期望的前进方向速度
            vy = output[1] * 2 - 1  # 期望的侧向速度
            
            # 将全向速度 (vx, vy) 转换为差速驱动 (linear, angular)
            speed = np.sqrt(vx**2 + vy**2)
            angle = np.arctan2(vy, vx)  # 期望移动方向（本体坐标系）
            
            # 转换策略:
            # - 如果目标方向在前方 (|angle| < 90°): 可以前进 + 转向
            # - 如果目标方向在后方: 原地转向
            if abs(angle) < np.pi / 2:
                linear_velocity = self.max_linear_velocity * np.cos(angle) * min(speed, 1.0)
            else:
                linear_velocity = 0.0
            
            # 角速度与期望方向角成正比
            angular_velocity = self.max_angular_velocity * np.clip(angle / (np.pi / 2), -1, 1)

            # 创建并发布Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = float(linear_velocity)
            twist_msg.angular.z = float(angular_velocity)

            self.command_publisher.publish(twist_msg)

            self.get_logger().debug(
                f'[{self.robot_name}] 网络输出 vx={vx:.2f}, vy={vy:.2f} -> '
                f'linear={linear_velocity:.4f} m/s, angular={angular_velocity:.4f} rad/s'
            )

        except Exception as e:
            self.get_logger().error(f'NEAT网络推理失败: {str(e)}')
            if self.real_robot_mode:
                self.command_publisher.publish(Twist())

    def on_swarm_start(self):
        if not self.real_robot_mode:
            return
        self._goal_reached = False
        self._distance_to_target = None
        if self.network is not None:
            self.network.reset()


def main(args=None):
    """创建Navigation模式节点，启动并处理销毁"""
    setup_node.init_and_spin(args, NavigationPattern)


if __name__ == '__main__':
    main()
