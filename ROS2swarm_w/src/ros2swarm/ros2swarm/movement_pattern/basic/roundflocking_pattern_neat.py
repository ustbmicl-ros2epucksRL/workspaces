#!/usr/bin/env python3
# flake8: noqa
"""
RoundFlocking Pattern - 使用NEAT神经网络控制机器人进行环绕任务

该模式使用预训练的NEAT神经网络来生成控制命令，实现机器人环绕目标的集群行为。
网络输入通过订阅 /global_info 话题获取全局信息，并订阅激光雷达话题检测其他智能体。
输出为线速度和角速度，映射到 u_range 范围。

观测结构（18维）：
- 智能体自身位置 (2维)
- 智能体自身速度 (2维)
- 智能体到目标的相对位置 (2维)
- 激光雷达测量值 (12维) - 360度分为12个区间，保留训练时的原始距离范围
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


def normalize_neat_output_to_signed(value, activation):
    """Map a NEAT output node value to the signed action interval."""
    if activation == 'sigmoid':
        return np.clip(value, 0.0, 1.0) * 2.0 - 1.0
    if activation == 'tanh':
        return np.clip(value, -1.0, 1.0)
    raise ValueError(f'不支持的输出激活函数: {activation}')


def sample_lidar_for_neat(ranges, n_lidar_rays, lidar_input_max_range):
    """Sample a physical scan into the raw distance range used in training.

    VMAS flocking trained this bundled genome with ``Lidar(max_range=0.2)``
    and appended those raw metre values directly to the observation.  The
    physical sensor range must therefore never be used as the NEAT input
    range.  In particular, a Burger LDS-01 reports up to 3.5 m while an
    e-puck RPLidar reports up to 12 m.
    """
    n_lidar_rays = int(n_lidar_rays)
    lidar_input_max_range = float(lidar_input_max_range)
    if n_lidar_rays <= 0:
        raise ValueError('n_lidar_rays必须大于0')
    if lidar_input_max_range <= 0.0:
        raise ValueError('lidar_input_max_range必须大于0')

    scan = np.asarray(ranges, dtype=np.float32)
    if scan.size == 0:
        return None

    scan = np.where(np.isfinite(scan), scan, lidar_input_max_range)
    scan = np.clip(scan, 0.0, lidar_input_max_range)

    sampled_ranges = []
    beams_per_sector = max(1, scan.size // n_lidar_rays)
    for ray_index in range(n_lidar_rays):
        start_index = ray_index * beams_per_sector
        middle_index = min(
            start_index + beams_per_sector // 2,
            scan.size - 1,
        )
        sampled_ranges.append(scan[middle_index])
    return np.asarray(sampled_ranges, dtype=np.float32)


class RoundFlockingPattern(MovementPattern):
    """
    使用NEAT神经网络进行环绕(Round Flocking)任务的模式节点

    通过订阅 /global_info 话题获取全局位置信息，订阅激光雷达检测其他智能体，
    使用NEAT网络计算控制命令，实现机器人环绕目标的集群行为。

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """初始化RoundFlocking模式节点"""
        super().__init__('roundflocking_pattern_neat')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('genome_file', 'flocking_genome_package_0.3.pkl'),  # 基因包文件名
                ('max_linear_velocity', 0.12),  # 最大线速度 (m/s) - 差速驱动约束
                ('max_angular_velocity', 1.0),  # 最大角速度 (rad/s) - 差速驱动约束
                ('timer_period', 0.1),  # 控制周期（秒）
                ('target_def_name', 'flocking_target'),  # 环绕目标DEF名称
                ('n_lidar_rays', 12),  # 激光雷达采样区间数
                # 这是NEAT训练输入上限，不是物理雷达量程。
                ('lidar_input_max_range', 0.2),
                ('world_scale', 1.75),  # 世界坐标归一化因子（Webots场景3.5x3.5即±1.75，VMAS训练时±1）
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
        self.target_def_name = self.get_parameter(
            'target_def_name').get_parameter_value().string_value
        self.n_lidar_rays = self.get_parameter(
            'n_lidar_rays').get_parameter_value().integer_value
        self.lidar_input_max_range = self.get_parameter(
            'lidar_input_max_range').get_parameter_value().double_value
        self.world_scale = self.get_parameter(
            'world_scale').get_parameter_value().double_value
        self.real_robot_mode = self.get_parameter(
            'real_robot_mode').get_parameter_value().bool_value
        self.data_timeout = self.get_parameter(
            'data_timeout').get_parameter_value().double_value

        # NEAT网络
        self.network = None
        self.output_activation_names = []
        # 网络输入维度：2(位置) + 2(速度) + 2(目标相对位置) + n_lidar_rays(激光雷达)
        self.network_input_dim = 6 + self.n_lidar_rays

        # 数据缓存
        self.global_info = None
        self.lidar_data = None
        self.robot_name = self.get_namespace().strip('/')
        self._missing_data_warn_counts = {}
        self._global_info_received_at = None
        self._lidar_received_at = None

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

        # 订阅激光雷达话题
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
            f'RoundFlocking模式初始化完成 [{self.robot_name}], 网络输入维度: {self.network_input_dim}, '
            f'激光雷达区间数: {self.n_lidar_rays}, '
            f'NEAT雷达输入范围: [0, {self.lidar_input_max_range}] m, '
            f'速度范围: linear=[-{self.max_linear_velocity}, {self.max_linear_velocity}], '
            f'angular=[-{self.max_angular_velocity}, {self.max_angular_velocity}]'
        )

    def _load_neat_network(self):
        """加载NEAT神经网络"""
        # 构建基因包完整路径
        package_dir = get_package_share_directory('ros2swarm')
        checkpoint_dir = os.path.join(
            package_dir, 'launch', 'pattern', 'movement_pattern', 'basic',
            'checkpoints', 'roundflocking'
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
            output_keys = neat_config.genome_config.output_keys
            self.output_activation_names = [
                genome.nodes[key].activation for key in output_keys
            ]

            self.network = RecurrentNetwork.create(genome, neat_config)
            self.network.reset()

            # 更新网络输入维度
            self.network_input_dim = len(self.network.input_nodes)

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
        sampled = sample_lidar_for_neat(
            msg.ranges,
            self.n_lidar_rays,
            self.lidar_input_max_range,
        )
        if sampled is not None:
            self.lidar_data = sampled
            self._lidar_received_at = time.monotonic()

    def _get_observation(self):
        """
        获取观测向量
        
        观测结构（18维）：
        - 智能体位置 (2维): x, y (归一化)
        - 智能体速度 (2维): vx, vy
        - 到目标的相对位置 (2维): target - robot (归一化)
        - 激光雷达测量值 (12维): 训练侧原始距离值 [0, 0.2m]
        
        注意：NEAT网络在VMAS环境训练时使用的坐标范围为 [-1, 1]，
        Webots仿真场景大小为 3.5x3.5，坐标范围为 [-1.75, 1.75]，
        需要对位置坐标进行归一化以匹配训练时的输入范围。
        
        Returns:
            list: 观测向量
        """
        if self.global_info is None:
            self.get_logger().debug('等待全局信息...')
            return None
        
        if self.lidar_data is None:
            self.get_logger().debug('等待激光雷达数据...')
            return None
        if self.real_robot_mode:
            now = time.monotonic()
            if (
                self._global_info_received_at is None
                or now - self._global_info_received_at > self.data_timeout
                or self._lidar_received_at is None
                or now - self._lidar_received_at > self.data_timeout
            ):
                self._warn_missing_data('环绕输入已过期')
                return None
        
        try:
            robots = self.global_info.get('robots', {})
            objects = self.global_info.get('objects', {})
            
            # 获取自身信息
            robot_info = robots.get(self.robot_name, None)
            if robot_info is None:
                self._warn_missing_data(f'未找到机器人 {self.robot_name} 的信息')
                return None
            
            robot_x = robot_info['position']['x']
            robot_y = robot_info['position']['y']
            robot_vx = robot_info['velocity']['x']
            robot_vy = robot_info['velocity']['y']
            
            # 获取机器人朝向（从3x3旋转矩阵提取yaw角）
            # 旋转矩阵按行优先存储：[R00, R01, R02, R10, R11, R12, R20, R21, R22]
            orientation = robot_info.get('orientation', [1, 0, 0, 0, 1, 0, 0, 0, 1])
            # yaw = atan2(R10, R00) 对于绕z轴的旋转
            robot_yaw = np.arctan2(orientation[3], orientation[0])
            
            # 获取目标信息
            target_info = objects.get(self.target_def_name, None)
            if target_info:
                # 全局坐标系下的相对位置
                target_rel_x_global = robot_x - target_info['position']['x']
                target_rel_y_global = robot_y - target_info['position']['y']
            else:
                self._warn_missing_data(
                    f'未找到环绕目标 {self.target_def_name} 的信息'
                )
                return None
            
            # 将相对位置和速度转换到机器人本体坐标系（绕z轴旋转-yaw角）
            # 这样网络看到的是：正x方向=机器人前方，正y方向=机器人左方
            cos_yaw = np.cos(-robot_yaw)
            sin_yaw = np.sin(-robot_yaw)
            
            # 转换target_rel到本体坐标系
            target_rel_x = cos_yaw * target_rel_x_global - sin_yaw * target_rel_y_global
            target_rel_y = sin_yaw * target_rel_x_global + cos_yaw * target_rel_y_global
            
            # 转换速度到本体坐标系
            robot_vx_body = cos_yaw * robot_vx - sin_yaw * robot_vy
            robot_vy_body = sin_yaw * robot_vx + cos_yaw * robot_vy
            
            # 归一化因子（将世界坐标缩放到网络训练时的范围）
            scale = self.world_scale
            
            # 构建观测向量（位置相关的维度需要归一化，使用本体坐标系）
            observation = [
                robot_x / scale,      # 智能体全局位置 x (归一化)
                robot_y / scale,      # 智能体全局位置 y (归一化)
                robot_vx_body,        # 智能体本体速度 vx
                robot_vy_body,        # 智能体本体速度 vy
                target_rel_x / scale, # 到目标的相对位置 x (本体坐标系，归一化)
                target_rel_y / scale, # 到目标的相对位置 y (本体坐标系，归一化)
            ]
            
            # 添加训练侧原始雷达距离，单位为米，范围[0, 0.2]。
            observation.extend(self.lidar_data.tolist())
            
            # 调试输出
            self.get_logger().debug(
                f'[{self.robot_name}] yaw={np.degrees(robot_yaw):.1f}°, '
                f'target_rel(body)=({target_rel_x/scale:.3f},{target_rel_y/scale:.3f}), '
                f'lidar_min={min(self.lidar_data):.2f}'
            )
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'获取观测失败: {str(e)}')
            return None

    def control_callback(self):
        """控制回调函数，使用NEAT网络计算并发布控制命令"""
        if self.network is None:
            self._warn_missing_data('NEAT网络未初始化，无法控制')
            self.command_publisher.publish(Twist())
            return

        try:
            # 获取观测
            observation = self._get_observation()
            if observation is None:
                self.command_publisher.publish(Twist())
                return
            
            # 通过NEAT网络计算输出
            output = self.network.activate(observation)

            # VMAS flocking任务的网络输出是全向移动的 [vx, vy]
            # 需要转换为差速驱动的 [linear, angular]
            # 
            # NEAT输出可能来自sigmoid或tanh，统一映射到[-1, 1]
            vx = normalize_neat_output_to_signed(
                output[0], self.output_activation_names[0]
            )  # 期望的前进方向速度
            vy = normalize_neat_output_to_signed(
                output[1], self.output_activation_names[1]
            )  # 期望的侧向速度

            
            # 将全向速度 (vx, vy) 转换为差速驱动 (linear, angular)
            # 计算期望移动的方向角和速度大小
            speed = np.sqrt(vx**2 + vy**2)
            angle = np.arctan2(vy, vx)  # 期望移动方向（本体坐标系）
            
            # 转换策略:
            # - 如果目标方向在前方 (|angle| < 90°): 可以前进 + 转向
            # - 如果目标方向在后方: 原地转向
            if abs(angle) < np.pi / 2:
                # 目标在前半部分，前进 + 转向
                linear_velocity = self.max_linear_velocity * np.cos(angle) * min(speed, 1.0)
            else:
                # 目标在后方，先原地转向
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
            self.command_publisher.publish(Twist())

    def on_swarm_start(self):
        if self.real_robot_mode and self.network is not None:
            self.network.reset()


def main(args=None):
    """创建RoundFlocking模式节点，启动并处理销毁"""
    setup_node.init_and_spin(args, RoundFlockingPattern)


if __name__ == '__main__':
    main()
