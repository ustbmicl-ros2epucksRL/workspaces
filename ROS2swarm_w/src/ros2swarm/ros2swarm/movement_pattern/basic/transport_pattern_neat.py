#!/usr/bin/env python3
# flake8: noqa
"""
Transport Pattern - 使用NEAT神经网络控制机器人进行运输任务

该模式使用预训练的NEAT神经网络来生成控制命令，实现机器人协作运输物体的行为。
网络输入通过订阅 /global_info 话题获取全局信息（机器人状态、包裹状态、目标位置）。
输出为线速度和角速度，映射到 u_range 范围。

观测结构（11维，对应1个包裹）：
- 智能体自身状态（4维）：位置(2维) + 速度(2维)
- 包裹信息（7维）：
  - 包裹到目标的相对位置(2维)
  - 包裹到智能体的相对位置(2维)
  - 包裹的速度(2维)
  - 包裹是否在目标上(1维)
"""
import os
import pickle
import numpy as np
import json

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from ros2swarm.utils import setup_node
from ros2swarm.movement_pattern.movement_pattern import MovementPattern
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

try:
    from neat.nn import RecurrentNetwork
    NEAT_AVAILABLE = True
except ImportError:
    NEAT_AVAILABLE = False


class TransportPattern(MovementPattern):
    """
    使用NEAT神经网络进行运输(Transport)任务的模式节点

    通过订阅 /global_info 话题获取全局信息，计算观测向量，
    使用NEAT网络计算控制命令，实现机器人协作运输物体到目标位置。

    pattern_node >> publishing to the self.get_namespace()/drive_command topic
    """

    def __init__(self):
        """初始化Transport模式节点"""
        super().__init__('transport_pattern_neat')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('genome_file', 'transport_genome_package.pkl'),  # 基因包文件名
                ('max_linear_velocity', 0.12),  # 最大线速度 (m/s) - 差速驱动约束
                ('max_angular_velocity', 1.0),  # 最大角速度 (rad/s) - 差速驱动约束
                ('timer_period', 0.1),  # 控制周期（秒）
                ('box_def_name', 'transport_box'),  # 包裹DEF名称
                ('target_def_name', 'target_zone'),  # 目标区域DEF名称
                ('world_scale', 1.75),  # 世界坐标归一化因子（Webots场景3.5x3.5即±1.75，VMAS训练时±1）
                ('goal_threshold', 0.125),  # 判断包裹是否在目标上的距离阈值（米）
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
        self.box_def_name = self.get_parameter(
            'box_def_name').get_parameter_value().string_value
        self.target_def_name = self.get_parameter(
            'target_def_name').get_parameter_value().string_value
        self.world_scale = self.get_parameter(
            'world_scale').get_parameter_value().double_value
        self.goal_threshold = self.get_parameter(
            'goal_threshold').get_parameter_value().double_value

        # NEAT网络
        self.network = None
        self.network_input_dim = 11  # 4 (自身状态) + 7 (1个包裹信息)

        # 全局信息缓存
        self.global_info = None
        self.robot_name = self.get_namespace().strip('/')
        self._missing_data_warn_counts = {}
        self._distance_to_target = None
        self._goal_reached = False

        # 加载NEAT网络
        if not NEAT_AVAILABLE:
            self.get_logger().error('neat-python库未安装! 请运行: pip install neat-python')
        else:
            self._load_neat_network()

        # 订阅全局信息话题
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        self.global_info_sub = self.create_subscription(
            String,
            '/global_info',
            self._global_info_callback,
            qos
        )
        self.get_logger().debug(f'订阅全局信息话题: /global_info')

        # 定时器用于处理和发布控制命令
        self.timer = self.create_timer(
            self.timer_period,
            self.swarm_command_controlled_timer(self.control_callback)
        )

        self.get_logger().debug(
            f'Transport模式初始化完成 [{self.robot_name}], 网络输入维度: {self.network_input_dim}, '
            f'速度范围: linear=[-{self.max_linear_velocity}, {self.max_linear_velocity}], '
            f'angular=[-{self.max_angular_velocity}, {self.max_angular_velocity}]'
        )

    def _load_neat_network(self):
        """加载NEAT神经网络"""
        # 构建基因包完整路径
        package_dir = get_package_share_directory('ros2swarm')
        checkpoint_dir = os.path.join(
            package_dir, 'launch', 'pattern', 'movement_pattern', 'basic',
            'checkpoints', 'transport'
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
        except json.JSONDecodeError as e:
            self.get_logger().error(f'解析全局信息失败: {e}')

    def _warn_missing_data(self, message: str):
        count = self._missing_data_warn_counts.get(message, 0) + 1
        self._missing_data_warn_counts[message] = count
        if count == 1:
            self.get_logger().warn(message)
        else:
            self.get_logger().debug(message)

    def _get_observation(self):
        """
        获取观测向量（从全局信息或随机生成）
        
        观测结构（11维）：
        - 智能体位置 (2维): x, y (归一化)
        - 智能体速度 (2维): vx, vy
        - 包裹到目标的相对位置 (2维) (归一化)
        - 智能体到包裹的相对位置 (2维) (归一化) - 注意：方向是 box - robot
        - 包裹速度 (2维)
        - 包裹是否在目标上 (1维)
        
        注意：NEAT网络在VMAS环境训练时使用的观测值范围较小（约±0.5），
        需要对ROS2仿真的世界坐标进行归一化以匹配训练时的输入范围。
        
        Returns:
            list: 观测向量
        """
        self._distance_to_target = None
        if self.global_info is None:
            self.get_logger().debug('等待全局信息...')
            return self._get_random_observation()
        
        try:
            robots = self.global_info.get('robots', {})
            objects = self.global_info.get('objects', {})
            
            # 获取自身信息
            robot_info = robots.get(self.robot_name, None)
            if robot_info is None:
                self._warn_missing_data(f'未找到机器人 {self.robot_name} 的信息')
                return self._get_random_observation()
            
            robot_x = robot_info['position']['x']
            robot_y = robot_info['position']['y']
            robot_vx = robot_info['velocity']['x']
            robot_vy = robot_info['velocity']['y']
            
            # 获取机器人朝向（从3x3旋转矩阵提取yaw角）
            # 旋转矩阵按行优先存储：[R00, R01, R02, R10, R11, R12, R20, R21, R22]
            orientation = robot_info.get('orientation', [1, 0, 0, 0, 1, 0, 0, 0, 1])
            # yaw = atan2(R10, R00) 对于绕z轴的旋转
            robot_yaw = np.arctan2(orientation[3], orientation[0])
            
            # 获取包裹信息
            box_info = objects.get(self.box_def_name, None)
            if box_info:
                box_x = box_info['position']['x']
                box_y = box_info['position']['y']
                box_vx = box_info['velocity']['x']
                box_vy = box_info['velocity']['y']
            else:
                box_x, box_y = 0.0, 0.0
                box_vx, box_vy = 0.0, 0.0
            
            # 获取目标信息
            target_info = objects.get(self.target_def_name, None)
            if target_info:
                target_x = target_info['position']['x']
                target_y = target_info['position']['y']
            else:
                target_x, target_y = 0.0, 0.0
            
            # 计算全局坐标系下的相对位置
            box_to_target_x_global = box_x - target_x
            box_to_target_y_global = box_y - target_y
            robot_to_box_x_global = box_x - robot_x
            robot_to_box_y_global = box_y - robot_y
            
            # 将相对位置转换到机器人本体坐标系（绕z轴旋转-yaw角）
            # 这样网络看到的是：正x方向=机器人前方，正y方向=机器人左方
            cos_yaw = np.cos(-robot_yaw)
            sin_yaw = np.sin(-robot_yaw)
            
            # 转换robot_to_box到本体坐标系
            robot_to_box_x = cos_yaw * robot_to_box_x_global - sin_yaw * robot_to_box_y_global
            robot_to_box_y = sin_yaw * robot_to_box_x_global + cos_yaw * robot_to_box_y_global
            
            # 转换box_to_target到本体坐标系（可选，但保持一致性）
            box_to_target_x = cos_yaw * box_to_target_x_global - sin_yaw * box_to_target_y_global
            box_to_target_y = sin_yaw * box_to_target_x_global + cos_yaw * box_to_target_y_global
            
            # 转换速度到本体坐标系
            robot_vx_body = cos_yaw * robot_vx - sin_yaw * robot_vy
            robot_vy_body = sin_yaw * robot_vx + cos_yaw * robot_vy
            box_vx_body = cos_yaw * box_vx - sin_yaw * box_vy
            box_vy_body = sin_yaw * box_vx + cos_yaw * box_vy
            
            # 判断包裹是否在目标上
            # 计算包裹中心到目标中心的距离
            # 阈值应小于 (目标半径 - 包裹半边长)，确保包裹完全在目标区域内
            # 默认：目标半径0.25m，包裹边长0.25m，阈值0.125m
            distance_to_target = np.sqrt(box_to_target_x_global**2 + box_to_target_y_global**2)
            self._distance_to_target = float(distance_to_target)
            on_goal = 1.0 if distance_to_target < self.goal_threshold else 0.0
            
            # 归一化因子（将世界坐标缩放到网络训练时的范围）
            scale = self.world_scale
            
            # 构建观测向量（位置相关的维度需要归一化，使用本体坐标系）
            observation = [
                robot_x / scale,           # 智能体全局位置 x (归一化)
                robot_y / scale,           # 智能体全局位置 y (归一化)
                robot_vx_body,             # 智能体本体速度 vx
                robot_vy_body,             # 智能体本体速度 vy
                box_to_target_x / scale,   # 包裹到目标 x (本体坐标系，归一化)
                box_to_target_y / scale,   # 包裹到目标 y (本体坐标系，归一化)
                robot_to_box_x / scale,    # 智能体到包裹 x (本体坐标系，归一化)
                robot_to_box_y / scale,    # 智能体到包裹 y (本体坐标系，归一化)
                box_vx_body,               # 包裹本体速度 vx
                box_vy_body,               # 包裹本体速度 vy
                on_goal,                   # 是否在目标上
            ]
            
            # 调试输出
            self.get_logger().debug(
                f'[{self.robot_name}] yaw={np.degrees(robot_yaw):.1f}°, '
                f'robot_to_box(body)=({robot_to_box_x/scale:.3f},{robot_to_box_y/scale:.3f})'
            )
            
            return observation
            
        except Exception as e:
            self.get_logger().error(f'获取观测失败: {str(e)}')
            return self._get_random_observation()

    def _get_random_observation(self):
        """生成随机观测向量用于测试"""
        return np.random.uniform(-1.0, 1.0, self.network_input_dim).tolist()

    def control_callback(self):
        """控制回调函数，使用NEAT网络计算并发布控制命令"""
        if self.network is None:
            self._warn_missing_data('NEAT网络未初始化，无法控制')
            self._publish_stop()
            return

        try:
            # 获取观测
            observation = self._get_observation()

            if self._goal_reached or (
                self._distance_to_target is not None
                and self._distance_to_target < self.goal_threshold
            ):
                if not self._goal_reached:
                    self.get_logger().info(
                        'Transport NEAT goal reached '
                        f'at distance={self._distance_to_target:.3f}; '
                        'stopping network commands'
                    )
                self._goal_reached = True
                self._publish_stop()
                return
            
            # 通过NEAT网络计算输出
            output = self.network.activate(observation)

            # 将输出映射到动作空间
            # NEAT输出在[0, 1]范围（sigmoid激活）
            # 映射到 u_range: [-max, max]
            linear_velocity = (output[0] * 2 - 1) * self.max_linear_velocity
            angular_velocity = (output[1] * 2 - 1) * self.max_angular_velocity

            # 创建并发布Twist消息
            twist_msg = Twist()
            twist_msg.linear.x = float(linear_velocity)
            twist_msg.angular.z = float(angular_velocity)

            self.command_publisher.publish(twist_msg)

            self.get_logger().debug(
                f'[{self.robot_name}] 控制输出 - 线速度: {linear_velocity:.4f} m/s, '
                f'角速度: {angular_velocity:.4f} rad/s'
            )

        except Exception as e:
            self.get_logger().error(f'NEAT网络推理失败: {str(e)}')
            self._publish_stop()

    def _publish_stop(self):
        self.command_publisher.publish(Twist())




def main(args=None):
    """创建Transport模式节点，启动并处理销毁"""
    setup_node.init_and_spin(args, TransportPattern)


if __name__ == '__main__':
    main()
