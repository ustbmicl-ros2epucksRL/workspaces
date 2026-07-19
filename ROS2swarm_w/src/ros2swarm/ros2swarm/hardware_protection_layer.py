#!/usr/bin/env python3
#    Copyright 2020 Marian Begemann
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#        http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ros2swarm.abstract_pattern import AbstractPattern
from ros2swarm.utils import setup_node
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions


def limit_twist_to_wheel_speed(linear_velocity, angular_velocity,
                               wheel_separation, max_wheel_linear_velocity):
    """Scale a differential-drive command to a joint wheel-speed limit.

    Scaling both components by the same factor preserves the avoidance path's
    curvature.  A non-positive wheel limit leaves legacy platforms unchanged.
    """
    linear_velocity = float(linear_velocity)
    angular_velocity = float(angular_velocity)
    wheel_separation = float(wheel_separation)
    max_wheel_linear_velocity = float(max_wheel_linear_velocity)
    if wheel_separation <= 0.0 or max_wheel_linear_velocity <= 0.0:
        return linear_velocity, angular_velocity

    half_track = wheel_separation / 2.0
    left_velocity = linear_velocity - angular_velocity * half_track
    right_velocity = linear_velocity + angular_velocity * half_track
    requested_max = max(abs(left_velocity), abs(right_velocity))
    if requested_max <= max_wheel_linear_velocity:
        return linear_velocity, angular_velocity

    scale = max_wheel_linear_velocity / requested_max
    return linear_velocity * scale, angular_velocity * scale


class HardwareProtectionLayer(AbstractPattern):
    """
    Protects the robot from crashing into obstacles.
    用于实现避免机器人与障碍物碰撞的保护机制

    pattern_node >> HardwareProtectionLayer >> publish twist message

    This is performed by listening to the self.get_namespace()/drive_command topic and
    publishing the message or an adjusted one in the case of
    obstacles to self.get_namespace()/cmd_vel
    该节点会监听机器人发送的移动命令，并根据传感器数据判断是否需要对命令进行调整碰撞。
    """

    def __init__(self):
        """Initialize the node."""
        # 初始化硬件保护层节点
        super().__init__('hardware_protection_layer')

        # 声明一些参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('hardware_protection_layer_max_range', 0.0),   # 最大探测距离
                ('hardware_protection_layer_min_range', 0.0),   # 最小探测距离
                ('hardware_protection_layer_front_attraction', 0.0),# 前方吸引因子
                ('hardware_protection_layer_threshold', 0), # 阈值，用于检测障碍物的条件
                ('max_translational_velocity', 0.0),    # 最大平移速度
                ('max_rotational_velocity', 0.0),    # 最大旋转速度
                ('wheel_separation', 0.0),
                ('max_wheel_linear_velocity', 0.0),
            ])

        # 订阅机器人移动的命令（drive_command 话题）
        self.command_subscription = self.create_subscription(
            Twist,
            self.get_namespace() + '/drive_command',
            self.swarm_command_controlled(self.command_callback),
            10)

        # 当前的传感器角度和距离（用于避障）
        self.current_angles = None
        self.current_ranges = None
        self.has_received_drive_command = False
        
        # 订阅传感器数据（range_data 话题）
        self.range_data_subscription = self.create_subscription(
            RangeData,
            self.get_namespace() + '/range_data',
            self.swarm_command_controlled(self.range_data_callback),
            qos_profile=qos_profile_sensor_data
        )
        
        # 创建发布器，发布经过调整后的速度命令（cmd_vel 话题）
        self.publisher_cmd_vel = self.create_publisher(Twist,
                                                       self.get_namespace() + '/cmd_vel',
                                                       10)

        # 获取各个参数的值
        self.param_max_range = float(self.get_parameter(
            "hardware_protection_layer_max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "hardware_protection_layer_min_range").get_parameter_value().double_value
        self.param_front_attraction = self.get_parameter(
            "hardware_protection_layer_front_attraction").get_parameter_value().double_value
        self.param_threshold = self.get_parameter(
            "hardware_protection_layer_threshold").get_parameter_value().integer_value
        self.param_max_translational_velocity = self.get_parameter(
            "max_translational_velocity").get_parameter_value().double_value
        self.param_max_rotational_velocity = self.get_parameter(
            "max_rotational_velocity").get_parameter_value().double_value
        self.param_wheel_separation = self.get_parameter(
            "wheel_separation").get_parameter_value().double_value
        self.param_max_wheel_linear_velocity = self.get_parameter(
            "max_wheel_linear_velocity").get_parameter_value().double_value

    def destroy_node(self):
        """Send a stop twist message and calls the super destroy method."""
        # 发送一个停止命令，并调用父类的 destroy_node 方法
        self.publisher_cmd_vel.publish(Twist())
        super().destroy_node()

    def vector_calc(self):
        """
        计算是否需要进行避障，并返回对应的避障向量。
        Returns -> [avoid_needed{boolean}, direction{Twist}]：返回两个值,一个表示是否需要避障,另一个是避障方向(Twist类型)
        """
        # 如果没有当前的距离数据，返回 False，表示不需要避障
        if self.current_ranges is None:
            return [False, None]

        # 设置避障的最大距离
        avoid_distance = self.param_max_range
        # 计算避障向量
        direction, obstacle_free = ScanCalculationFunctions.potential_field(self.param_front_attraction,
                                                     avoid_distance,
                                                     self.param_max_rotational_velocity,
                                                     self.param_max_translational_velocity,
                                                     self.param_min_range,
                                                     self.param_threshold,
                                                     self.current_ranges,
                                                     self.angles)
        limited_linear, limited_angular = limit_twist_to_wheel_speed(
            direction.linear.x,
            direction.angular.z,
            self.param_wheel_separation,
            self.param_max_wheel_linear_velocity,
        )
        direction.linear.x = limited_linear
        direction.angular.z = limited_angular
        # 如果没有障碍物，obstacle_free 为 True，不需要避障；否则需要避障
        avoid_needed = not obstacle_free

        return [avoid_needed, direction]

    def command_callback(self, msg):
        """
        Publish the received message or adjusts it.
        接收到速度命令时，判断是否需要调整。

        If it is needed to avoid an obstacle the avoid Twist message is published,
        the given command otherwise
        如果需要避障则发布避障方向的 Twist 消息，否则发布原来的消息。
        """
        self.get_logger().debug('heard: "%s"' % msg)
        self.has_received_drive_command = True

        # 计算是否需要避障和避障方向
        [adjust, direction] = self.vector_calc()

        # 如果需要调整，使用避障方向替代原来的命令
        if adjust:
            msg = direction
            self.get_logger().debug('Adjusting to"%s"' % direction)

        # 发布消息到 cmd_vel 话题
        self.publisher_cmd_vel.publish(msg)

    def range_data_callback(self, msg):
        """
        Check for every received scan if it is needed to avoid an obstacle.
        每次收到传感器数据时，检查是否需要避障。

        If needed publish avoid message, do nothing otherwise.
         如果需要避障，则发布避障消息，否则不做任何操作。
        """
        # 更新当前传感器的距离和角度数据
        self.current_ranges = msg.ranges
        self.angles = msg.angles

        self.get_logger().debug('heard: "%s"' % msg)

        if not self.has_received_drive_command:
            return

        [adjust, direction] = self.vector_calc()
        
        if adjust:
            self.publisher_cmd_vel.publish(direction)
            self.get_logger().debug('Adjusting to"%s"' % direction)

def main(args=None):
    setup_node.init_and_spin(args, HardwareProtectionLayer)


if __name__ == '__main__':
    main()
