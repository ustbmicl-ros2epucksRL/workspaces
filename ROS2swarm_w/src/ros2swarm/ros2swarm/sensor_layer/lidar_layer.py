#!/usr/bin/env python3
#    Copyright 2020 Josephine Brauer, Tanja Kaiser
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
from ros2swarm.utils import setup_node
from sensor_msgs.msg import LaserScan
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data
from ros2swarm.utils.scan_calculation_functions import ScanCalculationFunctions, ReductionOption

class LidarLayer(Node):
    """
    订阅激光雷达的 LaserScan 话题

    将激光雷达数据处理后发布为 RangeData 消息到 /range_data 话题

    提供一个服务来确定障碍物并找到中心射线

    整体目的是为后续的控制层(HardwareProtectionLayer 或 pattern_node)
    提供障碍物信息，用于避障或导航

    LidarLayer >> HardwareProtectionLayer >> publish twist message
    or
    LidarLayer >> pattern_node >> HardwareProtectionLayer
    >> publish twist message
    """

    def __init__(self):
        """初始化节点"""
        super().__init__('lidar_layer')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_range', 0.0),
                ('min_range', 0.0),
                ('object_reduction', ''),
                ('object_threshold', 0.0),
                ('object_min_width', 0),
                ('object_max_width', 0),
            ])
            
        # 获取参数值
        self.param_max_range = float(
            self.get_parameter("max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "min_range").get_parameter_value().double_value
        self.param_reduction = self.get_parameter(
            "object_reduction").get_parameter_value().string_value
        self.param_object_threshold = self.get_parameter(
            "object_threshold").get_parameter_value().double_value
        self.param_object_min_width = self.get_parameter(
            "object_min_width").get_parameter_value().integer_value
        self.param_object_max_width = self.get_parameter(
            "object_max_width").get_parameter_value().integer_value
        self.scan = None
        
        # 订阅激光雷达数据
        self.scan_subscription = self.create_subscription(
            LaserScan,
            self.get_namespace() + '/scan',
            self.scan_callback,
            qos_profile=qos_profile_sensor_data)
            
        # 发布 RangeData 消息
        self.range_data_publisher = self.create_publisher(
            RangeData,
            self.get_namespace() + '/range_data',
            10)

    def scan_callback(self, scan_msg):
        """
        保存激光雷达扫描数据

        根据当前扫描发布 RangeData 消息.
        """
        #将接收到的 LaserScan 数据保存到 self.scan
        self.scan = scan_msg
        
        # 计算每束激光相对于机器人 x 轴的角度，生成一个角度列表 angles
        angle_min = scan_msg.angle_min              #激光雷达的最小扫描角度
        angle_increment = scan_msg.angle_increment  #每束激光之间的角度增量
        angles = [(angle_min + i*angle_increment) for i in range(len(scan_msg.ranges))] #得到每个激光束的具体角度
        
        # publish range data message
        msg = RangeData()
        msg.header = scan_msg.header
        msg.ranges = scan_msg.ranges #每个扫描束的测量距离的数组
        msg.angles = angles
        self.range_data_publisher.publish(msg)

def main(args=None):
    """
    Create a node for the LiDAR Layer, spin it and handle the
    destruction.
    """
    setup_node.init_and_spin(args, LidarLayer)


if __name__ == '__main__':
    main()
