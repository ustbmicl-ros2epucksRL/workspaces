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
import functools
import rclpy

from rclpy.node import Node
from ros2swarm.utils import setup_node
from sensor_msgs.msg import Range
from communication_interfaces.msg import RangeData
from rclpy.qos import qos_profile_sensor_data

class IRLayer(Node):
    """
    订阅多个 Range 类型的话题（由 YAML 配置文件指定）
    ros2swarm/config/robot/sensor_specification.yaml.
    
    整合这些传感器数据并发布 RangeData 消息到 /range_data 话题

    整体目的是为后续的控制层(HardwareProtectionLayer 或 pattern_node)
    提供障碍物信息，用于避障或导航

    IRLayer >> HardwareProtectionLayer >> publish twist message
    or
    IRLayer >> pattern_node >> HardwareProtectionLayer
    >> publish twist message
    """

    def __init__(self):
        """初始化节点"""
        super().__init__('ir_layer') 
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('angles', [0.0]),
                ('range_topics', ['']),  #需要订阅的距离传感器的话题名列表
                ('max_range', 0.0),
                ('min_range', 0.0),
            ])
            
        # get the parameter values from sensor_specifications file
        self.angles = self.get_parameter(
            "angles").get_parameter_value().double_array_value
        self.range_topics = self.get_parameter(
            "range_topics").get_parameter_value().string_array_value 
        self.param_max_range = float(
            self.get_parameter("max_range").get_parameter_value().double_value)
        self.param_min_range = self.get_parameter(
            "min_range").get_parameter_value().double_value
            
        # other class variables 用于存储每个传感器的当前测量值
        self.current_ranges = [None] * len(self.range_topics)
        
        # publishers and subscribers of sensor topics
        self.range_subscriptions = [self.create_subscription(
                                       Range,
                                       self.get_namespace() + "/" + self.range_topics[i],
                                       functools.partial(self.range_callback, index=i), #将每个回调函数与不同的传感器索引绑定，便于在回调函数中识别数据来自哪个传感器
                                       qos_profile=qos_profile_sensor_data)
                                       for i in range(len(self.range_topics))] #为每个传感器定义一个订阅器

        print(self.get_namespace() + "/" + self.range_topics[0],"==========================")
        
        # 将整合后的传感器数据发布到 /range_data 话题
        self.range_data_publisher = self.create_publisher(
            RangeData,
            self.get_namespace() + '/range_data',
            100)

    def range_callback(self, range_msg, index):
        """
        Save the range in the array of current ranges and get the angle
        between the corresponding ray and the x-axis of the robot from
        the specifications file.

        Publish a RangeData message on basis of the current ranges, if
        at least one range measurement has already been received from
        each sensor.
        """
        
        # 保存收到的传感器数据
        self.current_ranges[index] = range_msg.range
        
        # 发布 RangeData 消息
        if not None in self.current_ranges:
            msg = RangeData()
            msg.header = range_msg.header
            msg.header.frame_id = ""
            msg.ranges = self.current_ranges
            msg.angles = self.angles
            self.range_data_publisher.publish(msg)
            self.current_ranges = [None] * len(self.range_topics) #重置 self.current_ranges 为 [None]，以便接收新的一轮传感器数据

def main(args=None):
    """
    Create a node for the IR Layer, spin it and handle the
    destruction.
    """
    setup_node.init_and_spin(args, IRLayer)


if __name__ == '__main__':
    main()
