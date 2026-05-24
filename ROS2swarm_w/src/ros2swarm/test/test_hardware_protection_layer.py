import unittest
from unittest.mock import Mock

import rclpy
from communication_interfaces.msg import RangeData
from geometry_msgs.msg import Twist

from ros2swarm.hardware_protection_layer import HardwareProtectionLayer


class TestHardwareProtectionLayer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args', '-r', '__ns:=/robot_1'])

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = HardwareProtectionLayer()
        self.node.publisher_cmd_vel = Mock()

    def tearDown(self):
        self.node.destroy_node()

    @staticmethod
    def _range_data():
        msg = RangeData()
        msg.ranges = [0.05, 0.05, 0.05]
        msg.angles = [0.0, 0.1, 0.2]
        return msg

    @staticmethod
    def _avoid_direction():
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = 0.5
        return twist

    def test_range_update_does_not_publish_without_drive_command(self):
        self.node.vector_calc = Mock(return_value=[True, self._avoid_direction()])

        self.node.range_data_callback(self._range_data())

        self.node.publisher_cmd_vel.publish.assert_not_called()

    def test_range_update_can_publish_after_drive_command_is_seen(self):
        self.node.vector_calc = Mock(return_value=[True, self._avoid_direction()])

        self.node.command_callback(Twist())
        self.node.publisher_cmd_vel.publish.reset_mock()

        self.node.range_data_callback(self._range_data())

        self.node.publisher_cmd_vel.publish.assert_called_once()
