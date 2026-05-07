"""
RoboSAR - Webots topic/TF adapter node.

Both sides keep their own topic conventions unchanged.
This node sits in the middle and does all mapping:

  ROS2 Webots side:  /robot_N/scan, /robot_N/odom, /robot_N/cmd_vel
  ROS1 RoboSAR side: /robosar_agent_bringup_node/agent_N/feedback/scan,
                     /robosar_agent_bringup_node/agent_N/odom,
                     /robosar_agent_bringup_node/agent_N/cmd_vel

Per robot:
  scan:    /robot_N/scan  ->  /.../agent_N/feedback/scan
  odom:    /robot_N/odom  ->  /.../agent_N/odom  (+ TF broadcast)
  cmd_vel: /.../agent_N/cmd_vel  ->  /robot_N/cmd_vel

Important behavior:
  - Recent Webots / webots_ros2_driver combinations already publish odom in a
    global world frame. In that case applying the historical spawn offset a
    second time corrupts the robot trajectories.
  - Older setups may still need the spawn offset workaround.

This adapter therefore keeps spawn-offset correction configurable. In the
current RoboSAR Webots pipeline the diff-drive odom is local to each robot's
spawn pose, so the offset stays enabled by default.

TF published with Webots sim-time:
  agent_N/odom -> agent_N/base_link            (dynamic, uses odom stamp)
  agent_N/base_link -> agent_N/base_laser_link (fixed, on /tf_static)
  agent_N/base_link -> agent_N/base_footprint  (fixed, on /tf_static)

map -> agent_N/odom is NOT published here; SLAM handles it.
"""
import copy
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


ROBOSAR_NS = 'robosar_agent_bringup_node'


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def quaternion_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def rotate_2d(x, y, theta):
    """Rotate (x, y) by angle theta."""
    c, s = math.cos(theta), math.sin(theta)
    return c * x - s * y, s * x + c * y


class RobotBridge:
    """Bridge instance for a single robot."""

    def __init__(
        self,
        node,
        robot_index,
        spawn_x,
        spawn_y,
        spawn_yaw,
        apply_spawn_offset,
        normalize_scan,
    ):
        self.node = node
        self.idx = robot_index
        self.robot_name = 'robot_%d' % robot_index
        self.agent_name = 'agent_%d' % robot_index
        self.spawn_x = spawn_x
        self.spawn_y = spawn_y
        self.spawn_yaw = spawn_yaw
        self.apply_spawn_offset = apply_spawn_offset
        self.normalize_scan = normalize_scan
        self._last_missing_stamp_log = self.node.get_clock().now()
        self._last_scan_stamp = None

        robosar_prefix = '/%s/%s' % (ROBOSAR_NS, self.agent_name)

        self.scan_pub = node.create_publisher(
            LaserScan,
            '%s/feedback/scan' % robosar_prefix,
            10)
        node.create_subscription(
            LaserScan,
            '/%s/scan' % self.robot_name,
            self._on_scan,
            10)

        self.odom_pub = node.create_publisher(
            Odometry,
            '%s/odom' % robosar_prefix,
            10)
        node.create_subscription(
            Odometry,
            '/%s/odom' % self.robot_name,
            self._on_odom,
            10)

        self.cmd_pub = node.create_publisher(
            Twist,
            '/%s/cmd_vel' % self.robot_name,
            10)
        node.create_subscription(
            Twist,
            '%s/cmd_vel' % robosar_prefix,
            self._on_cmd_vel,
            10)

    def _resolve_stamp(self, stamp, source_name):
        if stamp.sec != 0 or stamp.nanosec != 0:
            return stamp

        now = self.node.get_clock().now()
        if (now - self._last_missing_stamp_log).nanoseconds > 1_000_000_000:
            self.node.get_logger().warn(
                '%s %s missing header stamp, falling back to node clock'
                % (self.agent_name, source_name)
            )
            self._last_missing_stamp_log = now
        return now.to_msg()

    def _apply_spawn_offset(self, local_x, local_y, local_yaw):
        """Transform local odom (0,0,0 at start) to world frame."""
        wx, wy = rotate_2d(local_x, local_y, self.spawn_yaw)
        world_x = wx + self.spawn_x
        world_y = wy + self.spawn_y
        world_yaw = local_yaw + self.spawn_yaw
        return world_x, world_y, world_yaw

    def _on_scan(self, msg):
        stamp = self._resolve_stamp(msg.header.stamp, 'scan')
        scan = copy.deepcopy(msg)
        scan.header.stamp = stamp
        scan.header.frame_id = '%s/base_laser_link' % self.agent_name

        if self.normalize_scan and scan.angle_increment < 0.0:
            scan.ranges = list(reversed(scan.ranges))
            scan.intensities = list(reversed(scan.intensities))
            scan.angle_min, scan.angle_max = scan.angle_max, scan.angle_min
            scan.angle_increment = abs(scan.angle_increment)

        if self._last_scan_stamp is not None:
            dt = (
                stamp.sec - self._last_scan_stamp.sec
                + (stamp.nanosec - self._last_scan_stamp.nanosec) * 1e-9
            )
            if dt > 0.0 and scan.scan_time == 0.0:
                scan.scan_time = dt
            if dt > 0.0 and scan.time_increment == 0.0 and len(scan.ranges) > 1:
                scan.time_increment = dt / float(len(scan.ranges) - 1)
        self._last_scan_stamp = stamp

        self.scan_pub.publish(scan)

    def _on_odom(self, msg):
        stamp = self._resolve_stamp(msg.header.stamp, 'odom')

        local_x = msg.pose.pose.position.x
        local_y = msg.pose.pose.position.y
        local_yaw = quaternion_to_yaw(msg.pose.pose.orientation)

        if self.apply_spawn_offset:
            world_x, world_y, world_yaw = self._apply_spawn_offset(
                local_x, local_y, local_yaw)
        else:
            world_x, world_y, world_yaw = local_x, local_y, local_yaw

        odom = copy.deepcopy(msg)
        odom.header.stamp = stamp
        odom.header.frame_id = '%s/odom' % self.agent_name
        odom.child_frame_id = '%s/base_link' % self.agent_name
        odom.pose.pose.position.x = world_x
        odom.pose.pose.position.y = world_y
        odom.pose.pose.orientation = yaw_to_quaternion(world_yaw)
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = odom.header.frame_id
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = world_x
        t.transform.translation.y = world_y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom.pose.pose.orientation
        self.node.tf_broadcaster.sendTransform(t)

    def _on_cmd_vel(self, msg):
        self.cmd_pub.publish(msg)


class RobosarBridgeAdapter(Node):
    def __init__(self):
        super().__init__('robosar_bridge_adapter')

        self.declare_parameter('num_robots', 3)
        self.declare_parameter('spawn_x', [0.0])
        self.declare_parameter('spawn_y', [0.0])
        self.declare_parameter('spawn_yaw', [0.0])
        self.declare_parameter('apply_spawn_offset', True)
        self.declare_parameter('normalize_scan', True)

        num_robots = self.get_parameter(
            'num_robots').get_parameter_value().integer_value
        spawn_x = list(self.get_parameter(
            'spawn_x').get_parameter_value().double_array_value)
        spawn_y = list(self.get_parameter(
            'spawn_y').get_parameter_value().double_array_value)
        spawn_yaw = list(self.get_parameter(
            'spawn_yaw').get_parameter_value().double_array_value)
        apply_spawn_offset = self.get_parameter(
            'apply_spawn_offset').get_parameter_value().bool_value
        normalize_scan = self.get_parameter(
            'normalize_scan').get_parameter_value().bool_value

        while len(spawn_x) < num_robots:
            spawn_x.append(0.0)
        while len(spawn_y) < num_robots:
            spawn_y.append(0.0)
        while len(spawn_yaw) < num_robots:
            spawn_yaw.append(0.0)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.bridges = []
        self._static_transforms = []

        for i in range(num_robots):
            bridge = RobotBridge(
                self,
                i,
                spawn_x[i],
                spawn_y[i],
                spawn_yaw[i],
                apply_spawn_offset,
                normalize_scan,
            )
            self.bridges.append(bridge)

            agent = 'agent_%d' % i
            self.get_logger().info(
                '  %s: spawn=(%.3f, %.3f, yaw=%.2f)' %
                (agent, spawn_x[i], spawn_y[i], spawn_yaw[i]))

            t1 = TransformStamped()
            t1.header.frame_id = '%s/base_link' % agent
            t1.child_frame_id = '%s/base_laser_link' % agent
            t1.transform.rotation.w = 1.0
            # Keep this aligned with Webots e-puck turretSlot Hokuyo placement.
            t1.transform.translation.x = 0.01
            t1.transform.translation.z = 0.042
            self._static_transforms.append(t1)

            t2 = TransformStamped()
            t2.header.frame_id = '%s/base_link' % agent
            t2.child_frame_id = '%s/base_footprint' % agent
            t2.transform.rotation.w = 1.0
            self._static_transforms.append(t2)

        # ros1_bridge may start after this adapter, so re-send /tf_static
        # periodically to make late bridge subscribers receive the transforms.
        self._publish_static_transforms()
        self.create_timer(1.0, self._publish_static_transforms)

        self.get_logger().info(
            'RoboSAR Bridge Adapter started: %d robots '
            '(robot_N <-> %s/agent_N), use_sim_time=%s, '
            'apply_spawn_offset=%s, normalize_scan=%s'
            % (num_robots, ROBOSAR_NS,
               self.get_parameter('use_sim_time').value,
               apply_spawn_offset,
               normalize_scan))

    def _publish_static_transforms(self):
        stamp = self.get_clock().now().to_msg()
        for transform in self._static_transforms:
            transform.header.stamp = stamp
        self.static_tf_broadcaster.sendTransform(self._static_transforms)


def main(args=None):
    rclpy.init(args=args)
    node = RobosarBridgeAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
