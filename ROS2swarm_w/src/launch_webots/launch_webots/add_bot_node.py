"""
通过 Ros2Supervisor 的 spawn_node_from_string 服务，向已运行的 Webots 仿真中动态添加机器人
支持 E-puck（带 RpLidarA2）和 TurtleBot3 Burger 两种类型
"""
import argparse
import json
import time

from launch_webots.environment_generator import (
    generate_random_positions_with_obstacles,
    is_position_collision_free,
    occupied_positions_from_global_info,
)

DEFAULT_SPAWN_MIN = -1.0
DEFAULT_SPAWN_MAX = 1.0


def _epuck_proto(name: str, x: float, y: float, z: float) -> str:
    return f'''DEF {name} E-puck {{
  translation {x} {y} {z}
  rotation 0 0 1 3.1415
  controller "<extern>"
  supervisor TRUE
  name "{name}"
  version "2"
  camera_width 640
  camera_height 480
  turretSlot [
    Pi-puck {{
    }}
    RpLidarA2 {{
      translation 0.01 0.00 0.038
    }}
  ]
}}'''


def _burger_proto(name: str, x: float, y: float, z: float) -> str:
    return f'''DEF {name} TurtleBot3Burger {{
  translation {x} {y} {z}
  controller "<extern>"
  supervisor TRUE
  name "{name}"
  extensionSlot [
    Solid {{
      name "imu_link"
    }}
    GPS {{
    }}
    InertialUnit {{
      name "inertial_unit"
    }}
    RobotisLds01 {{
    }}
  ]
}}'''


def _resolve_spawn_pose(
    args: argparse.Namespace,
    occupied_positions: list | None = None,
) -> tuple[float, float]:
    """Generate or validate a spawn pose against the current scene occupancy."""
    if args.x_min >= args.x_max:
        raise ValueError('x_min must be smaller than x_max')
    if args.y_min >= args.y_max:
        raise ValueError('y_min must be smaller than y_max')

    has_x = args.x is not None
    has_y = args.y is not None
    if has_x != has_y:
        raise ValueError('x and y must be provided together')

    if has_x:
        x, y = args.x, args.y
        if not is_position_collision_free(
            x,
            y,
            args.min_distance,
            occupied_positions,
        ):
            raise ValueError('requested spawn position overlaps an existing object')
    else:
        x, y = generate_random_positions_with_obstacles(
            1,
            args.x_min,
            args.x_max,
            args.y_min,
            args.y_max,
            args.min_distance,
            args.random_seed,
            occupied_positions,
        )[0]

    return x, y


def _wait_for_global_info(node, rclpy, string_type, timeout: float) -> dict:
    """Wait for a current scene snapshot published by GlobalInfoSupervisor."""
    global_info = None

    def callback(message):
        nonlocal global_info
        try:
            data = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            return
        if isinstance(data, dict):
            global_info = data

    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

    qos = QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
    )
    subscription = node.create_subscription(string_type, '/global_info', callback, qos)
    deadline = time.monotonic() + timeout
    while global_info is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
    node.destroy_subscription(subscription)

    if global_info is None:
        raise RuntimeError(
            'timed out waiting for /global_info; start the simulation with '
            'create_enviroment.launch.py before adding random robots'
        )
    return global_info

def main():
    import rclpy
    from std_msgs.msg import String
    from webots_ros2_msgs.srv import SpawnNodeFromString

    parser = argparse.ArgumentParser(description='Spawn Robot into Webots')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('--x-min', type=float, default=DEFAULT_SPAWN_MIN,
                        help='minimum random x position [meters]')
    parser.add_argument('--x-max', type=float, default=DEFAULT_SPAWN_MAX,
                        help='maximum random x position [meters]')
    parser.add_argument('--y-min', type=float, default=DEFAULT_SPAWN_MIN,
                        help='minimum random y position [meters]')
    parser.add_argument('--y-max', type=float, default=DEFAULT_SPAWN_MAX,
                        help='maximum random y position [meters]')
    parser.add_argument('--random-seed', type=int,
                        help='optional seed for a reproducible random spawn pose')
    parser.add_argument('--min-distance', type=float, default=0.2,
                        help='minimum clearance from scene objects [meters]')
    parser.add_argument('--global-info-timeout', type=float, default=5.0,
                        help='seconds to wait for a /global_info scene snapshot')
    parser.add_argument('-t', '--type_of_robot', type=str, default='e_puck',
                        help='the type of robot (e_puck | burger)')

    args, unknown = parser.parse_known_args()
    if args.min_distance < 0:
        raise ValueError('min_distance must be non-negative')
    if args.global_info_timeout <= 0:
        raise ValueError('global_info_timeout must be positive')

    rclpy.init()
    node = rclpy.create_node("entity_spawner_" + args.robot_name)
    try:
        global_info = _wait_for_global_info(
            node,
            rclpy,
            String,
            args.global_info_timeout,
        )
        occupied_positions = occupied_positions_from_global_info(global_info)
        x, y = _resolve_spawn_pose(args, occupied_positions)

        service_name = '/Ros2Supervisor/spawn_node_from_string'
        client = node.create_client(SpawnNodeFromString, service_name)

        node.get_logger().info(f"等待 {service_name} 服务...")
        if not client.service_is_ready():
            client.wait_for_service()
        node.get_logger().info("服务已就绪")

        if args.type_of_robot.startswith("e_puck"):
            proto_string = _epuck_proto(args.robot_name, x, y, args.z)
        elif args.type_of_robot.startswith("burger"):
            proto_string = _burger_proto(args.robot_name, x, y, args.z)
        else:
            raise ValueError(f"不支持的机器人类型: {args.type_of_robot}")

        node.get_logger().info(
            f"正在生成 {args.type_of_robot} 机器人 '{args.robot_name}' "
            f"于 ({x:.3f}, {y:.3f}, {args.z:.3f})")

        request = SpawnNodeFromString.Request()
        request.data = proto_string

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        if future.result() is not None:
            if future.result().success:
                node.get_logger().info(f"机器人 '{args.robot_name}' 生成成功")
            else:
                node.get_logger().error(f"生成失败: {future.result()}")
        else:
            raise RuntimeError(f"服务调用异常: {future.exception()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
