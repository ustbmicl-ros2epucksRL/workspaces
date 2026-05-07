"""
通过 Ros2Supervisor 的 spawn_node_from_string 服务，向已运行的 Webots 仿真中动态添加机器人
支持 E-puck（带 RpLidarA2）和 TurtleBot3 Burger 两种类型
"""
import rclpy
import argparse
from webots_ros2_msgs.srv import SpawnNodeFromString


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


def main():
    parser = argparse.ArgumentParser(description='Spawn Robot into Webots')
    parser.add_argument('-n', '--robot_name', type=str, default='robot',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='robot',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')
    parser.add_argument('-t', '--type_of_robot', type=str, default='e_puck',
                        help='the type of robot (e_puck | burger)')

    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = rclpy.create_node("entity_spawner_" + args.robot_name)

    service_name = '/Ros2Supervisor/spawn_node_from_string'
    client = node.create_client(SpawnNodeFromString, service_name)

    node.get_logger().info(f"等待 {service_name} 服务...")
    if not client.service_is_ready():
        client.wait_for_service()
    node.get_logger().info("服务已就绪")

    if args.type_of_robot.startswith("e_puck"):
        proto_string = _epuck_proto(args.robot_name, args.x, args.y, args.z)
    elif args.type_of_robot.startswith("burger"):
        proto_string = _burger_proto(args.robot_name, args.x, args.y, args.z)
    else:
        raise ValueError(f"不支持的机器人类型: {args.type_of_robot}")

    node.get_logger().info(
        f"正在生成 {args.type_of_robot} 机器人 '{args.robot_name}' "
        f"于 ({args.x}, {args.y}, {args.z})")

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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
