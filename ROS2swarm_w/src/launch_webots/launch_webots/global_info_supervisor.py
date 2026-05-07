#!/usr/bin/env python3
"""
GlobalInfoSupervisor - Webots Supervisor 节点，发布全局信息到 ROS2 话题

该节点作为 Webots 控制器运行（synchronization FALSE），使用 Supervisor API 
获取场景中所有机器人和物体的位置信息，并发布到 ROS2 话题供其他节点使用。

重要：此控制器设置 synchronization FALSE，不会阻塞其他控制器的仿真步进，
但仍需调用 step() 来与 Webots 通信和获取数据。

发布话题：
- /global_info: 所有机器人和物体的位置信息（JSON格式）
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import json
import threading

# Webots Supervisor API
from controller import Supervisor


class GlobalInfoSupervisor(Node):
    """
    Webots Supervisor 节点，发布全局场景信息
    
    使用 synchronization FALSE 模式，控制器的 step() 不会阻塞其他控制器，
    但仍需调用 step() 来与 Webots 通信。
    """

    def __init__(self):
        super().__init__('global_info_supervisor')
        
        # 初始化 Webots Supervisor
        self.supervisor = Supervisor()
        self.timestep = int(self.supervisor.getBasicTimeStep())
        
        # QoS 配置
        qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # 发布全局信息（JSON格式）
        self.global_info_pub = self.create_publisher(
            String, 
            '/global_info', 
            qos
        )
        
        # 缓存节点引用
        self.robot_nodes = {}
        self.object_nodes = {}
        self.prev_positions = {}
        self.prev_time = 0.0
        self.nodes_found = False
        self.running = True
        
        self.get_logger().info(
            f'GlobalInfoSupervisor 启动中 (timestep={self.timestep}ms, synchronization=FALSE)...'
        )
        
        # 使用单独的线程运行 Webots step 循环
        # 这样 ROS2 spin 和 Webots step 可以并行运行
        self.webots_thread = threading.Thread(target=self._webots_loop, daemon=True)
        self.webots_thread.start()

    def _find_all_nodes(self):
        """查找场景中的所有机器人和物体节点"""
        self.robot_nodes.clear()
        self.object_nodes.clear()
        
        try:
            root = self.supervisor.getRoot()
            if root is None:
                self.get_logger().warn('无法获取根节点')
                return False
                
            children_field = root.getField('children')
            if children_field is None:
                self.get_logger().warn('无法获取 children 字段')
                return False
            
            count = children_field.getCount()
            self.get_logger().info(f'场景中有 {count} 个子节点')
            
            for i in range(count):
                node = children_field.getMFNode(i)
                if node is None:
                    continue
                    
                # 获取 DEF 名称
                def_name = node.getDef()
                if not def_name:
                    continue
                
                # 根据 DEF 名称分类
                if def_name.startswith('robot_'):
                    self.robot_nodes[def_name] = node
                    self.get_logger().info(f'找到机器人: {def_name}')
                elif def_name in ['transport_box', 'target_zone', 'flocking_target']:
                    self.object_nodes[def_name] = node
                    self.get_logger().info(f'找到物体: {def_name}')
                elif def_name.startswith('navigation_target_'):
                    self.object_nodes[def_name] = node
                    self.get_logger().info(f'找到导航目标: {def_name}')
                elif def_name.startswith('obstacle_'):
                    self.object_nodes[def_name] = node
                    self.get_logger().info(f'找到障碍物: {def_name}')
            
            if len(self.robot_nodes) > 0:
                self.get_logger().info(
                    f'GlobalInfoSupervisor 初始化完成 - '
                    f'找到 {len(self.robot_nodes)} 个机器人, '
                    f'{len(self.object_nodes)} 个物体'
                )
                return True
            else:
                self.get_logger().warn('未找到任何机器人节点')
            return False
            
        except Exception as e:
            self.get_logger().error(f'查找节点时出错: {e}')
            return False

    def _webots_loop(self):
        """Webots 步进循环（在单独线程中运行）"""
        # 等待几个仿真步让其他控制器初始化
        for _ in range(10):
            if self.supervisor.step(self.timestep) < 0:
                self.running = False
                return
        
        # 查找节点
        self.nodes_found = self._find_all_nodes()
        
        # 主循环
        while self.running:
            # 执行仿真步
            if self.supervisor.step(self.timestep) < 0:
                self.get_logger().info('Webots 仿真结束')
                self.running = False
                break
            
            # 发布全局信息
            self._publish_global_info()

    def _publish_global_info(self):
        """发布全局信息"""
        try:
            current_time = self.supervisor.getTime()
        except Exception as e:
            self.get_logger().debug(f'获取时间失败: {e}')
            return
        
        if current_time <= 0:
            return
        
        # 如果还没找到节点，尝试再次查找
        if not self.nodes_found or len(self.robot_nodes) == 0:
            self.nodes_found = self._find_all_nodes()
            if not self.nodes_found:
                return
        
        # 计算时间间隔
        dt = current_time - self.prev_time if self.prev_time > 0 else 0.032
        if dt <= 0:
            dt = 0.032
        self.prev_time = current_time
        
        # 收集机器人信息
        robots_info = {}
        for def_name, node in self.robot_nodes.items():
            try:
                pos = node.getPosition()
                if pos is None:
                    continue
                    
                rot = node.getOrientation()
                
                # 计算速度
                if def_name in self.prev_positions:
                    prev_pos = self.prev_positions[def_name]
                    vx = (pos[0] - prev_pos[0]) / dt
                    vy = (pos[1] - prev_pos[1]) / dt
                else:
                    vx, vy = 0.0, 0.0
                
                self.prev_positions[def_name] = (pos[0], pos[1], pos[2])
                
                robots_info[def_name] = {
                    'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                    'velocity': {'x': vx, 'y': vy},
                    'orientation': list(rot[0:9]) if rot else [1,0,0,0,1,0,0,0,1]
                }
            except Exception as e:
                self.get_logger().debug(f'获取机器人 {def_name} 信息失败: {e}')
        
        # 收集物体信息
        objects_info = {}
        for def_name, node in self.object_nodes.items():
            try:
                pos = node.getPosition()
                if pos is None:
                    continue
                
                if def_name in self.prev_positions:
                    prev_pos = self.prev_positions[def_name]
                    vx = (pos[0] - prev_pos[0]) / dt
                    vy = (pos[1] - prev_pos[1]) / dt
                else:
                    vx, vy = 0.0, 0.0
                
                self.prev_positions[def_name] = (pos[0], pos[1], pos[2])
                
                objects_info[def_name] = {
                    'position': {'x': pos[0], 'y': pos[1], 'z': pos[2]},
                    'velocity': {'x': vx, 'y': vy}
                }
            except Exception as e:
                self.get_logger().debug(f'获取物体 {def_name} 信息失败: {e}')
        
        # 发布数据
        if len(robots_info) > 0 or len(objects_info) > 0:
            global_info = {
                'timestamp': current_time,
                'robots': robots_info,
                'objects': objects_info
            }
            
            msg = String()
            msg.data = json.dumps(global_info)
            self.global_info_pub.publish(msg)
            
            # 每5秒打印一次调试信息
            """
            if int(current_time * 10) % 50 == 0:
                self.get_logger().info(
                    f'发布全局信息 - 时间: {current_time:.2f}s, '
                    f'机器人: {len(robots_info)}, 物体: {len(objects_info)}'
                )
            """

    def destroy_node(self):
        """销毁节点"""
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GlobalInfoSupervisor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
