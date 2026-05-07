#!/usr/bin/env python3
"""
环境物体生成模块
提供 Webots 世界文件中各种环境物体（机器人、障碍物、任务目标等）的 VRML 生成函数，
以及世界文件解析、碰撞检测、位置生成等工具函数。
"""
import os
import re
import math
import random


def parse_world_file_objects(world_file_path: str) -> list:
    """
    解析 Webots 世界文件(.wbt)，提取已存在物体的位置和边界信息
    
    Args:
        world_file_path: 世界文件路径
    
    Returns:
        list: 物体信息列表，每个元素包含:
              - 对于普通物体: (x, y, radius, name)
              - 对于矩形物体: (x, y, (size_x, size_y), name) 其中 size_x, size_y 是尺寸元组
    """
    objects = []
    
    if not os.path.exists(world_file_path):
        print(f"[parse_world] 警告: 世界文件不存在: {world_file_path}")
        return objects
    
    with open(world_file_path, 'r') as f:
        content = f.read()
    
    excluded_types = [
        'WorldInfo', 'Viewpoint', 'TexturedBackground', 'TexturedBackgroundLight',
        'RectangleArena', 'Floor', 'DirectionalLight', 'PointLight', 'SpotLight',
        'Background', 'Fog', 'Robot', 'E-puck', 'TurtleBot3Burger', 'TurtleBot3Waffle'
    ]
    
    def find_balanced_brace_content(text, start_pos):
        """从 start_pos 位置的 { 开始，找到匹配的 } 并返回内容"""
        if start_pos >= len(text) or text[start_pos] != '{':
            return None, start_pos
        
        depth = 0
        end_pos = start_pos
        for i in range(start_pos, len(text)):
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
                if depth == 0:
                    end_pos = i
                    break
        
        return text[start_pos:end_pos + 1], end_pos + 1
    
    pattern = r'(?:DEF\s+(\w+)\s+)?(\w+)\s*\{'
    
    pos = 0
    while pos < len(content):
        match = re.search(pattern, content[pos:])
        if not match:
            break
        
        match_start = pos + match.start()
        brace_pos = pos + match.end() - 1
        
        def_name = match.group(1)
        node_type = match.group(2)
        
        if node_type in excluded_types:
            pos = brace_pos + 1
            continue
        
        if node_type in ['IMPORTABLE', 'EXTERNPROTO', 'PROTO', 'USE']:
            pos = brace_pos + 1
            continue
        
        node_content, next_pos = find_balanced_brace_content(content, brace_pos)
        
        if node_content is None:
            pos = brace_pos + 1
            continue
        
        trans_match = re.search(r'translation\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)', node_content)
        if not trans_match:
            pos = next_pos
            continue
        
        x = float(trans_match.group(1))
        y = float(trans_match.group(2))
        z = float(trans_match.group(3))
        
        size_info = 0.1
        is_box = False
        size_x = 0.0
        size_y = 0.0
        
        bounding_box_match = re.search(
            r'boundingObject\s+(?:DEF\s+\w+\s+)?Box\s*\{\s*size\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',
            node_content
        )
        if bounding_box_match:
            size_x = float(bounding_box_match.group(1))
            size_y = float(bounding_box_match.group(2))
            is_box = True
        else:
            geom_box_match = re.search(
                r'geometry\s+Box\s*\{\s*size\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)',
                node_content
            )
            if geom_box_match:
                size_x = float(geom_box_match.group(1))
                size_y = float(geom_box_match.group(2))
                is_box = True
            else:
                bounding_cyl_match = re.search(
                    r'boundingObject\s+(?:DEF\s+\w+\s+)?Cylinder\s*\{[^}]*radius\s+([-\d.eE+]+)',
                    node_content
                )
                if bounding_cyl_match:
                    size_info = float(bounding_cyl_match.group(1))
                else:
                    geom_cyl_match = re.search(
                        r'geometry\s+Cylinder\s*\{[^}]*radius\s+([-\d.eE+]+)',
                        node_content
                    )
                    if geom_cyl_match:
                        size_info = float(geom_cyl_match.group(1))
        
        if is_box:
            size_info = (size_x, size_y)
        
        obj_name = def_name
        if not obj_name:
            name_match = re.search(r'name\s+"([^"]+)"', node_content)
            if name_match:
                obj_name = name_match.group(1)
            else:
                obj_name = f"{node_type}_{len(objects)}"
        
        objects.append((x, y, size_info, obj_name))
        pos = next_pos
        
    """
    if objects:
        print(f"[parse_world] 从世界文件解析到 {len(objects)} 个已存在物体:")
        for x, y, size, name in objects:
            if isinstance(size, tuple):
                print(f"  - {name}: 位置({x:.3f}, {y:.3f}), 尺寸=({size[0]:.3f} x {size[1]:.3f})")
            else:
                print(f"  - {name}: 位置({x:.3f}, {y:.3f}), 碰撞半径={size:.3f}")
    """

    return objects


def check_point_box_collision(px: float, py: float, 
                               box_cx: float, box_cy: float, 
                               box_sx: float, box_sy: float,
                               margin: float = 0.0) -> bool:
    """
    检查点是否与轴对齐矩形碰撞（考虑边距）
    
    Args:
        px, py: 点的坐标
        box_cx, box_cy: 矩形中心坐标
        box_sx, box_sy: 矩形尺寸（宽度、高度）
        margin: 额外边距
    
    Returns:
        bool: 如果发生碰撞返回 True
    """
    half_sx = box_sx / 2.0 + margin
    half_sy = box_sy / 2.0 + margin
    
    return (abs(px - box_cx) < half_sx and abs(py - box_cy) < half_sy)


def generate_random_positions_with_obstacles(
    num_items: int,
    x_min: float, x_max: float,
    y_min: float, y_max: float,
    min_distance: float = 0.15,
    seed: int = None,
    existing_positions: list = None
) -> list:
    """
    在指定区域内生成随机位置，确保与已有位置有最小间距
    
    Args:
        num_items: 需要生成的位置数量
        x_min, x_max: X轴范围
        y_min, y_max: Y轴范围
        min_distance: 最小间距
        seed: 随机种子
        existing_positions: 已存在的位置列表，支持两种格式:
                           - 圆形: (x, y, radius)
                           - 矩形: (x, y, (size_x, size_y))
    
    Returns:
        list: 生成的位置列表 [(x, y), ...]
    """
    if seed is not None:
        random.seed(seed)
    
    if existing_positions is None:
        existing_positions = []
    
    positions = []
    max_attempts = 1000
    
    for i in range(num_items):
        for attempt in range(max_attempts):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            
            valid = True
            for px, py in positions:
                dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                if dist < min_distance:
                    valid = False
                    break
            
            if valid:
                for obj in existing_positions:
                    if len(obj) >= 3:
                        ox, oy, size_info = obj[0], obj[1], obj[2]
                    else:
                        ox, oy = obj[0], obj[1]
                        size_info = 0.1
                    
                    if isinstance(size_info, tuple):
                        box_sx, box_sy = size_info
                        if check_point_box_collision(x, y, ox, oy, box_sx, box_sy, min_distance):
                            valid = False
                            break
                    else:
                        radius = size_info
                        dist = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
                        if dist < min_distance + radius:
                            valid = False
                            break
            
            if valid:
                positions.append((x, y))
                break
        else:
            cols = int(math.ceil(math.sqrt(num_items)))
            row = i // cols
            col = i % cols
            x_step = (x_max - x_min) / max(cols, 1)
            y_step = (y_max - y_min) / max(cols, 1)
            x = x_min + col * x_step + x_step / 2
            y = y_min + row * y_step + y_step / 2
            positions.append((x, y))
    
    return positions


def generate_epuck_robot(name: str, x: float, y: float, z: float = 0.0) -> str:
    """
    生成E-puck机器人的VRML定义
    
    Args:
        name: 机器人名称
        x, y, z: 位置坐标
    """
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
}}
'''
"""
    HokuyoUrg04lx {{
      translation 0.01 0.00 0.042
    }}
    HokuyoUrg04lxug01{{
      translation 0.01 0.00 0.042
    }}
    HokuyoUtm30lx {{
      translation 0.01 0.00 0.038
    }}    
    RpLidarA2 {{
      translation 0.01 0.00 0.038
    }}
"""
def generate_burger_robot(name: str, x: float, y: float, z: float = 0.0) -> str:
    """生成TurtleBot3 Burger机器人的VRML定义"""
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
}}
'''


def generate_transport_box(name: str, x: float, y: float, z: float = 0.05, size: float = 0.25) -> str:
    """生成运输任务的箱子（橙色可推动）"""
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.8 0.5 0.2
        roughness 0.5
        metalness 0
      }}
      geometry Box {{
        size {size} {size} {size}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {size} {size} {size}
  }}
  physics Physics {{
    density -1
    mass 0.5
  }}
}}
'''


def generate_target_zone(name: str, x: float, y: float, z: float = 0.001, radius: float = 0.25) -> str:
    """生成运输任务的目标点（绿色圆形区域，可进入，无碰撞）"""
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0 0.8 0.2
        roughness 0.3
        metalness 0
        transparency 0.3
      }}
      geometry Cylinder {{
        height 0.002
        radius {radius}
      }}
    }}
  ]
  name "{name}"
}}
'''


def generate_flocking_target(name: str, x: float, y: float, z: float = 0.15, radius: float = 0.1, height: float = 0.3) -> str:
    """生成环绕任务的圆柱目标（蓝色）"""
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.2 0.4 0.8
        roughness 0.3
        metalness 0.5
      }}
      geometry Cylinder {{
        height {height}
        radius {radius}
      }}
    }}
  ]
  name "{name}"
  boundingObject Cylinder {{
    height {height}
    radius {radius}
  }}
  physics Physics {{
    density -1
    mass 6
  }}
}}
'''


def generate_navigation_target(name: str, x: float, y: float, z: float = 0.001, 
                               radius: float = 0.12, color: tuple = (0.8, 0.2, 0.2)) -> str:
    """
    生成导航任务的目标点（可进入，无碰撞，不同颜色）
    
    Args:
        name: 目标点名称
        x, y, z: 位置坐标
        radius: 目标点半径
        color: 颜色 (R, G, B)，每个机器人的目标点颜色不同
    """
    r, g, b = color
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor {r} {g} {b}
        roughness 0.3
        metalness 0
        transparency 0.2
      }}
      geometry Cylinder {{
        height 0.002
        radius {radius}
      }}
    }}
  ]
  name "{name}"
}}
'''


NAVIGATION_TARGET_COLORS = [
    (0.9, 0.2, 0.2),   # 红色
    (0.2, 0.8, 0.2),   # 绿色
    (0.2, 0.4, 0.9),   # 蓝色
    (0.9, 0.7, 0.1),   # 黄色
    (0.8, 0.3, 0.8),   # 紫色
    (0.1, 0.8, 0.8),   # 青色
    (0.9, 0.5, 0.2),   # 橙色
    (0.6, 0.3, 0.1),   # 棕色
    (0.9, 0.2, 0.6),   # 粉色
    (0.4, 0.6, 0.3),   # 橄榄色
]


def generate_obstacle(name: str, x: float, y: float, z: float = 0.05, size: float = 0.2) -> str:
    """生成随机障碍物（灰色刚体方块）"""
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 0.6 0.6 0.6
        roughness 0.8
        metalness 0.1
      }}
      geometry Box {{
        size {size} {size} {size}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {size} {size} {size}
  }}
  physics Physics {{
    density -1
    mass 6
  }}
}}
'''


def generate_global_info_supervisor() -> str:
    """生成 GlobalInfoSupervisor 机器人（用于发布全局信息）
    
    注意：synchronization FALSE 使得此控制器不会阻塞仿真步进，
    它可以独立于其他控制器运行，只是被动获取信息。
    """
    return '''Robot {
  name "GlobalInfoSupervisor"
  controller "<extern>"
  supervisor TRUE
  synchronization FALSE
}
'''


def generate_world_file(
    template_world: str,
    output_world: str,
    robot_type: str,
    num_robots: int,
    positions: list,
    z_height: float = 0.0,
    task_objects: dict = None
) -> str:
    """基于模板世界文件生成包含机器人和任务物体的新世界文件"""
    with open(template_world, 'r') as f:
        template_content = f.read()
    
    output_content = template_content
    if not output_content.endswith('\n'):
        output_content += '\n'
    
    if task_objects:
        if 'transport_box' in task_objects and task_objects['transport_box']:
            x, y = task_objects['transport_box']
            output_content += generate_transport_box('transport_box', x, y)
            output_content += "\n"
        
        if 'target_zone' in task_objects and task_objects['target_zone']:
            x, y = task_objects['target_zone']
            output_content += generate_target_zone('target_zone', x, y)
            output_content += "\n"
        
        if 'flocking_target' in task_objects and task_objects['flocking_target']:
            x, y = task_objects['flocking_target']
            output_content += generate_flocking_target('flocking_target', x, y)
            output_content += "\n"
        
        if 'navigation_targets' in task_objects and task_objects['navigation_targets']:
            for i, (x, y) in enumerate(task_objects['navigation_targets']):
                color = NAVIGATION_TARGET_COLORS[i % len(NAVIGATION_TARGET_COLORS)]
                output_content += generate_navigation_target(f'navigation_target_{i}', x, y, color=color)
                output_content += "\n"
        
        if 'obstacles' in task_objects and task_objects['obstacles']:
            for i, (x, y) in enumerate(task_objects['obstacles']):
                output_content += generate_obstacle(f'obstacle_{i}', x, y)
                output_content += "\n"
    
    for i in range(num_robots):
        robot_name = f"robot_{i}"
        x, y = positions[i]
        
        if robot_type == "e_puck":
            output_content += generate_epuck_robot(robot_name, x, y, z_height)
        elif robot_type == "burger":
            output_content += generate_burger_robot(robot_name, x, y, z_height)
        else:
            raise ValueError(f"不支持的机器人类型: {robot_type}")
        output_content += "\n"
    
    output_content += generate_global_info_supervisor()
    output_content += "\n"
    
    output_dir = os.path.dirname(output_world)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    with open(output_world, 'w') as f:
        f.write(output_content)
    
    print(f"[generate_world] 已生成世界文件: {output_world}")
    
    return output_world


def get_task_type_from_pattern(pattern: str) -> str:
    """
    根据pattern名称判断任务类型
    
    Returns:
        'transport': 运输任务
        'roundflocking': 环绕任务
        'navigation': 导航任务
        'none': 无特殊任务
    """
    pattern_lower = pattern.lower()
    if 'transport' in pattern_lower:
        return 'transport'
    elif 'roundflocking' in pattern_lower:
        return 'roundflocking'
    elif 'navigation' in pattern_lower:
        return 'navigation'
    else:
        return 'none'
