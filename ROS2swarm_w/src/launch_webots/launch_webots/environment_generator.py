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

PEDESTRIAN_ROOT_HEIGHT = 1.27
PEDESTRIAN_PROTO_URL = (
    'https://raw.githubusercontent.com/cyberbotics/webots/'
    '{version}/projects/humans/pedestrian/protos/Pedestrian.proto'
)
DEFAULT_ROBOT_ROTATION = '0 0 1 3.1415'
DEFAULT_TRANSPORT_OBJECT_SHAPE = 'cube'
TRANSPORT_OBJECT_SHAPES = (
    'cube',
    'cylinder',
    'triangular_prism',
)
TRANSPORT_OBJECT_SIZE = 0.25
TRANSPORT_TARGET_ZONE_RADIUS = 0.25
TRANSPORT_OBJECT_MASS = 0.6
TRANSPORT_OBJECT_Z = 0.05
ROUND_FLOCKING_TARGET_CLEARANCE = 0.6
RUNTIME_ROBOT_RADIUS = 0.0
RUNTIME_DEFAULT_OBJECT_RADIUS = 0.1


class NoCollisionFreePositionError(ValueError):
    """Raised when the requested area cannot fit another collision-free item."""


def inset_bounds_for_clearance(
    min_value: float,
    max_value: float,
    clearance: float,
) -> tuple[float, float]:
    """Return bounds inset by ``clearance`` when the interval is wide enough."""
    min_value = float(min_value)
    max_value = float(max_value)
    clearance = max(0.0, float(clearance))
    if max_value < min_value:
        min_value, max_value = max_value, min_value

    if max_value - min_value < 2.0 * clearance:
        center = (min_value + max_value) * 0.5
        return center, center
    return min_value + clearance, max_value - clearance


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


def is_position_collision_free(
    x: float,
    y: float,
    min_distance: float,
    existing_positions: list | None = None,
) -> bool:
    """Return whether a candidate position clears all occupied positions."""
    for obj in existing_positions or []:
        if len(obj) >= 3:
            ox, oy, size_info = obj[0], obj[1], obj[2]
        else:
            ox, oy = obj[0], obj[1]
            size_info = RUNTIME_DEFAULT_OBJECT_RADIUS

        if isinstance(size_info, tuple):
            box_sx, box_sy = size_info
            if check_point_box_collision(x, y, ox, oy, box_sx, box_sy, min_distance):
                return False
        else:
            distance = math.hypot(x - ox, y - oy)
            if distance < min_distance + float(size_info):
                return False
    return True


def runtime_object_radius(name: str) -> float:
    """Return the conservative collision radius for a `/global_info` object."""
    if name.startswith('victim_'):
        return 0.22
    if name.startswith('fire_source_'):
        return 0.18
    if name.startswith('transport_box'):
        return TRANSPORT_OBJECT_SIZE / 2.0
    if name.startswith('target_zone'):
        return TRANSPORT_TARGET_ZONE_RADIUS
    if name == 'flocking_target':
        return 0.15
    if name.startswith('navigation_target_'):
        return 0.05
    if name.startswith('obstacle_'):
        return 0.1
    return RUNTIME_DEFAULT_OBJECT_RADIUS


def occupied_positions_from_global_info(global_info: dict) -> list[tuple[float, float, float]]:
    """Convert the runtime `/global_info` payload to collision occupancy data."""
    occupied_positions = []

    for entries, radius_for_name in (
        (global_info.get('robots', {}), lambda _name: RUNTIME_ROBOT_RADIUS),
        (global_info.get('objects', {}), runtime_object_radius),
    ):
        for name, entry in entries.items():
            try:
                position = entry['position']
                x = float(position['x'])
                y = float(position['y'])
            except (KeyError, TypeError, ValueError):
                continue
            occupied_positions.append((x, y, radius_for_name(name)))

    return occupied_positions


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
            
            if valid and not is_position_collision_free(
                x,
                y,
                min_distance,
                existing_positions,
            ):
                valid = False
            
            if valid:
                positions.append((x, y))
                break
        else:
            raise NoCollisionFreePositionError(
                f'could not place item {i + 1}/{num_items} without collision '
                f'after {max_attempts} attempts'
            )
    
    return positions


def is_prime_count(value: int) -> bool:
    """Return whether a positive item count is prime."""
    if value < 2:
        return False
    if value == 2:
        return True
    if value % 2 == 0:
        return False
    limit = int(math.sqrt(value))
    for factor in range(3, limit + 1, 2):
        if value % factor == 0:
            return False
    return True


def navigation_grid_columns(num_items: int) -> int:
    """Choose deterministic navigation target columns from robot count.

    Rule:
    - even counts use two columns;
    - prime counts use one column;
    - other composite counts use the smallest factor greater than one.
    """
    if num_items <= 1:
        return 1
    if num_items % 2 == 0:
        return 2
    if is_prime_count(num_items):
        return 1
    for factor in range(3, num_items + 1, 2):
        if num_items % factor == 0:
            return factor
    return 1


def generate_grid_positions(
    num_items: int,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
    columns: int | None = None,
) -> list[tuple[float, float]]:
    """Generate a deterministic column-major grid layout inside the given bounds."""
    if num_items <= 0:
        return []
    if columns is None:
        columns = navigation_grid_columns(num_items)
    if columns <= 0:
        raise ValueError('columns must be positive')
    if x_min > x_max or y_min > y_max:
        raise ValueError('position bounds must be ordered')

    x_span = x_max - x_min
    y_span = y_max - y_min
    columns = min(columns, num_items)
    if columns == 1:
        x_positions = [(x_min + x_max) / 2.0]
    else:
        x_min_inner = x_min + 0.25 * x_span
        x_max_inner = x_max - 0.25 * x_span
        x_step = (x_max_inner - x_min_inner) / (columns - 1)
        x_positions = [
            x_min_inner + column * x_step
            for column in range(columns)
        ]

    rows = int(math.ceil(num_items / columns))
    if rows == 1:
        y_positions = [(y_min + y_max) / 2.0]
    else:
        row_min = y_min + 0.10 * y_span
        row_max = y_max - 0.10 * y_span
        row_step = (row_max - row_min) / (rows - 1)
        y_positions = [row_min + row * row_step for row in range(rows)]

    positions = []
    for column, x in enumerate(x_positions):
        start = column * rows
        column_count = min(rows, max(0, num_items - start))
        positions.extend((x, y_positions[row]) for row in range(column_count))
    return positions


def generate_two_column_positions(
    num_items: int,
    x_min: float,
    x_max: float,
    y_min: float,
    y_max: float,
) -> list[tuple[float, float]]:
    """Generate a deterministic two-column layout inside the given bounds."""
    return generate_grid_positions(num_items, x_min, x_max, y_min, y_max, columns=2)


def generate_epuck_robot(name: str, x: float, y: float, z: float = 0.0) -> str:
    """
    生成E-puck机器人的VRML定义
    
    Args:
        name: 机器人名称
        x, y, z: 位置坐标
    """
    return f'''DEF {name} E-puck {{
  translation {x} {y} {z}
  rotation {DEFAULT_ROBOT_ROTATION}
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
  rotation {DEFAULT_ROBOT_ROTATION}
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


def normalize_transport_object_shape(shape: str | None) -> str:
    """标准化运输物体形状配置。"""
    if shape is None or str(shape).strip() == '':
        return DEFAULT_TRANSPORT_OBJECT_SHAPE

    normalized = str(shape).strip().lower()
    if normalized not in TRANSPORT_OBJECT_SHAPES:
        raise ValueError(
            f'不支持的 transport_object_shape: {shape}; '
            f'可选值: {", ".join(TRANSPORT_OBJECT_SHAPES)}'
        )
    return normalized


def _transport_object_appearance() -> str:
    return '''PBRAppearance {
        baseColor 0.8 0.5 0.2
        roughness 0.5
        metalness 0
      }'''


def _transport_object_physics(mass: float) -> str:
    return f'''Physics {{
    density -1
    mass {mass}
  }}'''


def _generate_cube_transport_box(name: str, x: float, y: float, z: float, size: float, mass: float) -> str:
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance {_transport_object_appearance()}
      geometry Box {{
        size {size} {size} {size}
      }}
    }}
  ]
  name "{name}"
  boundingObject Box {{
    size {size} {size} {size}
  }}
  physics {_transport_object_physics(mass)}
}}
'''


def _generate_cylinder_transport_box(name: str, x: float, y: float, z: float, size: float, mass: float) -> str:
    radius = size / 2.0
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance {_transport_object_appearance()}
      geometry Cylinder {{
        radius {radius}
        height {size}
      }}
    }}
  ]
  name "{name}"
  boundingObject Cylinder {{
    radius {radius}
    height {size}
  }}
  physics {_transport_object_physics(mass)}
}}
'''


def _generate_triangular_prism_transport_box(
    name: str,
    x: float,
    y: float,
    z: float,
    size: float,
    mass: float,
) -> str:
    half_base = size / 2.0
    half_depth = size / 2.0
    half_height = size / 2.0
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance {_transport_object_appearance()}
      geometry DEF {name}_mesh IndexedFaceSet {{
        coord Coordinate {{
          point [
            {-half_base} {-half_depth} {-half_height}
            {half_base} {-half_depth} {-half_height}
            0 {-half_depth} {half_height}
            {-half_base} {half_depth} {-half_height}
            {half_base} {half_depth} {-half_height}
            0 {half_depth} {half_height}
          ]
        }}
        coordIndex [
          0 1 2 -1
          3 5 4 -1
          0 2 5 3 -1
          1 4 5 2 -1
          0 3 4 1 -1
        ]
      }}
    }}
  ]
  name "{name}"
  boundingObject Group {{
    children [
      Shape {{
        geometry USE {name}_mesh
      }}
    ]
  }}
  physics {_transport_object_physics(mass)}
}}
'''


def generate_transport_box(
    name: str,
    x: float,
    y: float,
    z: float = TRANSPORT_OBJECT_Z,
    size: float = TRANSPORT_OBJECT_SIZE,
    mass: float = TRANSPORT_OBJECT_MASS,
    shape: str = DEFAULT_TRANSPORT_OBJECT_SHAPE,
) -> str:
    """生成运输任务的可推动物体。"""
    normalized_shape = normalize_transport_object_shape(shape)

    if normalized_shape == 'cube':
        return _generate_cube_transport_box(name, x, y, z, size, mass)
    if normalized_shape == 'cylinder':
        return _generate_cylinder_transport_box(name, x, y, z, size, mass)
    return _generate_triangular_prism_transport_box(name, x, y, z, size, mass)


def generate_victim(name: str, x: float, y: float, z: float = 0.0) -> str:
    """生成受害者实体（Webots 官方 Pedestrian，静态带碰撞体）"""
    root_z = z + PEDESTRIAN_ROOT_HEIGHT
    return f'''DEF {name} Pedestrian {{
  translation {x} {y} {root_z}
  name "{name}"
  shirtColor 0.8 0.2 0.2
  pantsColor 0.18 0.18 0.24
  shoesColor 0.16 0.1 0.06
  skinColor 0.96 0.79 0.68
  enableBoundingObject TRUE
}}
'''


def generate_fire_source(
    name: str,
    x: float,
    y: float,
    z: float = 0.09,
    radius: float = 0.14,
    height: float = 0.18,
) -> str:
    """生成火源占位实体（静态圆柱）"""
    return f'''DEF {name} Solid {{
  translation {x} {y} {z}
  children [
    Shape {{
      appearance PBRAppearance {{
        baseColor 1 0.3 0.05
        emissiveColor 0.9 0.2 0.0
        roughness 0.25
        metalness 0
      }}
      geometry Cylinder {{
        radius {radius}
        height {height}
      }}
    }}
  ]
  name "{name}"
  boundingObject Cylinder {{
    radius {radius}
    height {height}
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


def _resolve_webots_version(world_content: str) -> str:
    """从世界文件头中解析 Webots 版本号。"""
    match = re.search(r'^#VRML_SIM\s+(\S+)\s+utf8', world_content, re.MULTILINE)
    if match:
        return match.group(1)
    return 'R2025a'


def _ensure_pedestrian_proto_import(world_content: str) -> str:
    """确保世界文件声明了 Pedestrian EXTERNPROTO。"""
    if 'Pedestrian.proto' in world_content:
        return world_content

    version = _resolve_webots_version(world_content)
    import_line = f'EXTERNPROTO "{PEDESTRIAN_PROTO_URL.format(version=version)}"'
    header_match = re.match(r'(#VRML_SIM[^\n]*\n)(.*)$', world_content, re.DOTALL)
    if not header_match:
        return import_line + '\n\n' + world_content

    header = header_match.group(1)
    remainder = header_match.group(2).lstrip('\n')
    return header + '\n' + import_line + '\n' + remainder


def platform_basic_time_step_ms(robot_type: str) -> int | None:
    """Match the existing controller rates only for platform-generated worlds."""
    platform_mode = os.environ.get('MASAR_PLATFORM_MODE', '').strip().lower()
    if platform_mode not in {'1', 'true', 'yes', 'on'}:
        return None
    if robot_type == 'burger':
        return 10
    if robot_type == 'e_puck':
        return 20
    raise ValueError(f'unsupported robot type for platform timing: {robot_type}')


def generate_world_file(
    template_world: str,
    output_world: str,
    robot_type: str,
    num_robots: int,
    positions: list,
    z_height: float = 0.0,
    task_objects: dict = None,
    basic_time_step_ms: int | None = None,
) -> str:
    """基于模板世界文件生成包含机器人和任务物体的新世界文件"""
    with open(template_world, 'r') as f:
        template_content = f.read()
    
    output_content = template_content
    if basic_time_step_ms is not None:
        if isinstance(basic_time_step_ms, bool) or not isinstance(basic_time_step_ms, int) or basic_time_step_ms <= 0:
            raise ValueError('basic_time_step_ms must be a positive integer')
        output_content, replacements = re.subn(
            r'(\bbasicTimeStep\s+)[0-9]+(?:\.[0-9]+)?',
            rf'\g<1>{basic_time_step_ms}',
            output_content,
            count=1,
        )
        if replacements != 1:
            raise ValueError('template world has no basicTimeStep')
    if task_objects and task_objects.get('victims'):
        output_content = _ensure_pedestrian_proto_import(output_content)
    if not output_content.endswith('\n'):
        output_content += '\n'

    transport_object_shape = DEFAULT_TRANSPORT_OBJECT_SHAPE
    if task_objects:
        transport_object_shape = normalize_transport_object_shape(
            task_objects.get('transport_object_shape')
        )
    if task_objects:
        if 'victims' in task_objects and task_objects['victims']:
            for i, (x, y) in enumerate(task_objects['victims']):
                output_content += generate_victim(f'victim_{i}', x, y)
                output_content += "\n"

        if 'fires' in task_objects and task_objects['fires']:
            for i, (x, y) in enumerate(task_objects['fires']):
                output_content += generate_fire_source(f'fire_source_{i}', x, y)
                output_content += "\n"

        if 'transport_boxes' in task_objects and task_objects['transport_boxes']:
            for i, (x, y) in enumerate(task_objects['transport_boxes']):
                output_content += generate_transport_box(
                    f'transport_box_{i}',
                    x,
                    y,
                    shape=transport_object_shape,
                )
                output_content += "\n"

        if 'transport_targets' in task_objects and task_objects['transport_targets']:
            for i, (x, y) in enumerate(task_objects['transport_targets']):
                output_content += generate_target_zone(f'target_zone_{i}', x, y)
                output_content += "\n"

        if 'transport_box' in task_objects and task_objects['transport_box']:
            x, y = task_objects['transport_box']
            output_content += generate_transport_box(
                'transport_box',
                x,
                y,
                shape=transport_object_shape,
            )
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
