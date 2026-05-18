"""ROS2swarm checkpoint controller adapters used by the ROS1 task layer."""

from __future__ import annotations

from dataclasses import dataclass
import math
import pickle
from typing import Iterable, List, Mapping, Sequence

import numpy as np

from .cmd_mux import Twist2D

try:
    from neat.nn import RecurrentNetwork

    NEAT_AVAILABLE = True
except ImportError:  # pragma: no cover - exercised on machines without neat-python.
    RecurrentNetwork = None
    NEAT_AVAILABLE = False


@dataclass(frozen=True)
class MatrixPose:
    x: float
    y: float
    yaw: float = 0.0
    z: float = 0.0


@dataclass(frozen=True)
class RobotInfo:
    pose: MatrixPose
    velocity_x: float
    velocity_y: float
    orientation: Sequence[float]


@dataclass(frozen=True)
class SceneObjectInfo:
    x: float
    y: float
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    z: float = 0.0


def yaw_from_orientation_matrix(orientation: Sequence[float]) -> float:
    """Extract yaw from the ROS2swarm row-major 3x3 orientation matrix."""
    if len(orientation) < 4:
        return 0.0
    return math.atan2(float(orientation[3]), float(orientation[0]))


def rotate_global_to_body(x: float, y: float, yaw: float) -> tuple[float, float]:
    cos_yaw = math.cos(-yaw)
    sin_yaw = math.sin(-yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


def sample_lidar_midpoints(
    ranges: Sequence[float],
    n_lidar_rays: int,
    max_range: float,
) -> List[float]:
    """Mirror ROS2swarm's sector-midpoint LaserScan sampling."""
    if n_lidar_rays <= 0:
        raise ValueError("n_lidar_rays must be positive")
    if not ranges:
        return [float(max_range)] * n_lidar_rays

    values = np.array(ranges, dtype=np.float32)
    values = np.where(np.isinf(values), max_range, values)
    values = np.where(np.isnan(values), max_range, values)
    values = np.clip(values, 0.0, max_range)

    beams_per_sector = max(1, len(values) // n_lidar_rays)
    sampled = []
    for index in range(n_lidar_rays):
        start_idx = index * beams_per_sector
        mid_idx = min(start_idx + beams_per_sector // 2, len(values) - 1)
        sampled.append(float(values[mid_idx]))
    return sampled


def build_roundflocking_observation(
    robot: RobotInfo,
    target: SceneObjectInfo,
    lidar_ranges: Sequence[float],
    n_lidar_rays: int,
    max_range: float,
    world_scale: float,
) -> List[float]:
    """Build the 6+N observation vector used by ROS2swarm roundflocking."""
    yaw = yaw_from_orientation_matrix(robot.orientation)
    target_rel_global_x = robot.pose.x - target.x
    target_rel_global_y = robot.pose.y - target.y
    target_rel_x, target_rel_y = rotate_global_to_body(
        target_rel_global_x,
        target_rel_global_y,
        yaw,
    )
    robot_vx_body, robot_vy_body = rotate_global_to_body(
        robot.velocity_x,
        robot.velocity_y,
        yaw,
    )
    scale = max(float(world_scale), 1e-6)
    observation = [
        robot.pose.x / scale,
        robot.pose.y / scale,
        robot_vx_body,
        robot_vy_body,
        target_rel_x / scale,
        target_rel_y / scale,
    ]
    observation.extend(sample_lidar_midpoints(lidar_ranges, n_lidar_rays, max_range))
    return observation


def build_transport_observation(
    robot: RobotInfo,
    box: SceneObjectInfo,
    target: SceneObjectInfo,
    world_scale: float,
    goal_threshold: float,
) -> List[float]:
    """Build the 11-dimensional ROS2swarm transport observation vector."""
    yaw = yaw_from_orientation_matrix(robot.orientation)
    box_to_target_x_global = box.x - target.x
    box_to_target_y_global = box.y - target.y
    robot_to_box_x_global = box.x - robot.pose.x
    robot_to_box_y_global = box.y - robot.pose.y

    box_to_target_x, box_to_target_y = rotate_global_to_body(
        box_to_target_x_global,
        box_to_target_y_global,
        yaw,
    )
    robot_to_box_x, robot_to_box_y = rotate_global_to_body(
        robot_to_box_x_global,
        robot_to_box_y_global,
        yaw,
    )
    robot_vx_body, robot_vy_body = rotate_global_to_body(
        robot.velocity_x,
        robot.velocity_y,
        yaw,
    )
    box_vx_body, box_vy_body = rotate_global_to_body(
        box.velocity_x,
        box.velocity_y,
        yaw,
    )

    scale = max(float(world_scale), 1e-6)
    distance_to_target = math.hypot(box_to_target_x_global, box_to_target_y_global)
    on_goal = 1.0 if distance_to_target < goal_threshold else 0.0
    return [
        robot.pose.x / scale,
        robot.pose.y / scale,
        robot_vx_body,
        robot_vy_body,
        box_to_target_x / scale,
        box_to_target_y / scale,
        robot_to_box_x / scale,
        robot_to_box_y / scale,
        box_vx_body,
        box_vy_body,
        on_goal,
    ]


def differential_twist_from_omni_output(
    output: Sequence[float],
    max_linear_velocity: float,
    max_angular_velocity: float,
) -> Twist2D:
    """Convert ROS2swarm roundflocking [vx, vy] output to differential drive."""
    vx = float(output[0]) * 2.0 - 1.0
    vy = float(output[1]) * 2.0 - 1.0
    speed = math.hypot(vx, vy)
    angle = math.atan2(vy, vx)

    if abs(angle) < math.pi / 2.0:
        linear = max_linear_velocity * math.cos(angle) * min(speed, 1.0)
    else:
        linear = 0.0
    angular = max_angular_velocity * max(-1.0, min(1.0, angle / (math.pi / 2.0)))
    return Twist2D(float(linear), float(angular))


def direct_twist_from_network_output(
    output: Sequence[float],
    max_linear_velocity: float,
    max_angular_velocity: float,
) -> Twist2D:
    """Map ROS2swarm transport sigmoid outputs directly into Twist limits."""
    linear = (float(output[0]) * 2.0 - 1.0) * max_linear_velocity
    angular = (float(output[1]) * 2.0 - 1.0) * max_angular_velocity
    return Twist2D(float(linear), float(angular))


def robot_info_from_global_info(
    global_info: Mapping,
    robot_name: str,
) -> RobotInfo | None:
    raw = global_info.get("robots", {}).get(robot_name)
    if raw is None:
        return None
    pos = raw.get("position", {})
    vel = raw.get("velocity", {})
    orientation = raw.get("orientation", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
    return RobotInfo(
        pose=MatrixPose(
            float(pos.get("x", 0.0)),
            float(pos.get("y", 0.0)),
            yaw_from_orientation_matrix(orientation),
            float(pos.get("z", 0.0)),
        ),
        velocity_x=float(vel.get("x", 0.0)),
        velocity_y=float(vel.get("y", 0.0)),
        orientation=orientation,
    )


def object_info_from_global_info(
    global_info: Mapping,
    object_name: str,
) -> SceneObjectInfo | None:
    raw = global_info.get("objects", {}).get(object_name)
    if raw is None:
        return None
    pos = raw.get("position", {})
    vel = raw.get("velocity", {})
    return SceneObjectInfo(
        x=float(pos.get("x", 0.0)),
        y=float(pos.get("y", 0.0)),
        z=float(pos.get("z", 0.0)),
        velocity_x=float(vel.get("x", 0.0)),
        velocity_y=float(vel.get("y", 0.0)),
    )


class NeatNetworkFactory:
    """Load a neat-python genome package and create independent recurrent nets."""

    def __init__(self, package_path: str):
        if not NEAT_AVAILABLE:
            raise RuntimeError("neat-python is not installed")
        self.package_path = package_path
        with open(package_path, "rb") as stream:
            package = pickle.load(stream)
        self.package = package
        self.genome = package["genome"]
        self.neat_config = package["neat_config"]

    def create(self):
        network = RecurrentNetwork.create(self.genome, self.neat_config)
        network.reset()
        return network

    @property
    def input_dim(self) -> int:
        return len(self.neat_config.genome_config.input_keys)


def clamp_observation(observation: Iterable[float], expected_dim: int) -> List[float]:
    values = [float(value) for value in observation]
    if len(values) < expected_dim:
        values.extend([0.0] * (expected_dim - len(values)))
    if len(values) > expected_dim:
        values = values[:expected_dim]
    return values
