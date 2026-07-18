"""Runtime home-pose capture for response-group return-home behavior."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, MutableMapping, Sequence

from .checkpoint_controller import MatrixPose


@dataclass(frozen=True)
class HomePoseSnapshot:
    robot_names: tuple[str, ...]
    robot_poses: Mapping[str, MatrixPose]
    center: MatrixPose
    slots: Mapping[str, MatrixPose]
    source: str

    def as_param_dict(self) -> dict:
        return {
            "source": self.source,
            "center": {
                "x": self.center.x,
                "y": self.center.y,
                "yaw": self.center.yaw,
                "z": self.center.z,
            },
            "robots": {
                name: {
                    "x": self.robot_poses[name].x,
                    "y": self.robot_poses[name].y,
                    "yaw": self.robot_poses[name].yaw,
                    "z": self.robot_poses[name].z,
                }
                for name in self.robot_names
                if name in self.robot_poses
            },
            "slots": {
                name: {
                    "x": self.slots[name].x,
                    "y": self.slots[name].y,
                    "yaw": self.slots[name].yaw,
                    "z": self.slots[name].z,
                }
                for name in self.robot_names
                if name in self.slots
            },
        }


def record_initial_pose(
    robot_poses: MutableMapping[str, MatrixPose],
    robot_name: str,
    pose: MatrixPose,
) -> bool:
    """Record the first odometry pose for a robot and ignore later motion."""
    name = str(robot_name)
    if name in robot_poses:
        return False
    robot_poses[name] = pose
    return True


def fallback_home_snapshot(
    robot_names: Sequence[str],
    center: MatrixPose,
    source: str = "configured_fallback",
) -> HomePoseSnapshot:
    names = tuple(str(name) for name in robot_names)
    return HomePoseSnapshot(
        robot_names=names,
        robot_poses={name: center for name in names},
        center=center,
        slots={name: MatrixPose(0.0, 0.0, 0.0, 0.0) for name in names},
        source=source,
    )


def capture_home_snapshot(
    robot_names: Sequence[str],
    robot_poses: Mapping[str, MatrixPose],
    fallback_center: MatrixPose | None = None,
    min_robot_fraction: float = 1.0,
) -> HomePoseSnapshot | None:
    """Capture per-robot home slots and the group center from current poses.

    The returned slots are expressed in the captured group-center frame.  The
    center yaw is the circular mean of available robot yaws, which keeps the
    return-home orientation aligned with the actual spawned formation.
    """
    names = tuple(str(name) for name in robot_names)
    available = {name: robot_poses[name] for name in names if name in robot_poses}
    if not names:
        return None
    required = max(1, int(math.ceil(len(names) * max(0.0, min(1.0, min_robot_fraction)))))
    if len(available) < required:
        return None

    center = _center_pose(available.values(), fallback_center)
    slots = {
        name: MatrixPose(
            pose.x - center.x,
            pose.y - center.y,
            _normalize_angle(pose.yaw - center.yaw),
            pose.z - center.z,
        )
        for name, pose in available.items()
    }
    return HomePoseSnapshot(
        robot_names=names,
        robot_poses=available,
        center=center,
        slots=slots,
        source="runtime_capture",
    )


def _center_pose(poses, fallback_center: MatrixPose | None) -> MatrixPose:
    poses = list(poses)
    if not poses:
        return fallback_center or MatrixPose(0.0, 0.0, 0.0, 0.0)
    x = sum(pose.x for pose in poses) / float(len(poses))
    y = sum(pose.y for pose in poses) / float(len(poses))
    z = sum(pose.z for pose in poses) / float(len(poses))
    sin_sum = sum(math.sin(pose.yaw) for pose in poses)
    cos_sum = sum(math.cos(pose.yaw) for pose in poses)
    if abs(sin_sum) <= 1e-9 and abs(cos_sum) <= 1e-9:
        yaw = fallback_center.yaw if fallback_center is not None else 0.0
    else:
        yaw = math.atan2(sin_sum, cos_sum)
    return MatrixPose(x, y, yaw, z)


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle
