"""Emergency task queue and staging-pose logic for the response group."""

from __future__ import annotations

from dataclasses import dataclass, replace
import math
from typing import Dict, Iterable, List, Optional, Tuple

from .checkpoint_controller import MatrixPose


TASK_VICTIM = 1
TASK_FIRE = 2
TASK_BOX = 3

STATUS_NEW = 1
STATUS_DISPATCHED = 2
STATUS_EN_ROUTE = 3
STATUS_ACTING = 4
STATUS_DONE = 5
STATUS_FAILED = 6

BEHAVIOR_ROUNDFLOCKING = "TASK_ROUNDFLOCKING"
BEHAVIOR_TRANSPORT = "TASK_TRANSPORT"
BEHAVIOR_SPIN = "TASK_SPIN_6S_PLACEHOLDER"


@dataclass(frozen=True)
class StagingDistances:
    victim: float = 0.9
    fire: float = 1.1
    box: float = 0.8

    def for_type(self, task_type: int) -> float:
        if task_type == TASK_FIRE:
            return self.fire
        if task_type == TASK_BOX:
            return self.box
        return self.victim


@dataclass(frozen=True)
class EmergencyObjectSpec:
    object_type: int
    object_id: int
    scene_name: str
    detector_robot: str
    pose: MatrixPose
    discovered: bool = True
    goal_pose: Optional[MatrixPose] = None
    has_goal_pose: bool = False


@dataclass(frozen=True)
class ManagedTask:
    task_id: int
    task_type: int
    status: int
    object_id: int
    scene_name: str
    detector_robot: str
    precise_pose: MatrixPose
    staging_pose: MatrixPose
    goal_pose: Optional[MatrixPose]
    has_goal_pose: bool
    hold_seconds: float
    behavior_type: str
    assigned_robots: Tuple[str, ...]
    discovered_order: int

    @property
    def key(self) -> tuple[int, int, str]:
        return self.task_type, self.object_id, self.scene_name


def behavior_for_task_type(task_type: int, use_spin_placeholder: bool = False) -> str:
    if use_spin_placeholder:
        return BEHAVIOR_SPIN
    if task_type == TASK_BOX:
        return BEHAVIOR_TRANSPORT
    return BEHAVIOR_ROUNDFLOCKING


def _safe_direction(dx: float, dy: float, fallback_yaw: float = 0.0) -> tuple[float, float, float]:
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return math.cos(fallback_yaw), math.sin(fallback_yaw), fallback_yaw
    return dx / norm, dy / norm, math.atan2(dy, dx)


def compute_staging_pose(
    task_type: int,
    object_pose: MatrixPose,
    group_center: MatrixPose,
    goal_pose: MatrixPose | None,
    distances: StagingDistances,
) -> MatrixPose:
    distance = distances.for_type(task_type)
    if task_type == TASK_BOX and goal_pose is not None:
        direction_x, direction_y, yaw = _safe_direction(
            goal_pose.x - object_pose.x,
            goal_pose.y - object_pose.y,
            object_pose.yaw,
        )
    else:
        direction_x, direction_y, yaw = _safe_direction(
            object_pose.x - group_center.x,
            object_pose.y - group_center.y,
            object_pose.yaw,
        )

    return MatrixPose(
        x=object_pose.x - direction_x * distance,
        y=object_pose.y - direction_y * distance,
        yaw=yaw,
    )


def priority_for_task_type(task_type: int) -> int:
    if task_type == TASK_VICTIM:
        return 0
    if task_type == TASK_FIRE:
        return 1
    if task_type == TASK_BOX:
        return 2
    return 99


class ResponseTaskQueue:
    """Small deterministic queue for emergency response tasks."""

    STATUS_NEW = STATUS_NEW
    STATUS_DISPATCHED = STATUS_DISPATCHED
    STATUS_EN_ROUTE = STATUS_EN_ROUTE
    STATUS_ACTING = STATUS_ACTING
    STATUS_DONE = STATUS_DONE
    STATUS_FAILED = STATUS_FAILED

    def __init__(
        self,
        assigned_robots: Iterable[str],
        staging_distances: StagingDistances,
        hold_seconds: float = 6.0,
        use_spin_placeholder: bool = False,
    ):
        self.assigned_robots = tuple(assigned_robots)
        self.staging_distances = staging_distances
        self.hold_seconds = float(hold_seconds)
        self.use_spin_placeholder = bool(use_spin_placeholder)
        self._tasks: Dict[tuple[int, int, str], ManagedTask] = {}
        self._active_task_id: int | None = None
        self._next_task_id = 1
        self._order = 0

    @property
    def active_task_id(self) -> int | None:
        return self._active_task_id

    def upsert_object(
        self,
        obj: EmergencyObjectSpec,
        group_center: MatrixPose,
    ) -> bool:
        if not obj.discovered:
            return False
        key = (obj.object_type, obj.object_id, obj.scene_name)
        if key in self._tasks:
            return False

        staging = compute_staging_pose(
            task_type=obj.object_type,
            object_pose=obj.pose,
            group_center=group_center,
            goal_pose=obj.goal_pose if obj.has_goal_pose else None,
            distances=self.staging_distances,
        )
        task = ManagedTask(
            task_id=self._next_task_id,
            task_type=obj.object_type,
            status=STATUS_NEW,
            object_id=obj.object_id,
            scene_name=obj.scene_name,
            detector_robot=obj.detector_robot,
            precise_pose=obj.pose,
            staging_pose=staging,
            goal_pose=obj.goal_pose,
            has_goal_pose=bool(obj.has_goal_pose and obj.goal_pose is not None),
            hold_seconds=self.hold_seconds,
            behavior_type=behavior_for_task_type(
                obj.object_type,
                use_spin_placeholder=self.use_spin_placeholder,
            ),
            assigned_robots=self.assigned_robots,
            discovered_order=self._order,
        )
        self._tasks[key] = task
        self._next_task_id += 1
        self._order += 1
        return True

    def dispatch_next(self) -> ManagedTask | None:
        if self._active_task_id is not None:
            return self.current_task()
        candidates = [
            task
            for task in self._tasks.values()
            if task.status in (STATUS_NEW, STATUS_DISPATCHED)
        ]
        if not candidates:
            return None
        task = min(
            candidates,
            key=lambda item: (priority_for_task_type(item.task_type), item.discovered_order),
        )
        task = replace(task, status=STATUS_EN_ROUTE)
        self._tasks[task.key] = task
        self._active_task_id = task.task_id
        return task

    def current_task(self) -> ManagedTask | None:
        if self._active_task_id is None:
            return None
        for task in self._tasks.values():
            if task.task_id == self._active_task_id:
                return task
        self._active_task_id = None
        return None

    def mark_navigation_done(self, task_id: int) -> ManagedTask | None:
        task = self._find_task(task_id)
        if task is None or task.status != STATUS_EN_ROUTE:
            return task
        task = replace(task, status=STATUS_ACTING)
        self._tasks[task.key] = task
        return task

    def mark_task_done(self, task_id: int) -> ManagedTask | None:
        task = self._find_task(task_id)
        if task is None:
            return None
        task = replace(task, status=STATUS_DONE)
        self._tasks[task.key] = task
        if self._active_task_id == task_id:
            self._active_task_id = None
        return task

    def mark_failed(self, task_id: int) -> ManagedTask | None:
        task = self._find_task(task_id)
        if task is None:
            return None
        task = replace(task, status=STATUS_FAILED)
        self._tasks[task.key] = task
        if self._active_task_id == task_id:
            self._active_task_id = None
        return task

    def all_tasks(self) -> List[ManagedTask]:
        return sorted(
            self._tasks.values(),
            key=lambda item: (priority_for_task_type(item.task_type), item.discovered_order),
        )

    def _find_task(self, task_id: int) -> ManagedTask | None:
        for task in self._tasks.values():
            if task.task_id == task_id:
                return task
        return None
