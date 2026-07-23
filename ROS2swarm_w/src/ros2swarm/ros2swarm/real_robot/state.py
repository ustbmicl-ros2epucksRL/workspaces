"""Marker-to-entity mapping and velocity estimation."""

from __future__ import annotations

from dataclasses import dataclass

from .geometry import wrap_to_pi


@dataclass(frozen=True)
class EntityEstimate:
    """Filtered planar state for one tracked entity."""

    name: str
    entity_type: int
    x: float
    y: float
    yaw: float
    vx: float
    vy: float
    pose_valid: bool
    twist_valid: bool
    confidence: float
    stamp: float


class EntityTracker:
    """Alpha-filtered pose tracker with bounded prediction gaps."""

    def __init__(self, *, velocity_alpha: float = 0.65, max_dt: float = 0.5):
        self.velocity_alpha = min(1.0, max(0.0, float(velocity_alpha)))
        self.max_dt = max(1e-3, float(max_dt))
        self._states = {}

    def update(
        self,
        *,
        name: str,
        entity_type: int,
        x: float,
        y: float,
        yaw: float,
        stamp: float,
        confidence: float,
    ) -> EntityEstimate:
        stamp = float(stamp)
        previous = self._states.get(name)
        x = float(x)
        y = float(y)
        yaw = float(yaw)
        vx = 0.0
        vy = 0.0
        if previous is not None:
            dt = stamp - previous.stamp
            if 1e-4 <= dt <= self.max_dt:
                raw_vx = (x - previous.x) / dt
                raw_vy = (y - previous.y) / dt
                alpha = self.velocity_alpha
                vx = alpha * raw_vx + (1.0 - alpha) * previous.vx
                vy = alpha * raw_vy + (1.0 - alpha) * previous.vy
            elif dt <= 0.0:
                x, y, yaw = previous.x, previous.y, previous.yaw
                vx, vy = previous.vx, previous.vy

        estimate = EntityEstimate(
            name=str(name),
            entity_type=int(entity_type),
            x=x,
            y=y,
            yaw=wrap_to_pi(yaw),
            vx=vx,
            vy=vy,
            pose_valid=True,
            twist_valid=previous is not None,
            confidence=min(1.0, max(0.0, float(confidence))),
            stamp=stamp,
        )
        self._states[name] = estimate
        return estimate

    def invalidate_stale(
        self,
        now: float,
        max_age: float,
    ) -> list[EntityEstimate]:
        """Return current estimates with stale entities marked invalid."""
        output = []
        for estimate in self._states.values():
            age = float(now) - estimate.stamp
            if age > float(max_age) or age < -float(max_age):
                output.append(
                    EntityEstimate(
                        **{
                            **estimate.__dict__,
                            "pose_valid": False,
                            "twist_valid": False,
                        }
                    )
                )
            else:
                output.append(estimate)
        return output
