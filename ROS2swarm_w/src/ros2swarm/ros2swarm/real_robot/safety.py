"""Pure safety functions shared by the ROS node and unit tests."""

from __future__ import annotations

from dataclasses import dataclass

from geometry_msgs.msg import Twist


@dataclass(frozen=True)
class SafetyLimits:
    max_linear_velocity: float
    max_angular_velocity: float
    wheel_separation: float
    max_wheel_linear_velocity: float
    max_linear_acceleration: float
    max_angular_acceleration: float


def clamp_twist_to_limits(
    command: Twist,
    limits: SafetyLimits,
    *,
    previous: Twist | None = None,
    dt: float | None = None,
) -> Twist:
    """Apply velocity, wheel-speed and optional acceleration limits."""
    output = Twist()
    linear = max(
        -limits.max_linear_velocity,
        min(limits.max_linear_velocity, float(command.linear.x)),
    )
    angular = max(
        -limits.max_angular_velocity,
        min(limits.max_angular_velocity, float(command.angular.z)),
    )

    if (
        limits.wheel_separation > 0.0
        and limits.max_wheel_linear_velocity > 0.0
    ):
        half_track = limits.wheel_separation / 2.0
        left = linear - angular * half_track
        right = linear + angular * half_track
        requested = max(abs(left), abs(right))
        if requested > limits.max_wheel_linear_velocity:
            scale = limits.max_wheel_linear_velocity / requested
            linear *= scale
            angular *= scale

    if previous is not None and dt is not None and dt > 1e-6:
        if limits.max_linear_acceleration > 0.0:
            delta = limits.max_linear_acceleration * dt
            linear = max(
                float(previous.linear.x) - delta,
                min(float(previous.linear.x) + delta, linear),
            )
        if limits.max_angular_acceleration > 0.0:
            delta = limits.max_angular_acceleration * dt
            angular = max(
                float(previous.angular.z) - delta,
                min(float(previous.angular.z) + delta, angular),
            )

    output.linear.x = linear
    output.angular.z = angular
    return output
