"""Small static OccupancyGrid builder for the response formation demo."""

from __future__ import annotations

import math
from typing import Iterable, Mapping


def build_static_occupancy_grid(
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    obstacles: Iterable[Mapping[str, float]],
) -> list[int]:
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")

    data = [0] * (width * height)
    for my in range(height):
        y = origin_y + (my + 0.5) * resolution
        for mx in range(width):
            x = origin_x + (mx + 0.5) * resolution
            if _inside_any_obstacle(x, y, obstacles):
                data[my * width + mx] = 100
    return data


def _inside_any_obstacle(x: float, y: float, obstacles: Iterable[Mapping[str, float]]) -> bool:
    for obstacle in obstacles:
        obstacle_type = str(obstacle.get("type", "rectangle"))
        if obstacle_type == "rectangle":
            if _inside_rectangle(x, y, obstacle):
                return True
        elif obstacle_type == "circle":
            if _inside_circle(x, y, obstacle):
                return True
    return False


def _inside_rectangle(x: float, y: float, obstacle: Mapping[str, float]) -> bool:
    cx = float(obstacle["x"])
    cy = float(obstacle["y"])
    sx = float(obstacle["size_x"])
    sy = float(obstacle["size_y"])
    return abs(x - cx) <= 0.5 * sx and abs(y - cy) <= 0.5 * sy


def _inside_circle(x: float, y: float, obstacle: Mapping[str, float]) -> bool:
    cx = float(obstacle["x"])
    cy = float(obstacle["y"])
    radius = float(obstacle["radius"])
    return math.hypot(x - cx, y - cy) <= radius
