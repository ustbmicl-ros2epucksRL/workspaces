#!/usr/bin/env python3
"""Fit the overhead-camera affine transform from a four-column CSV file."""

from __future__ import annotations

import argparse
import csv
import math

from ros2swarm.real_robot.geometry import fit_planar_affine


def _read_pairs(path: str):
    camera_points = []
    world_points = []
    with open(path, newline="", encoding="utf-8") as csv_file:
        for line_number, row in enumerate(csv.reader(csv_file), start=1):
            if not row or all(not value.strip() for value in row):
                continue
            try:
                values = [float(value) for value in row]
            except ValueError:
                if not camera_points:
                    continue
                raise ValueError(
                    f"line {line_number} contains a non-numeric value"
                )
            if len(values) != 4:
                raise ValueError(
                    f"line {line_number} must have exactly four columns"
                )
            camera_points.append(values[:2])
            world_points.append(values[2:])
    return camera_points, world_points


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_file",
        help="CSV columns: camera_x,camera_y,world_x,world_y",
    )
    arguments = parser.parse_args()
    camera_points, world_points = _read_pairs(arguments.csv_file)
    transform, rmse = fit_planar_affine(camera_points, world_points)
    determinant = transform[0] * transform[4] - transform[1] * transform[3]
    values = ", ".join(f"{value:.9g}" for value in transform)
    print(f"camera_to_world: [{values}]")
    print(f"rmse_m: {rmse:.6g}")
    print(f"linear_determinant: {determinant:.6g}")
    if math.isclose(determinant, 0.0, abs_tol=1e-8):
        raise RuntimeError("calibration is degenerate")


if __name__ == "__main__":
    main()
