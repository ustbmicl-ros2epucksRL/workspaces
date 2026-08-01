#!/usr/bin/env python3
"""Capture immutable pre-change Webots launcher dry-run output."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_DIR = REPO_ROOT / "tests" / "fixtures" / "baseline"
SCRIPTS = ("start_simulation.sh", "start_robosar_webots.sh")


def main() -> int:
    commit = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout.strip()
    environment = os.environ.copy()
    environment.update(
        {
            "WORKSPACE_DIR": str(REPO_ROOT),
            "ROS2SWARM_VNC_DISPLAY": ":99",
            "ROS2SWARM_XAUTHORITY": "/tmp/masar-platform-baseline.xauth",
        }
    )

    BASELINE_DIR.mkdir(parents=True, exist_ok=True)
    for script in SCRIPTS:
        result = subprocess.run(
            ["bash", str(REPO_ROOT / script), "--dry-run", "--no-vglrun"],
            cwd=REPO_ROOT,
            env=environment,
            check=True,
            capture_output=True,
            text=True,
        )
        if result.stderr:
            raise RuntimeError(f"{script} wrote unexpected stderr: {result.stderr}")
        output_path = BASELINE_DIR / f"{Path(script).stem}.{commit}.stdout.txt"
        output_path.write_text(result.stdout, encoding="utf-8")
        print(output_path.relative_to(REPO_ROOT))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
