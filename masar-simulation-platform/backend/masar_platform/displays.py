"""Resolved, immutable display transports for each scenario."""

from __future__ import annotations

from dataclasses import dataclass
from http.client import HTTPConnection
from collections.abc import Callable
from typing import Any

from .contracts import ScenarioManifest
from .profiles import get_display_profile


LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"
GAZEBO_DISPLAY = ":101"
RVIZ_DISPLAY = ":102"
RVIZ_PROCESS_ID = "display_rviz"
RVIZ_COMMAND_REF = "display.ros1.rviz"
RVIZ_CONFIG = "/root/workspace/masar_ws/src/robosar/robosar_sim_bringup/rviz/multi_robot.rviz"


@dataclass(frozen=True, slots=True)
class SimulationDisplay:
    transport: str
    path: str
    display: str | None
    required: bool = True

    def as_dict(self) -> dict[str, Any]:
        return {
            "transport": self.transport,
            "path": self.path,
            "display": self.display,
            "required": self.required,
        }


@dataclass(frozen=True, slots=True)
class RvizDisplay:
    agent: str
    process_id: str
    command_ref: str
    argv: tuple[str, ...]
    environment: dict[str, str]
    display: str
    required: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "agent": self.agent,
            "process_id": self.process_id,
            "command_ref": self.command_ref,
            "argv": list(self.argv),
            "environment": dict(self.environment),
            "required": self.required,
        }


@dataclass(frozen=True, slots=True)
class ScenarioDisplayPlan:
    simulation: SimulationDisplay
    rviz: RvizDisplay | None

    def as_dict(self) -> dict[str, Any]:
        return {
            "simulation": self.simulation.as_dict(),
            "rviz": None if self.rviz is None else self.rviz.as_dict(),
        }


def display_plan(manifest: ScenarioManifest) -> ScenarioDisplayPlan:
    """Return the exact browser transports and optional RViz process."""

    if manifest.simulator == "webots":
        simulation = SimulationDisplay("webots", "/webots/", None)
    else:
        simulation = SimulationDisplay("novnc", "/display/gazebo/", GAZEBO_DISPLAY)

    profile = get_display_profile(manifest.display_profile)
    if "rviz" not in profile.panels:
        return ScenarioDisplayPlan(simulation, None)

    environment = {
        "DISPLAY": RVIZ_DISPLAY,
        "MASAR_PLATFORM_MODE": "1",
        "NO_PROXY": LOCAL_NO_PROXY,
        "ROS_DOMAIN_ID": str(manifest.runtime.ros2_domain_id),
        "ROS_MASTER_URI": manifest.runtime.ros1_master_uri,
        "no_proxy": LOCAL_NO_PROXY,
    }
    rviz = RvizDisplay(
        agent="ros1",
        process_id=RVIZ_PROCESS_ID,
        command_ref=RVIZ_COMMAND_REF,
        argv=("rviz", "-d", RVIZ_CONFIG),
        environment=environment,
        display=RVIZ_DISPLAY,
    )
    return ScenarioDisplayPlan(simulation, rviz)


def check_display_prerequisites(
    manifest: ScenarioManifest,
    enabled: bool,
    *,
    gazebo_port: int = 18_131,
    rviz_port: int = 18_132,
    probe: Callable[[int], bool] | None = None,
) -> tuple[bool, str]:
    """Check only the persistent transports required before a run starts."""

    if not enabled:
        return False, "visual gateways are disabled until Tasks 5-7 are accepted"
    plan = display_plan(manifest)
    required_ports: list[tuple[str, int]] = []
    if plan.simulation.transport == "novnc":
        required_ports.append(("gazebo", gazebo_port))
    if plan.rviz is not None:
        required_ports.append(("rviz", rviz_port))
    check = probe or _novnc_http_ready
    unavailable = [name for name, port in required_ports if not check(port)]
    if unavailable:
        return False, f"display gateway unavailable: {', '.join(unavailable)}"
    if not required_ports:
        return True, "Webots stream is started and checked as part of the scenario"
    return True, "display gateway ready: " + ", ".join(name for name, _port in required_ports)


def _novnc_http_ready(port: int) -> bool:
    connection = HTTPConnection("127.0.0.1", port, timeout=1.0)
    try:
        connection.request("GET", "/vnc.html")
        response = connection.getresponse()
        response.read(1)
        return response.status == 200
    except OSError:
        return False
    finally:
        connection.close()


__all__ = [
    "GAZEBO_DISPLAY",
    "LOCAL_NO_PROXY",
    "RVIZ_COMMAND_REF",
    "RVIZ_CONFIG",
    "RVIZ_DISPLAY",
    "RVIZ_PROCESS_ID",
    "RvizDisplay",
    "ScenarioDisplayPlan",
    "SimulationDisplay",
    "display_plan",
    "check_display_prerequisites",
]
