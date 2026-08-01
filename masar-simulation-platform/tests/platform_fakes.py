from __future__ import annotations

from pathlib import Path
from typing import Any

from masar_platform.artifacts import ArtifactStore
from masar_platform.catalog import ScenarioCatalog
from masar_platform.db import Database
from masar_platform.events import EventStore
from masar_platform.leases import LeaseStore
from masar_platform.orchestrator import Orchestrator, PreflightCheck
from masar_platform.state_machine import RunStateMachine


ROOT = Path(__file__).resolve().parents[1]


class FakeAgent:
    def __init__(
        self,
        kind: str,
        trace: list[str],
        *,
        ready: bool = True,
        health_ok: bool = True,
        exit_during_readiness: bool = False,
    ) -> None:
        self.kind = kind
        self.trace = trace
        self.ready = ready
        self.health_ok = health_ok
        self.exit_during_readiness = exit_during_readiness
        self.processes: dict[str, dict[str, Any]] = {}

    def health(self) -> dict[str, Any]:
        self.trace.append(f"health:{self.kind}")
        if not self.health_ok:
            from masar_platform.agents import AgentUnavailable

            raise AgentUnavailable("offline")
        return {"agent": self.kind, "protocol_version": 1, "status": "ok"}

    def start(self, run_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        process_id = payload["process_id"]
        self.trace.append(f"start:{self.kind}:{process_id}")
        record = {
            "run_id": run_id,
            "process_id": process_id,
            "status": "RUNNING",
            "pgid": len(self.processes) + 100,
        }
        self.processes[process_id] = record
        return record

    def stop(self, run_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        process_id = payload["process_id"]
        self.trace.append(f"stop:{self.kind}:{process_id}")
        record = self.processes.setdefault(
            process_id,
            {"run_id": run_id, "process_id": process_id, "pgid": 100},
        )
        record["status"] = "STOPPED"
        return record

    def probe(self, run_id: str) -> dict[str, Any]:
        self.trace.append(f"probe:{self.kind}")
        return {"run_id": run_id, "processes": list(self.processes.values())}

    def probe_readiness(self, run_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        process = list(self.processes.values())[-1]
        if self.exit_during_readiness:
            process["status"] = "EXITED"
        self.trace.append(f"ready:{self.kind}:{process['process_id']}:{self.ready}")
        return {
            "run_id": run_id,
            "processes": [dict(process)],
            "probes": [
                {
                    "type": probe["type"],
                    "name": probe["name"],
                    "required": probe.get("required", True),
                    "ready": self.ready,
                    "status": "READY" if self.ready else "STALE",
                }
                for probe in payload["probes"]
            ],
        }

    def logs(self, run_id: str, process_id: str, **kwargs: Any) -> dict[str, Any]:
        return {
            "run_id": run_id,
            "process_id": process_id,
            "stream": kwargs.get("stream", "stdout"),
            "data": "fake log\n",
            "cursor": 9,
            "eof": False,
        }

    def command(self, run_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.trace.append(f"command:{self.kind}:{payload['action']}")
        return {"run_id": run_id, "action": payload["action"], "published": True}

    def graph_snapshot(self, run_id: str, payload: dict[str, Any]) -> dict[str, Any]:
        self.trace.append(f"graph:{self.kind}")
        return {
            "run_id": run_id,
            "agent": self.kind,
            "nodes": [],
            "topics": [],
            "available": True,
            "nodes_available": True,
            "topics_available": True,
        }


class FakePreflight:
    def __init__(self, trace: list[str]) -> None:
        self.trace = trace

    def check(self, manifest):
        self.trace.append(f"preflight:{manifest.scenario_id}")
        return (PreflightCheck("fake", "PASS", "test preflight"),)


def build_test_orchestrator(
    tmp_path: Path,
    *,
    ros1_ready: bool = True,
    ros2_ready: bool = True,
    ros1_health: bool = True,
    ros2_health: bool = True,
    ros1_exit: bool = False,
    ros2_exit: bool = False,
    real_preflight: bool = False,
    owner_id: str = "test-api",
    max_readiness_attempts: int | None = 2,
) -> tuple[Orchestrator, dict[str, FakeAgent], list[str]]:
    from masar_platform.orchestrator import PreflightChecker

    database = Database(tmp_path / "platform.sqlite3")
    database.initialize()
    events = EventStore(database, tmp_path / "runs")
    leases = LeaseStore(database, events)
    states = RunStateMachine(database, events)
    artifacts = ArtifactStore(database, tmp_path / "runs")
    trace: list[str] = []
    agents = {
        "ros1": FakeAgent(
            "ros1",
            trace,
            ready=ros1_ready,
            health_ok=ros1_health,
            exit_during_readiness=ros1_exit,
        ),
        "ros2": FakeAgent(
            "ros2",
            trace,
            ready=ros2_ready,
            health_ok=ros2_health,
            exit_during_readiness=ros2_exit,
        ),
    }
    preflight = PreflightChecker(tmp_path / "runs", agents) if real_preflight else FakePreflight(trace)
    orchestrator = Orchestrator(
        ScenarioCatalog.from_directory(ROOT / "scenarios"),
        states,
        leases,
        events,
        artifacts,
        agents,
        preflight,  # type: ignore[arg-type]
        owner_id=owner_id,
        readiness_poll_seconds=0,
        max_readiness_attempts=max_readiness_attempts,
        sleep=lambda _seconds: None,
    )
    return orchestrator, agents, trace
