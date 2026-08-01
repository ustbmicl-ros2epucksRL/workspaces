from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from agents.runtime_agent.processes import (
    CommandNotAllowed,
    CommandPolicy,
    DEFAULT_COMMAND_POLICIES,
    GracePeriods,
    ProcessManager,
    RECORDER_REGEXES,
)


def _manager(tmp_path: Path) -> ProcessManager:
    return ProcessManager(
        agent_kind="test",
        workspace=tmp_path,
        runs_dir=tmp_path / "runs",
        command_policies={"test.python": CommandPolicy("test", sys.executable)},
    )


def _wait_for(predicate: object, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():  # type: ignore[operator]
            return
        time.sleep(0.01)
    raise AssertionError("condition did not become true")


def test_spawn_records_pid_pgid_run_command_and_argv(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    argv = [sys.executable, "-c", "import time; print('ready', flush=True); time.sleep(30)"]

    record = manager.start("run-1", "step-1", "test.python", argv)
    try:
        assert record.pid > 0
        assert record.pgid == record.pid
        assert os.getpgid(record.pid) == record.pgid
        assert record.run_id == "run-1"
        assert record.process_id == "step-1"
        assert record.command_ref == "test.python"
        assert list(record.argv) == argv
        persisted = json.loads(Path(record.record_path).read_text(encoding="utf-8"))
        assert persisted["pid"] == record.pid
        assert persisted["pgid"] == record.pgid
        assert persisted["argv"] == argv
    finally:
        manager.stop("run-1", "step-1", GracePeriods(0.2, 0.2, 0.2))


def test_unknown_command_ref_and_mismatched_executable_are_rejected(tmp_path: Path) -> None:
    manager = _manager(tmp_path)

    with pytest.raises(CommandNotAllowed):
        manager.start("run-1", "step-1", "unknown", [sys.executable, "-c", "pass"])
    with pytest.raises(CommandNotAllowed):
        manager.start("run-1", "step-1", "test.python", ["/bin/sh", "-c", "sleep 30"])

    assert manager.probe("run-1") == []


@pytest.mark.parametrize(
    ("agent", "command_ref", "argv", "expected_output"),
    (
        (
            "ros1",
            "record.ros1.exploration",
            [
                "rosbag",
                "record",
                "--regex",
                "--output-name",
                "@run/bags/ros1/recording",
                RECORDER_REGEXES["record.ros1.exploration"],
            ],
            "bags/ros1/recording",
        ),
        (
            "ros2",
            "record.ros2.swarm_sensor",
            [
                "ros2",
                "bag",
                "record",
                "--regex",
                RECORDER_REGEXES["record.ros2.swarm_sensor"],
                "--output",
                "@run/bags/ros2",
            ],
            "bags/ros2",
        ),
    ),
)
def test_recorder_commands_are_static_and_materialize_only_inside_the_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    agent: str,
    command_ref: str,
    argv: list[str],
    expected_output: str,
) -> None:
    captured: list[list[str]] = []

    class Child:
        pid = os.getpid()

        def poll(self):
            return None

    def fake_popen(actual, **_kwargs):
        captured.append(list(actual))
        return Child()

    monkeypatch.setattr(subprocess, "Popen", fake_popen)
    monkeypatch.setattr(os, "getpgid", lambda _pid: os.getpid())
    manager = ProcessManager(
        agent_kind=agent,
        workspace=tmp_path,
        runs_dir=tmp_path / "runs" / f"agent-{agent}",
        command_policies=DEFAULT_COMMAND_POLICIES,
    )

    manager.start("run-1", "recorder", command_ref, argv)

    assert captured
    assert str(tmp_path / "runs" / "run-1" / expected_output) in captured[0]
    assert "@run" not in " ".join(captured[0])

    unsafe = list(argv)
    unsafe[-1] = "--all"
    with pytest.raises(CommandNotAllowed):
        manager.start("run-2", "recorder", command_ref, unsafe)

    arbitrary = list(argv)
    regex_index = 5 if agent == "ros1" else 4
    arbitrary[regex_index] = "^(?:/.*)$"
    with pytest.raises(CommandNotAllowed):
        manager.start("run-3", "recorder", command_ref, arbitrary)


def test_stop_escalates_only_the_owned_process_group(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    ignore_signals = (
        "import signal,time; "
        "signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); "
        "print('ready', flush=True); time.sleep(30)"
    )
    external = subprocess.Popen(
        [sys.executable, "-c", ignore_signals],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    owned = manager.start("run-1", "step-1", "test.python", [sys.executable, "-c", ignore_signals])
    try:
        _wait_for(lambda: "ready" in manager.read_logs("run-1", "step-1", "stdout", 0).data)

        stopped = manager.stop("run-1", "step-1", GracePeriods(0.05, 0.05, 0.05))

        assert stopped.status == "STOPPED"
        assert stopped.signals_sent == ("SIGINT", "SIGTERM", "SIGKILL")
        assert external.poll() is None
        with pytest.raises(ProcessLookupError):
            os.getpgid(owned.pid)
    finally:
        if external.poll() is None:
            os.killpg(os.getpgid(external.pid), signal.SIGKILL)
        external.wait(timeout=3)


def test_log_cursor_reads_only_new_bytes(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    manager.start(
        "run-1",
        "step-1",
        "test.python",
        [sys.executable, "-c", "print('alpha'); print('beta')"],
    )
    _wait_for(lambda: manager.get("run-1", "step-1").status == "EXITED")

    first = manager.read_logs("run-1", "step-1", "stdout", after=0, limit=6)
    second = manager.read_logs("run-1", "step-1", "stdout", after=first.cursor, limit=1024)
    third = manager.read_logs("run-1", "step-1", "stdout", after=second.cursor, limit=1024)

    assert first.data == "alpha\n"
    assert second.data == "beta\n"
    assert third.data == ""
    assert third.cursor == second.cursor
    assert third.eof is True


def test_child_environment_always_enables_platform_mode_and_local_no_proxy(tmp_path: Path) -> None:
    manager = _manager(tmp_path)
    script = (
        "import os; "
        "print(os.environ['MASAR_PLATFORM_MODE']); "
        "print(os.environ['NO_PROXY']); "
        "print(os.environ['no_proxy'])"
    )
    manager.start(
        "run-1",
        "step-1",
        "test.python",
        [sys.executable, "-c", script],
        {"NO_PROXY": "example.internal", "no_proxy": "example.internal"},
    )
    _wait_for(lambda: manager.get("run-1", "step-1").status == "EXITED")

    lines = manager.read_logs("run-1", "step-1", "stdout", after=0).data.splitlines()
    assert lines[0] == "1"
    for value in lines[1:]:
        assert value.split(",") == ["example.internal", "127.0.0.1", "localhost", "::1"]


def test_persisted_owner_can_be_probed_and_stopped_after_manager_restart(tmp_path: Path) -> None:
    first = _manager(tmp_path)
    record = first.start(
        "run-1",
        "step-1",
        "test.python",
        [sys.executable, "-c", "import time; time.sleep(30)"],
    )
    second = _manager(tmp_path)

    recovered = second.get("run-1", "step-1")
    assert recovered.pid == record.pid
    assert recovered.pgid == record.pgid
    assert second.probe("run-1")[0].status == "RUNNING"

    stopped = second.stop("run-1", "step-1", GracePeriods(0.1, 0.1, 0.1))
    assert stopped.status == "STOPPED"


def test_failed_ownership_record_write_reaps_the_started_process(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    manager = _manager(tmp_path)
    captured: list[object] = []

    def fail_write(record: object) -> None:
        captured.append(record)
        raise OSError("disk write failed")

    monkeypatch.setattr(manager, "_write_record", fail_write)

    with pytest.raises(OSError, match="disk write failed"):
        manager.start(
            "run-1",
            "step-1",
            "test.python",
            [sys.executable, "-c", "import time; time.sleep(30)"],
        )

    assert captured
    record = captured[0]
    _wait_for(lambda: not Path("/proc", str(record.pid)).exists())  # type: ignore[attr-defined]
