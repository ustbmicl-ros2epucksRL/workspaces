from __future__ import annotations

import os
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[2]
ROS1_IMAGE = (
    "talent18/masar_ros1_srv-g:v2@"
    "sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59"
)
ROS2_IMAGE = (
    "talent18/masar_ros2_bridge-w:v2@"
    "sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c"
)
NODE_IMAGE = (
    "node:22-alpine@"
    "sha256:c610fcdfb1d5b4740dd70c284ed3cb16bb857e0f7166196e36a5501df7a3aa32"
)


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content, encoding="utf-8")
    path.chmod(0o755)


def _prepare_harness(tmp_path: Path) -> tuple[dict[str, str], Path, Path]:
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    state_dir = tmp_path / "state"
    state_dir.mkdir()
    workspace = tmp_path / "workspace"
    (workspace / "masar_ws").mkdir(parents=True)
    (workspace / "ROS2swarm_w").mkdir()
    (workspace / "masar-simulation-platform").symlink_to(ROOT, target_is_directory=True)
    xauthority = tmp_path / "Xauthority"
    xauthority.touch()
    secret = tmp_path / "agent.secret"
    secret.write_bytes(b"s" * 32)
    secret.chmod(0o600)
    runtime_dir = tmp_path / "runtime"
    runtime_dir.mkdir()

    _write_executable(
        fake_bin / "docker",
        r'''#!/usr/bin/env bash
set -eu
printf '%s\n' "$*" >> "$FAKE_DOCKER_LOG"
case "${1:-} ${2:-}" in
  "info ") exit 0 ;;
  "compose version") printf 'Docker Compose version v5.2.0\n'; exit 0 ;;
  "image inspect")
    image=${!#}
    if [[ $image == *masar_ros1_srv-g* ]]; then marker=ros1-image; else marker=ros2-image; fi
    [[ -f "$FAKE_STATE/$marker" ]]
    exit
    ;;
esac
case "${1:-}" in
  pull)
    if [[ ${2:-} == *masar_ros1_srv-g* ]]; then marker=ros1-image; else marker=ros2-image; fi
    touch "$FAKE_STATE/$marker"
    exit 0
    ;;
  inspect)
    name=${!#}
    marker=${name//\//_}
    [[ -f "$FAKE_STATE/$marker" ]] || exit 1
    if [[ "$*" == *com.docker.compose.project* ]]; then
      if [[ -f "$FAKE_STATE/$marker.compose" ]]; then
        printf 'masar-simulation-platform\n'
      fi
    else
      printf 'true\n'
    fi
    exit 0
    ;;
  stop|rm)
    exit 0
    ;;
  compose)
    if [[ "$*" == *" run "* && "$*" == *ros1-runtime* ]]; then
      mkdir -p "$FAKE_WORKSPACE/masar_ws/devel"
      touch "$FAKE_WORKSPACE/masar_ws/devel/setup.bash"
      exit 0
    fi
    if [[ "$*" == *" run "* && "$*" == *ros2-runtime* ]]; then
      mkdir -p "$FAKE_WORKSPACE/ROS2swarm_w/install"
      touch "$FAKE_WORKSPACE/ROS2swarm_w/install/setup.bash"
      exit 0
    fi
    if [[ "$*" == *" up "* ]]; then
      touch "$FAKE_STATE/gateway"
      touch "$FAKE_STATE/compose-running"
      exit 0
    fi
    if [[ "$*" == *" down"* ]]; then
      rm -f "$FAKE_STATE/gateway" "$FAKE_STATE/compose-running"
      exit 0
    fi
    if [[ "$*" == *" config --services"* ]]; then
      printf '%s\n' api ros1-runtime ros2-runtime ros1-web ros2-web webots-guard display caddy
      exit 0
    fi
    if [[ "$*" == *" ps "* || "$*" == *" ps" ]]; then
      if [[ -f "$FAKE_STATE/compose-running" ]]; then
        for service in api ros1-runtime ros2-runtime ros1-web ros2-web webots-guard display caddy; do
          printf '{"Service":"%s","State":"running","Health":"healthy"}\n' "$service"
        done
      fi
      exit 0
    fi
    exit 0
    ;;
esac
exit 0
''',
    )
    _write_executable(
        fake_bin / "curl",
        r'''#!/usr/bin/env bash
set -eu
printf '%s\n' "$*" >> "$FAKE_CURL_LOG"
url=${!#}
if [[ "$*" == *"POST"* && $url == */stop ]]; then
  rm -f "$FAKE_STATE/current-run"
  printf '{"run":{"state":"COMPLETED","reason":"USER_STOP"}}\n'
  exit 0
fi
case "$url" in
  */api/v1/runs/current)
    if [[ -f "$FAKE_STATE/current-run" ]]; then
      run_id=$(cat "$FAKE_STATE/current-run")
      printf '{"run":{"run_id":"%s","state":"RUNNING"}}\n' "$run_id"
    else
      printf '{"run":null}\n'
    fi
    ;;
  */api/v1/capabilities)
    [[ -f "$FAKE_STATE/gateway" ]] || exit 22
    printf '{"workspaces":[],"scenarios":[]}\n'
    ;;
  *:18101/healthz)
    printf '{"data":{"agent":"ros1","status":"ok","capabilities":[]},"ok":true}\n'
    ;;
  *:18102/healthz)
    printf '{"data":{"agent":"ros2","status":"ok","capabilities":["webots_preflight"]},"ok":true}\n'
    ;;
  *) exit 22 ;;
esac
''',
    )
    _write_executable(fake_bin / "uname", "#!/usr/bin/env bash\nprintf 'x86_64\\n'\n")
    _write_executable(fake_bin / "nvidia-smi", "#!/usr/bin/env bash\nexit 0\n")
    _write_executable(fake_bin / "xauth", "#!/usr/bin/env bash\nexit 0\n")
    _write_executable(
        fake_bin / "ss",
        r'''#!/usr/bin/env bash
set -eu
if [[ -n ${FAKE_PORT_OWNER:-} ]]; then
  printf 'LISTEN 0 128 127.0.0.1:18101 0.0.0.0:* users:(("%s",pid=999,fd=3))\n' "$FAKE_PORT_OWNER"
fi
''',
    )

    docker_log = tmp_path / "docker.log"
    curl_log = tmp_path / "curl.log"
    docker_log.touch()
    curl_log.touch()
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "FAKE_STATE": str(state_dir),
            "FAKE_WORKSPACE": str(workspace),
            "FAKE_DOCKER_LOG": str(docker_log),
            "FAKE_CURL_LOG": str(curl_log),
            "MASAR_WORKSPACE_ROOT": str(workspace),
            "MASAR_XAUTHORITY": str(xauthority),
            "MASAR_PLATFORM_SECRET_PATH": str(secret),
            "MASAR_RUNTIME_DIR": str(runtime_dir),
            "MASAR_PLATFORM_SKIP_DEVICE_CHECKS": "1",
        }
    )
    return environment, docker_log, curl_log


def _run_platform(environment: dict[str, str], command: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        ["bash", str(ROOT / "deploy" / "platform.sh"), command],
        cwd=ROOT,
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )


def test_doctor_normalizes_an_explicit_workspace_symlink(tmp_path: Path) -> None:
    environment, _, _ = _prepare_harness(tmp_path)
    workspace = Path(environment["FAKE_WORKSPACE"])
    alias = tmp_path / "portable-checkout-root"
    alias.symlink_to(workspace, target_is_directory=True)
    environment["MASAR_WORKSPACE_ROOT"] = str(alias)

    result = _run_platform(environment, "doctor")

    assert result.returncode == 0, result.stdout + result.stderr
    assert f"workspace root: {workspace.resolve()}" in result.stdout


def test_invalid_workspace_root_fails_before_docker(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    environment["MASAR_WORKSPACE_ROOT"] = str(tmp_path / "missing-root")

    result = _run_platform(environment, "doctor")

    assert result.returncode != 0
    assert "workspace root" in result.stderr
    assert docker_log.read_text(encoding="utf-8") == ""


def test_init_skips_pull_when_both_local_digests_match(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    (state / "ros1-image").touch()
    (state / "ros2-image").touch()

    result = _run_platform(environment, "init")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    assert sum(call.startswith("image inspect ") for call in calls) == 2
    assert not any(call.startswith("pull ") for call in calls)
    assert "digest matched; skipping pull" in result.stdout


def test_init_pulls_only_the_missing_runtime_image(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    (Path(environment["FAKE_STATE"]) / "ros1-image").touch()

    result = _run_platform(environment, "init")

    assert result.returncode == 0, result.stdout + result.stderr
    pulls = [call.removeprefix("pull ") for call in docker_log.read_text(encoding="utf-8").splitlines() if call.startswith("pull ")]
    assert pulls == [ROS2_IMAGE]


def test_start_builds_missing_ros_overlays_before_compose_up(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    (state / "ros1-image").touch()
    (state / "ros2-image").touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    ros1_build = next(index for index, call in enumerate(calls) if " run " in call and "ros1-runtime" in call)
    ros2_build = next(index for index, call in enumerate(calls) if " run " in call and "ros2-runtime" in call)
    compose_up = next(index for index, call in enumerate(calls) if " up " in call)
    assert ros1_build < ros2_build < compose_up


def test_start_skips_existing_overlays(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    (state / "ros1-image").touch()
    (state / "ros2-image").touch()
    workspace = Path(environment["FAKE_WORKSPACE"])
    (workspace / "masar_ws" / "devel").mkdir()
    (workspace / "masar_ws" / "devel" / "setup.bash").touch()
    (workspace / "ROS2swarm_w" / "install").mkdir()
    (workspace / "ROS2swarm_w" / "install" / "setup.bash").touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    assert not any(" run " in call and "-runtime" in call for call in calls)
    assert "overlay already available; skipping build" in result.stdout


def test_start_builds_frontend_assets_in_node_container_before_compose_up(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    (state / "ros1-image").touch()
    (state / "ros2-image").touch()
    environment["MASAR_PLATFORM_FORCE_FRONTEND_BUILD"] = "1"
    workspace = Path(environment["FAKE_WORKSPACE"])
    for setup in (
        workspace / "masar_ws" / "devel" / "setup.bash",
        workspace / "ROS2swarm_w" / "install" / "setup.bash",
    ):
        setup.parent.mkdir(parents=True)
        setup.touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    frontend_build = next(
        index for index, call in enumerate(calls) if call.startswith("run ") and NODE_IMAGE in call
    )
    compose_up = next(index for index, call in enumerate(calls) if " up " in call)
    assert frontend_build < compose_up
    assert not any("build caddy" in call for call in calls)


def test_start_is_idempotent_when_all_services_are_healthy(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    for marker in ("ros1-image", "ros2-image", "gateway", "compose-running"):
        (state / marker).touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    assert not any(call.startswith(("stop ", "rm ")) for call in calls)
    assert not any(" down" in call or " up " in call for call in calls)
    assert "already healthy" in result.stdout


def test_start_reports_one_to_one_gateway_and_ssh_entry(tmp_path: Path) -> None:
    environment, _, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    for marker in ("ros1-image", "ros2-image", "gateway", "compose-running"):
        (state / marker).touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "http://127.0.0.1:18090" in result.stdout
    assert "LocalForward 18090 127.0.0.1:18090" in result.stdout
    assert "18089" not in result.stdout
    assert "7897" not in result.stdout


def test_start_migrates_only_exact_legacy_container_names(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    for name in ("ros1-image", "ros2-image", "masar_ros1_srv-g", "masar_ros2_bridge-w", "unrelated"):
        (state / name).touch()
    workspace = Path(environment["FAKE_WORKSPACE"])
    for setup in (
        workspace / "masar_ws" / "devel" / "setup.bash",
        workspace / "ROS2swarm_w" / "install" / "setup.bash",
    ):
        setup.parent.mkdir(parents=True)
        setup.touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    assert "stop masar_ros1_srv-g" in calls
    assert "rm masar_ros1_srv-g" in calls
    assert "stop masar_ros2_bridge-w" in calls
    assert "rm masar_ros2_bridge-w" in calls
    assert not any("unrelated" in call for call in calls)


def test_start_refuses_unrelated_port_owner(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    (state / "ros1-image").touch()
    (state / "ros2-image").touch()
    environment["FAKE_PORT_OWNER"] = "rogue"

    result = _run_platform(environment, "start")

    assert result.returncode != 0
    assert "unrelated process owns platform port" in result.stderr
    calls = docker_log.read_text(encoding="utf-8").splitlines()
    assert not any(call.startswith(("stop ", "rm ")) or " up " in call for call in calls)


def test_stop_requests_active_run_stop_before_compose_down(tmp_path: Path) -> None:
    environment, docker_log, curl_log = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    (state / "gateway").touch()
    (state / "current-run").write_text("run-active", encoding="utf-8")

    result = _run_platform(environment, "stop")

    assert result.returncode == 0, result.stdout + result.stderr
    curls = curl_log.read_text(encoding="utf-8").splitlines()
    assert any("POST" in call and "/runs/run-active/stop" in call for call in curls)
    assert any(" down" in call or call.endswith(" down") for call in docker_log.read_text(encoding="utf-8").splitlines())


def test_makefile_exposes_only_supported_deployment_targets() -> None:
    expected = {
        "docker-init": "./deploy/platform.sh init",
        "docker-start": "./deploy/platform.sh start",
        "docker-stop": "./deploy/platform.sh stop",
        "docker-status": "./deploy/platform.sh status",
        "docker-logs": "./deploy/platform.sh logs api",
        "docker-doctor": "./deploy/platform.sh doctor",
    }
    for target, command in expected.items():
        arguments = ["make", "-n", target]
        if target == "docker-logs":
            arguments.append("SERVICE=api")
        completed = subprocess.run(
            arguments, cwd=ROOT, text=True, capture_output=True, check=False
        )
        assert completed.returncode == 0, completed.stderr
        assert command in completed.stdout
