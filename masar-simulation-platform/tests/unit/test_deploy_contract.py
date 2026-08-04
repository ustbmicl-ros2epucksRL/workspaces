from pathlib import Path
import subprocess

import yaml
from uvicorn.config import Config


ROOT = Path(__file__).resolve().parents[2]
LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"
ROS1_IMAGE = (
    "talent18/masar_ros1_srv-g:v2@"
    "sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59"
)
ROS2_IMAGE = (
    "talent18/masar_ros2_bridge-w:v2@"
    "sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c"
)


def test_api_runtime_has_a_websocket_protocol_implementation() -> None:
    config = Config("masar_platform.main:app")
    config.load()

    assert config.ws_protocol_class is not None


def test_every_compose_service_excludes_loopback_from_proxy() -> None:
    document = yaml.safe_load((ROOT / "deploy" / "compose.yaml").read_text(encoding="utf-8"))

    assert document["services"]
    for service_name, service in document["services"].items():
        environment = service.get("environment", {})
        assert environment.get("NO_PROXY") == LOCAL_NO_PROXY, service_name
        assert environment.get("no_proxy") == LOCAL_NO_PROXY, service_name


def test_compose_configuration_is_valid() -> None:
    completed = subprocess.run(
        ["docker", "compose", "-f", str(ROOT / "deploy" / "compose.yaml"), "config", "--quiet"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr


def test_compose_api_reads_the_shared_agent_secret_read_only() -> None:
    document = yaml.safe_load((ROOT / "deploy" / "compose.yaml").read_text(encoding="utf-8"))

    assert "../.runtime/agent.secret:/run/masar-platform/agent.secret:ro" in document["services"][
        "api"
    ]["volumes"]


def test_compose_api_can_read_restrictive_development_bind_mounts() -> None:
    document = yaml.safe_load((ROOT / "deploy" / "compose.yaml").read_text(encoding="utf-8"))

    # The API mounts the checkout for edit-and-restart development. A checkout
    # may contain owner-only source files, so the container must not rely on
    # the image's unrelated numeric non-root UID to import the mounted package.
    assert document["services"]["api"]["user"] == "0:0"


def test_compose_owns_both_business_runtimes() -> None:
    document = yaml.safe_load((ROOT / "deploy" / "compose.yaml").read_text(encoding="utf-8"))
    services = document["services"]

    assert set(services) == {
        "api",
        "ros1-runtime",
        "ros2-runtime",
        "ros1-web",
        "ros2-web",
        "webots-guard",
        "display",
        "caddy",
    }
    assert "profiles" not in services["api"]

    expected = {
        "ros1-runtime": (ROS1_IMAGE, "masar_ros1_srv-g", "ros1", "18101"),
        "ros2-runtime": (ROS2_IMAGE, "masar_ros2_bridge-w", "ros2", "18102"),
    }
    for service_name, (image, container_name, agent, port) in expected.items():
        service = services[service_name]
        assert service["image"] == image
        assert service["container_name"] == container_name
        assert service["network_mode"] == "host"
        assert service["ipc"] == "host"
        assert service["init"] is True
        assert service["gpus"] == "all"
        assert service["group_add"] == ["video"]
        assert service["devices"] == ["/dev/dri:/dev/dri"]
        assert service["restart"] == "unless-stopped"
        assert "${MASAR_WORKSPACE_ROOT:-../..}:/root/workspace:rw" in service["volumes"]
        assert "${MASAR_XAUTHORITY:-${HOME}/.Xauthority}:/tmp/.docker.xauth:ro" in service["volumes"]
        assert "/tmp/.X11-unix:/tmp/.X11-unix:rw" in service["volumes"]
        assert "../.runtime/agent.secret:/run/masar-platform/agent.secret:ro" in service["volumes"]
        command = service["command"]
        assert command[:2] == [
            "/root/workspace/masar-simulation-platform/deploy/agent-bootstrap.sh",
            agent,
        ]
        assert service["healthcheck"]["test"][-1].find(port) >= 0

    assert services["ros2-runtime"]["environment"]["ROS_DOMAIN_ID"] == "${MASAR_ROS2_DOMAIN_ID:-42}"
    assert services["api"]["depends_on"] == {
        "ros1-runtime": {"condition": "service_healthy"},
        "ros2-runtime": {"condition": "service_healthy"},
    }
    assert services["ros1-web"]["depends_on"] == {"ros1-runtime": {"condition": "service_healthy"}}
    assert services["ros2-web"]["depends_on"] == {"ros2-runtime": {"condition": "service_healthy"}}


def test_runtime_services_mount_mutable_platform_content() -> None:
    compose = yaml.safe_load((ROOT / "deploy" / "compose.yaml").read_text(encoding="utf-8"))
    services = compose["services"]

    api_volumes = services["api"]["volumes"]
    assert "../backend:/opt/masar-platform/backend:ro" in api_volumes
    assert "../scenarios:/opt/masar-platform/scenarios:ro" in api_volumes
    assert services["api"]["environment"]["PYTHONPATH"] == "/opt/masar-platform/backend"

    assert "./sidecars/run-ros1-sidecar:/usr/local/bin/run-ros1-sidecar:ro" in services[
        "ros1-web"
    ]["volumes"]
    assert "./sidecars/ros1-supervisord.conf:/etc/supervisor/supervisord.conf:ro" in services[
        "ros1-web"
    ]["volumes"]
    assert "./sidecars/run-ros2-sidecar:/usr/local/bin/run-ros2-sidecar:ro" in services[
        "ros2-web"
    ]["volumes"]
    assert "./sidecars/ros2-supervisord.conf:/etc/supervisor/supervisord.conf:ro" in services[
        "ros2-web"
    ]["volumes"]

    assert "./display/display_manager.py:/usr/local/bin/display_manager.py:ro" in services[
        "display"
    ]["volumes"]
    assert "./novnc/supervisord.conf:/etc/supervisor/conf.d/masar-display.conf:ro" in services[
        "display"
    ]["volumes"]
    assert "./webots-guard/webots-port-guard:/usr/local/bin/webots-port-guard:ro" in services[
        "webots-guard"
    ]["volumes"]

    caddy = services["caddy"]
    assert caddy["image"].startswith("caddy:2.10.2-alpine@sha256:")
    assert "./caddy/Caddyfile:/etc/caddy/Caddyfile:ro" in caddy["volumes"]
    assert "../frontend/dist:/srv/frontend:ro" in caddy["volumes"]
    assert "build" not in caddy


def test_bind_mounted_runtime_entrypoints_are_executable() -> None:
    entrypoints = (
        ROOT / "deploy" / "sidecars" / "run-ros1-sidecar",
        ROOT / "deploy" / "sidecars" / "run-ros2-sidecar",
        ROOT / "deploy" / "sidecars" / "sidecar-health",
        ROOT / "deploy" / "display" / "display_manager.py",
        ROOT / "deploy" / "display" / "xstartup-gazebo",
        ROOT / "deploy" / "display" / "xstartup-rviz",
        ROOT / "deploy" / "display" / "xstartup-webots",
        ROOT / "deploy" / "webots-guard" / "webots-port-guard",
    )

    for entrypoint in entrypoints:
        assert entrypoint.stat().st_mode & 0o111, entrypoint



def test_ros1_web_sidecar_can_decode_gazebo_world_state_messages() -> None:
    dockerfile = (ROOT / "deploy" / "ros1-web" / "Dockerfile").read_text(encoding="utf-8")

    assert "ros-noetic-gazebo-msgs" in dockerfile
