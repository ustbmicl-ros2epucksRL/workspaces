from pathlib import Path
import re
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEPLOY = ROOT / "deploy"
LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"


def _compose() -> dict:
    return yaml.safe_load((DEPLOY / "compose.yaml").read_text(encoding="utf-8"))


def _manifest(name: str) -> dict:
    return yaml.safe_load((DEPLOY / "sidecars" / f"{name}-supervisor.yaml").read_text(encoding="utf-8"))


def test_compose_defines_loopback_only_web_stack() -> None:
    services = _compose()["services"]

    assert {"api", "ros1-web", "ros2-web", "caddy"} <= services.keys()
    for name in ("api", "ros1-web", "ros2-web", "caddy"):
        service = services[name]
        assert service["network_mode"] == "host", name
        assert service["environment"]["NO_PROXY"] == LOCAL_NO_PROXY, name
        assert service["environment"]["no_proxy"] == LOCAL_NO_PROXY, name
        assert "ports" not in service, name

    assert services["ros1-web"]["environment"]["ROS_MASTER_URI"] == "http://127.0.0.1:11311"
    assert services["ros2-web"]["environment"]["ROS_DOMAIN_ID"] == "${MASAR_ROS2_DOMAIN_ID:-0}"
    assert services["ros2-web"]["environment"]["ROS_LOCALHOST_ONLY"] == "${MASAR_ROS2_LOCALHOST_ONLY:-0}"
    assert services["ros2-web"]["ipc"] == "host"
    assert services["caddy"]["image"].startswith("caddy:2.10.2-alpine@sha256:")


def test_sidecar_images_pin_ros_distro_and_packages() -> None:
    ros1 = (DEPLOY / "ros1-web" / "Dockerfile").read_text(encoding="utf-8")
    ros2 = (DEPLOY / "ros2-web" / "Dockerfile").read_text(encoding="utf-8")

    assert re.search(r"^FROM ros:noetic-ros-base-focal@sha256:[0-9a-f]{64}$", ros1, re.MULTILINE)
    assert "ros-noetic-rosbridge-suite" in ros1
    assert "ros-noetic-tf2-web-republisher" in ros1
    assert "ros-noetic-web-video-server" in ros1
    assert "patch_ros1_readonly.py" in ros1
    patch = (DEPLOY / "sidecars" / "patch_ros1_readonly.py").read_text(encoding="utf-8")
    assert "rosbridge_capabilities = [Subscribe, Defragment]" in patch
    assert "listenWS(factory, context_factory, interface=address)" in patch

    assert re.search(r"^FROM ros:humble-ros-base-jammy@sha256:[0-9a-f]{64}$", ros2, re.MULTILINE)
    assert "ros-humble-rosbridge-suite" in ros2
    assert "ros-humble-tf2-web-republisher" in ros2
    assert "ros-humble-web-video-server" in ros2
    ros2_runner = (DEPLOY / "sidecars" / "run-ros2-sidecar").read_text(encoding="utf-8")
    assert "topics_glob:=\"'[]'\"" in ros2_runner
    assert "topics_pub_glob:=\"'[]'\"" in ros2_runner
    assert "topics_sub_glob:=\"${MASAR_ROS2_TOPICS_GLOB" in ros2_runner


def test_supervisor_manifests_have_exact_ports_and_read_only_policy() -> None:
    ros1 = _manifest("ros1")
    ros2 = _manifest("ros2")

    assert ros1["distribution"] == "noetic"
    assert ros1["ports"] == {"rosbridge": "127.0.0.1:18111", "video": "127.0.0.1:18121"}
    assert ros1["browser_policy"] == {
        "publish": False,
        "call_service": False,
        "parameters": False,
    }
    assert set(ros1["programs"]) == {"rosbridge", "tf2_web_republisher", "web_video_server"}

    assert ros2["distribution"] == "humble"
    assert ros2["ports"] == {"rosbridge": "127.0.0.1:18112", "video": "127.0.0.1:18122"}
    assert ros2["browser_policy"] == {
        "topics_pub_glob": "[]",
        "services_glob": "[]",
        "params_glob": "[]",
    }
    assert set(ros2["programs"]) == {"rosbridge", "tf2_web_republisher", "web_video_server"}


def test_caddy_is_single_origin_and_preserves_websocket_upgrades() -> None:
    caddy = (DEPLOY / "caddy" / "Caddyfile").read_text(encoding="utf-8")

    assert "127.0.0.1:18090" in caddy
    for route in (
        "/api/*",
        "/ws/*",
        "/ros1/*",
        "/ros2/*",
        "/video/ros1/*",
        "/video/ros2/*",
        "/display/gazebo/*",
        "/display/rviz/*",
        "/webots/*",
    ):
        assert route in caddy

    assert "header_up Upgrade {http.request.header.Upgrade}" in caddy
    assert "header_up Connection {http.request.header.Connection}" in caddy
    assert "header_up Host 127.0.0.1:18111" in caddy
    assert "header_up Host 127.0.0.1:18112" in caddy
    assert "path_regexp traversal" in caddy
    assert "default_bind 127.0.0.1" in caddy
    assert "protocols h1 h2" in caddy
    assert "18088" not in caddy
    assert "18101" not in caddy
    assert "18102" not in caddy


def test_compose_configuration_is_valid() -> None:
    completed = subprocess.run(
        ["docker", "compose", "-f", str(DEPLOY / "compose.yaml"), "config", "--quiet"],
        cwd=ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr
