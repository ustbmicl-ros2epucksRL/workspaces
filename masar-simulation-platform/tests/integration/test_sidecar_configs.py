import configparser
from pathlib import Path
import re
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEPLOY = ROOT / "deploy"
LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"


def _compose() -> dict:
    return yaml.safe_load((DEPLOY / "compose.yaml").read_text(encoding="utf-8"))


def _supervisor(name: str) -> configparser.ConfigParser:
    parser = configparser.ConfigParser()
    parser.read(DEPLOY / "sidecars" / f"{name}-supervisord.conf", encoding="utf-8")
    return parser


def test_compose_defines_loopback_only_web_stack() -> None:
    services = _compose()["services"]

    assert {"api", "ros1-web", "ros2-web", "webots-guard", "caddy"} <= services.keys()
    for name in ("api", "ros1-web", "ros2-web", "webots-guard", "caddy"):
        service = services[name]
        assert service["network_mode"] == "host", name
        assert service["environment"]["NO_PROXY"] == LOCAL_NO_PROXY, name
        assert service["environment"]["no_proxy"] == LOCAL_NO_PROXY, name
        assert "ports" not in service, name

    assert services["ros1-web"]["environment"]["ROS_MASTER_URI"] == "http://127.0.0.1:11311"
    assert services["ros2-web"]["environment"]["ROS_DOMAIN_ID"] == "${MASAR_ROS2_DOMAIN_ID:-42}"
    assert services["ros2-web"]["environment"]["ROS_LOCALHOST_ONLY"] == "${MASAR_ROS2_LOCALHOST_ONLY:-0}"
    assert services["ros2-web"]["ipc"] == "host"
    assert "../../masar_ws:/root/workspace/masar_ws:ro" in services["ros1-web"]["volumes"]
    assert "../../ROS2swarm_w:/root/workspace/ROS2swarm_w:ro" in services["ros2-web"]["volumes"]
    assert services["caddy"]["image"].startswith("caddy:2.10.2-alpine@sha256:")
    assert "./caddy/Caddyfile:/etc/caddy/Caddyfile:ro" in services["caddy"]["volumes"]
    assert "../frontend/dist:/srv/frontend:ro" in services["caddy"]["volumes"]
    assert "build" not in services["caddy"]
    assert services["webots-guard"]["cap_add"] == ["NET_ADMIN"]
    assert services["webots-guard"]["security_opt"] == ["no-new-privileges:true"]
    assert services["webots-guard"]["read_only"] is True
    assert services["webots-guard"]["entrypoint"] == ["/usr/local/bin/webots-port-guard"]
    assert "./webots-guard/webots-port-guard:/usr/local/bin/webots-port-guard:ro" in services[
        "webots-guard"
    ]["volumes"]
    assert services["webots-guard"]["environment"]["MASAR_WEBOTS_STREAM_PORT"] == "18141"
    assert services["caddy"]["depends_on"]["webots-guard"]["condition"] == "service_healthy"


def test_sidecar_images_pin_ros_distro_and_packages() -> None:
    ros1 = (DEPLOY / "ros1-web" / "Dockerfile").read_text(encoding="utf-8")
    ros2 = (DEPLOY / "ros2-web" / "Dockerfile").read_text(encoding="utf-8")

    assert re.search(r"^FROM ros:noetic-ros-base-focal@sha256:[0-9a-f]{64}$", ros1, re.MULTILINE)
    assert "ros-noetic-rosbridge-suite" in ros1
    assert "ros-noetic-tf2-web-republisher" in ros1
    assert "ros-noetic-web-video-server" in ros1
    assert "patch_ros1_readonly.py" in ros1
    patch = (DEPLOY / "sidecars" / "patch_ros1_readonly.py").read_text(encoding="utf-8")
    assert "rosbridge_capabilities = [CallService, Subscribe, Defragment]" in patch
    assert "listenWS(factory, context_factory, interface=address)" in patch
    ros1_runner = (DEPLOY / "sidecars" / "run-ros1-sidecar").read_text(encoding="utf-8")
    assert "source /root/workspace/masar_ws/devel/setup.bash" in ros1_runner
    assert "masar_ws devel overlay is unavailable" in ros1_runner
    assert "roslaunch rosbridge_server rosbridge_websocket.launch" in ros1_runner
    assert "services_glob:=\"[rosapi/topics,/rosapi/topics]\"" in ros1_runner

    assert re.search(r"^FROM ros:humble-ros-base-jammy@sha256:[0-9a-f]{64}$", ros2, re.MULTILINE)
    assert "ros-humble-rosbridge-suite" in ros2
    assert "ros-humble-tf2-web-republisher" in ros2
    assert "ros-humble-web-video-server" in ros2
    ros2_runner = (DEPLOY / "sidecars" / "run-ros2-sidecar").read_text(encoding="utf-8")
    assert "source /root/workspace/ROS2swarm_w/install/setup.bash" in ros2_runner
    assert "topics_glob:=\"'[]'\"" in ros2_runner
    assert "topics_pub_glob:=\"'[]'\"" in ros2_runner
    assert "topics_sub_glob:=\"${MASAR_ROS2_TOPICS_GLOB" in ros2_runner
    assert "services_glob:=\"'[rosapi/topics,/rosapi/topics]'\"" in ros2_runner
    assert "call_services_in_new_thread:=true" in ros2_runner


def test_supervisor_configs_run_exactly_the_three_sidecar_programs() -> None:
    expected_programs = {"rosbridge", "tf2_web_republisher", "web_video_server"}
    for distribution in ("ros1", "ros2"):
        supervisor = _supervisor(distribution)
        program_sections = {section for section in supervisor if section.startswith("program:")}
        assert program_sections == {f"program:{program}" for program in expected_programs}
        for program in expected_programs:
            settings = supervisor[f"program:{program}"]
            assert settings["command"] == f"/usr/local/bin/run-{distribution}-sidecar {program}"
            assert settings.getboolean("stopasgroup") is True
            assert settings.getboolean("killasgroup") is True


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
        "/display/webots/*",
    ):
        assert route in caddy

    assert "header_up Upgrade {http.request.header.Upgrade}" in caddy
    assert "header_up Connection {http.request.header.Connection}" in caddy
    assert "header_up Host 127.0.0.1:18111" in caddy
    assert "header_up Host 127.0.0.1:18112" in caddy
    assert "path_regexp traversal" in caddy
    assert "default_bind 127.0.0.1" in caddy
    assert "protocols h1 h2" in caddy
    assert "root * /srv/frontend" in caddy
    assert "try_files {path} /index.html" in caddy
    assert "file_server" in caddy
    assert "18088" not in caddy
    assert "reverse_proxy 127.0.0.1:18141" not in caddy
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
