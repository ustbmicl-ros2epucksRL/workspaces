from pathlib import Path
import subprocess

import yaml


ROOT = Path(__file__).resolve().parents[2]
LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"


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
