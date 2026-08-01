from pathlib import Path
import subprocess


WORKSPACE = Path(__file__).resolve().parents[3]
ROS1_SCRIPTS = (
    WORKSPACE / "masar_ws" / "launch_robosar_g.sh",
    WORKSPACE / "masar_ws" / "launch_robosar_w.sh",
    WORKSPACE
    / "masar_ws"
    / "src"
    / "robosar"
    / "robosar_emergency_response"
    / "scripts"
    / "run_tepper_response_demo.sh",
    WORKSPACE
    / "masar_ws"
    / "src"
    / "robosar"
    / "robosar_emergency_response"
    / "scripts"
    / "run_response_deform_demo.sh",
)


def test_platform_managed_scripts_are_valid_bash_and_have_an_ownership_mode() -> None:
    for script in ROS1_SCRIPTS:
        completed = subprocess.run(
            ["bash", "-n", str(script)],
            check=False,
            capture_output=True,
            text=True,
        )
        assert completed.returncode == 0, f"{script}: {completed.stderr}"
        assert "MASAR_PLATFORM_MODE" in script.read_text(encoding="utf-8")


def test_native_global_cleanup_is_bypassed_before_any_global_match_in_platform_mode() -> None:
    for name in ("launch_robosar_g.sh", "launch_robosar_w.sh"):
        source = (WORKSPACE / "masar_ws" / name).read_text(encoding="utf-8")
        cleanup = source.index("cleanup() {")
        platform_guard = source.index("if platform_mode_enabled; then", cleanup)
        global_cleanup = source.index('cleanup_gui_processes "INT"', cleanup)
        assert platform_guard < global_cleanup
        assert 'launcher=(setsid)' in source
        assert 'if ! platform_mode_enabled; then\n    launcher=(setsid)' in source


def test_platform_invocations_disable_original_qt_and_clean_master_mode() -> None:
    from masar_platform.catalog import ScenarioCatalog
    from masar_platform.commands import render_invocation

    catalog = ScenarioCatalog.from_directory(WORKSPACE / "masar-simulation-platform" / "scenarios")
    for scenario_id in (
        "exploration.gazebo.tepper",
        "exploration.webots.robosar",
        "rescue.gazebo.tepper-response",
    ):
        manifest = catalog.get(scenario_id)
        for step in manifest.steps:
            invocation = render_invocation(step.command_ref, {}, manifest=manifest)
            assert invocation.environment["MASAR_PLATFORM_MODE"] == "1"
    rescue = render_invocation(
        "ros1.tepper_response",
        {},
        manifest=catalog.get("rescue.gazebo.tepper-response"),
    )
    assert "--no-gui" in rescue.argv
    assert "--shared-master" in rescue.argv
