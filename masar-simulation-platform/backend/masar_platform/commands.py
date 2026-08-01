"""Static command allowlist and shell-free argv rendering."""

from __future__ import annotations

import re
from dataclasses import dataclass
from functools import lru_cache
from types import MappingProxyType
from typing import TYPE_CHECKING, Any, Mapping

from .contracts import ManifestValidationError

if TYPE_CHECKING:
    from .contracts import ScenarioManifest


_SHELL_FRAGMENT = re.compile(r"(?:\$\(|`|[|;&<>\n\r])")
_ENVIRONMENT_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_LOCAL_NO_PROXY = "127.0.0.1,localhost,::1"
_PLATFORM_ENVIRONMENT = (
    ("MASAR_PLATFORM_MODE", "1"),
    ("NO_PROXY", _LOCAL_NO_PROXY),
    ("no_proxy", _LOCAL_NO_PROXY),
)


def _validate_environment(environment: tuple[tuple[str, str], ...]) -> None:
    names: set[str] = set()
    for name, value in environment:
        if not _ENVIRONMENT_NAME.fullmatch(name) or name in names:
            raise ValueError("fixed environment names must be unique portable identifiers")
        if not value or not value.isprintable() or _SHELL_FRAGMENT.search(value):
            raise ValueError(f"fixed environment value for {name} contains shell or control characters")
        names.add(name)


class CommandRenderError(ValueError):
    """Raised when a command ref or substituted parameter is not allowlisted."""


@dataclass(frozen=True, slots=True)
class CommandRef:
    agent: str
    executable: str
    fixed_args: tuple[str, ...] = ()
    launch_parameters: tuple[str, ...] = ()
    positional_parameters: tuple[str, ...] = ()
    fixed_args_after_parameters: bool = False
    fixed_environment: tuple[tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        environment = tuple((str(name), str(value)) for name, value in self.fixed_environment)
        _validate_environment(environment)
        object.__setattr__(self, "fixed_environment", environment)


@dataclass(frozen=True, slots=True)
class CommandInvocation:
    """A validated argv plus immutable process environment overrides."""

    argv: tuple[str, ...]
    environment: Mapping[str, str]

    def __post_init__(self) -> None:
        environment = tuple((str(name), str(value)) for name, value in self.environment.items())
        _validate_environment(environment)
        object.__setattr__(self, "environment", MappingProxyType(dict(environment)))


_SWARM_PARAMETERS = (
    "webots_world",
    "pattern",
    "number_robots",
    "total_robots",
    "log_level",
    "webots_mode",
    "webots_gui",
    "robot",
    "sensor_type",
    "x_min",
    "x_max",
    "y_min",
    "y_max",
    "min_distance",
    "z_height",
    "driving_swarm",
    "logging",
    "num_obstacles",
    "transport_object_shape",
)

COMMAND_REFS: Mapping[str, CommandRef] = MappingProxyType(
    {
        "ros2.swarm": CommandRef(
            agent="ros2",
            executable="ros2",
            fixed_args=(
                "launch",
                "launch_webots",
                "create_enviroment.launch.py",
                "webots_stream:=true",
                "webots_port:=18141",
            ),
            launch_parameters=_SWARM_PARAMETERS,
        ),
        "ros1.response_deform": CommandRef(
            agent="ros1",
            executable="./run_response_deform_demo.sh",
            fixed_args=("--no-rviz", "--no-vglrun", "--shared-master"),
        ),
        "ros1.response_controller": CommandRef(
            agent="ros1",
            executable="./run_response_controller_demo.sh",
            fixed_args=("--no-rviz", "--no-vglrun", "--shared-master"),
            positional_parameters=("mode",),
            fixed_args_after_parameters=True,
        ),
        "ros1.exploration_gazebo": CommandRef(
            agent="ros1",
            executable="./launch_robosar_g.sh",
            fixed_args=("--no-rviz", "--no-vglrun"),
        ),
        "ros2.exploration_webots": CommandRef(
            agent="ros2",
            executable="./start_robosar_webots.sh",
            fixed_args=("--no-vglrun",),
            fixed_environment=(("WEBOTS_STREAM", "true"), ("WEBOTS_PORT", "18141")),
        ),
        "ros1.exploration_webots": CommandRef(
            agent="ros1",
            executable="./launch_robosar_w.sh",
            fixed_args=("--no-rviz", "--no-vglrun"),
        ),
        "ros1.tepper_response": CommandRef(
            agent="ros1",
            executable="./run_tepper_response_demo.sh",
            fixed_args=("--no-rviz", "--no-vglrun", "--no-gui", "--shared-master"),
        ),
    }
)

_CONTROLLER_MODES = frozenset(
    {
        "roundflocking",
        "roundflocking-checkpoint",
        "roundflocking-heuristic",
        "transport",
        "transport-checkpoint",
        "transport-heuristic",
    }
)
def render_command(
    command_ref: str,
    parameters: Mapping[str, Any] | None = None,
    *,
    manifest: "ScenarioManifest | None" = None,
) -> list[str]:
    """Validate parameters and return argv without invoking a shell."""

    try:
        definition = COMMAND_REFS[command_ref]
    except KeyError as exc:
        raise CommandRenderError(f"unknown command_ref: {command_ref}") from exc

    supplied = {} if parameters is None else dict(parameters)
    if manifest is not None:
        if not any(step.command_ref == command_ref for step in manifest.steps):
            raise CommandRenderError(f"command_ref {command_ref} is not declared by {manifest.scenario_id}")
        try:
            resolved = manifest.resolve_parameters(supplied)
        except ManifestValidationError as exc:
            raise CommandRenderError(str(exc)) from exc
    else:
        default_manifest = _default_manifest_for_command(command_ref)
        if default_manifest is not None:
            try:
                resolved = default_manifest.resolve_parameters(supplied)
            except ManifestValidationError as exc:
                raise CommandRenderError(str(exc)) from exc
        else:
            resolved = _validate_without_manifest(command_ref, definition, supplied)

    argv = [definition.executable]
    if not definition.fixed_args_after_parameters:
        argv.extend(definition.fixed_args)
    for name in definition.positional_parameters:
        if name not in resolved:
            raise CommandRenderError(f"missing required parameter: {name}")
        argv.append(_safe_value(name, resolved[name]))
    for name in definition.launch_parameters:
        if name in resolved:
            argv.append(f"{name}:={_safe_value(name, resolved[name])}")
    if definition.fixed_args_after_parameters:
        argv.extend(definition.fixed_args)

    _validate_argv(argv)
    return argv


def command_environment(
    command_ref: str,
    *,
    manifest: "ScenarioManifest | None" = None,
) -> Mapping[str, str]:
    """Return immutable platform and ROS graph environment overrides."""

    try:
        definition = COMMAND_REFS[command_ref]
    except KeyError as exc:
        raise CommandRenderError(f"unknown command_ref: {command_ref}") from exc
    environment = dict(_PLATFORM_ENVIRONMENT)
    environment.update(dict(definition.fixed_environment))
    if manifest is not None:
        if not any(step.command_ref == command_ref for step in manifest.steps):
            raise CommandRenderError(f"command_ref {command_ref} is not declared by {manifest.scenario_id}")
        environment["ROS_MASTER_URI"] = manifest.runtime.ros1_master_uri
        environment["ROS_DOMAIN_ID"] = str(manifest.runtime.ros2_domain_id)
        if command_ref == "ros2.exploration_webots":
            environment["ROS1_MASTER_IP"] = "127.0.0.1"
    normalized = tuple(environment.items())
    _validate_environment(normalized)
    return MappingProxyType(dict(normalized))


def render_invocation(
    command_ref: str,
    parameters: Mapping[str, Any] | None = None,
    *,
    manifest: "ScenarioManifest | None" = None,
) -> CommandInvocation:
    """Render a complete immutable process invocation without executing it."""

    return CommandInvocation(
        argv=tuple(render_command(command_ref, parameters, manifest=manifest)),
        environment=command_environment(command_ref, manifest=manifest),
    )


def _validate_without_manifest(
    command_ref: str,
    definition: CommandRef,
    supplied: dict[str, Any],
) -> dict[str, Any]:
    accepted = set(definition.launch_parameters) | set(definition.positional_parameters)
    unknown = set(supplied) - accepted
    if unknown:
        raise CommandRenderError(f"unknown parameter(s): {sorted(unknown)}")
    if command_ref == "ros1.response_controller":
        mode = supplied.get("mode")
        if mode not in _CONTROLLER_MODES:
            raise CommandRenderError(f"mode must be one of {sorted(_CONTROLLER_MODES)}")
    return supplied


@lru_cache(maxsize=len(COMMAND_REFS))
def _default_manifest_for_command(command_ref: str) -> "ScenarioManifest | None":
    from .catalog import load_catalog

    matches = [
        manifest
        for manifest in load_catalog()
        if any(step.command_ref == command_ref for step in manifest.steps)
    ]
    return matches[0] if len(matches) == 1 else None


def _safe_value(name: str, value: Any) -> str:
    if isinstance(value, bool):
        if name in {"driving_swarm", "logging"}:
            rendered = "True" if value else "False"
        else:
            rendered = "true" if value else "false"
    elif isinstance(value, (str, int, float)) and not isinstance(value, complex):
        rendered = str(value)
    else:
        raise CommandRenderError(f"{name} has an unsupported value type")
    if not rendered or _SHELL_FRAGMENT.search(rendered):
        raise CommandRenderError(f"{name} contains forbidden shell characters")
    return rendered


def _validate_argv(argv: list[str]) -> None:
    forbidden = ("18088", "masar-unified-platform", "agent.py", "bash -c")
    for item in argv:
        if not item or _SHELL_FRAGMENT.search(item) or any(value in item for value in forbidden):
            raise CommandRenderError("rendered argv violates the static command policy")


__all__ = [
    "COMMAND_REFS",
    "CommandInvocation",
    "CommandRef",
    "CommandRenderError",
    "command_environment",
    "render_command",
    "render_invocation",
]
