"""Strict, versioned contracts for scenario manifests.

The manifest is intentionally a small data contract.  It contains no shell
fragments; process launchers resolve command references through ``commands``.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, Literal

from pydantic import BaseModel, ConfigDict, Field, field_validator, model_validator


class ManifestValidationError(ValueError):
    """Raised when a scenario manifest or its resolved parameters is invalid."""


ParameterType = Literal["string", "integer", "number", "boolean"]


class ParameterDefinition(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    type: ParameterType
    default: Any
    enum: tuple[Any, ...] | None = None
    minimum: float | int | None = None
    maximum: float | int | None = None
    description: str | None = None

    @field_validator("enum", mode="before")
    @classmethod
    def _tuple_enum(cls, value: Any) -> Any:
        return tuple(value) if value is not None else None

    @model_validator(mode="after")
    def _validate_definition(self) -> "ParameterDefinition":
        if self.minimum is not None and self.maximum is not None and self.minimum > self.maximum:
            raise ValueError("minimum must not exceed maximum")
        if self.type == "string" and self.enum is None:
            raise ValueError("string parameters require an explicit enum allowlist")
        if self.type in {"integer", "number"} and (self.minimum is None or self.maximum is None):
            raise ValueError("numeric parameters require minimum and maximum")
        _validate_value(self.type, self.default, self.enum, self.minimum, self.maximum, "default")
        if self.enum is not None and not self.enum:
            raise ValueError("enum must not be empty")
        return self


class ReadinessProbe(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    type: Literal["ros1_topic", "ros2_topic", "http", "file"]
    name: str = Field(min_length=1)
    required: bool = True


class Readiness(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    all: tuple[ReadinessProbe, ...] = Field(min_length=1)


class GraceSeconds(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    sigint: Literal[10]
    sigterm: Literal[5]
    sigkill: Literal[5]


class ScenarioStep(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    id: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9_-]*$")
    agent: Literal["ros1", "ros2"]
    command_ref: str = Field(min_length=1)
    depends_on: tuple[str, ...] = ()
    readiness: Readiness
    start_timeout_seconds: int = Field(gt=0)
    grace_seconds: GraceSeconds


class ScenarioManifest(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    schema_version: Literal[1]
    scenario_id: str = Field(min_length=1, pattern=r"^[a-z][a-z0-9.-]*$")
    module: Literal["swarm", "control", "exploration", "rescue"]
    simulator: Literal["gazebo", "webots"]
    display_profile: str = Field(min_length=1)
    parameters: dict[str, ParameterDefinition]
    steps: tuple[ScenarioStep, ...] = Field(min_length=1)
    stop_order: tuple[str, ...] = Field(min_length=1)
    data_profile: str = Field(min_length=1)
    artifact_profile: str = Field(min_length=1)
    minimum_free_bytes: Literal[2_147_483_648]

    @model_validator(mode="after")
    def _validate_graph_and_compatibility(self) -> "ScenarioManifest":
        ids = [step.id for step in self.steps]
        if len(ids) != len(set(ids)):
            raise ValueError("step IDs must be unique")
        id_set = set(ids)
        for step in self.steps:
            if len(step.depends_on) != len(set(step.depends_on)):
                raise ValueError(f"step {step.id} contains duplicate dependencies")
            unknown = set(step.depends_on) - id_set
            if unknown:
                raise ValueError(f"step {step.id} depends on unknown step(s): {sorted(unknown)}")
            if step.id in step.depends_on:
                raise ValueError(f"step {step.id} cannot depend on itself")
        if len(self.stop_order) != len(set(self.stop_order)) or set(self.stop_order) != id_set:
            raise ValueError("stop_order must contain each step exactly once")
        stop_index = {step_id: index for index, step_id in enumerate(self.stop_order)}
        for step in self.steps:
            for dependency in step.depends_on:
                if stop_index[step.id] >= stop_index[dependency]:
                    raise ValueError("stop_order must reverse dependency order")
        _ensure_acyclic(self.steps)
        if self.module == "rescue" and self.simulator == "webots":
            raise ValueError("complete/rescue + webots is not supported")
        _validate_manifest_compatibility(self)
        return self

    @property
    def parameters_defaults(self) -> dict[str, Any]:
        return {name: definition.default for name, definition in self.parameters.items()}

    def resolve_parameters(self, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        values = {} if values is None else dict(values)
        unknown = set(values) - set(self.parameters)
        if unknown:
            raise ManifestValidationError(f"unknown parameter(s): {sorted(unknown)}")
        resolved = self.parameters_defaults
        resolved.update(values)
        for name, value in resolved.items():
            definition = self.parameters[name]
            _validate_value(
                definition.type,
                value,
                definition.enum,
                definition.minimum,
                definition.maximum,
                name,
            )
        _validate_resolved_compatibility(self, resolved)
        return resolved


def _validate_value(
    parameter_type: ParameterType,
    value: Any,
    enum: tuple[Any, ...] | None,
    minimum: float | int | None,
    maximum: float | int | None,
    field_name: str,
) -> None:
    valid_type = {
        "string": isinstance(value, str),
        "integer": isinstance(value, int) and not isinstance(value, bool),
        "number": isinstance(value, (int, float)) and not isinstance(value, bool),
        "boolean": isinstance(value, bool),
    }[parameter_type]
    if not valid_type:
        raise ManifestValidationError(f"{field_name} must be {parameter_type}")
    if enum is not None and value not in enum:
        raise ManifestValidationError(f"{field_name} must be one of {list(enum)}")
    if minimum is not None and value < minimum:
        raise ManifestValidationError(f"{field_name} must be >= {minimum}")
    if maximum is not None and value > maximum:
        raise ManifestValidationError(f"{field_name} must be <= {maximum}")


def _ensure_acyclic(steps: tuple[ScenarioStep, ...]) -> None:
    graph = {step.id: set(step.depends_on) for step in steps}
    visiting: set[str] = set()
    visited: set[str] = set()

    def visit(node: str) -> None:
        if node in visiting:
            raise ValueError("dependency cycle detected")
        if node in visited:
            return
        visiting.add(node)
        for dependency in graph[node]:
            visit(dependency)
        visiting.remove(node)
        visited.add(node)

    for node in graph:
        visit(node)


def _validate_manifest_compatibility(manifest: ScenarioManifest) -> None:
    names = set(manifest.parameters)
    if manifest.module == "swarm":
        required = {"pattern", "robot", "sensor_type", "number_robots", "webots_world"}
        missing = required - names
        if missing:
            raise ValueError(f"swarm manifest missing parameter definitions: {sorted(missing)}")
    if manifest.module == "control" and manifest.scenario_id.endswith("response-controller"):
        if "mode" not in names:
            raise ValueError("response-controller manifest must define mode")


def _validate_resolved_compatibility(manifest: ScenarioManifest, values: Mapping[str, Any]) -> None:
    robot = values.get("robot")
    sensor_type = values.get("sensor_type")
    pattern = str(values.get("pattern", ""))
    if robot == "burger" and sensor_type == "ir":
        raise ManifestValidationError("burger robot does not provide the ir sensor layer")
    if robot == "e_puck" and values.get("driving_swarm") is True:
        raise ManifestValidationError("driving_swarm is only supported for burger")
    object_shape = values.get("transport_object_shape")
    if object_shape not in (None, "cube") and "transport" not in pattern:
        raise ManifestValidationError("transport_object_shape requires a transport pattern")
    if values.get("total_robots", 0) < values.get("number_robots", 0):
        raise ManifestValidationError("total_robots must be >= number_robots")
    if "x_min" in values and values["x_min"] >= values["x_max"]:
        raise ManifestValidationError("x_min must be less than x_max")
    if "y_min" in values and values["y_min"] >= values["y_max"]:
        raise ManifestValidationError("y_min must be less than y_max")


__all__ = [
    "GraceSeconds",
    "ManifestValidationError",
    "ParameterDefinition",
    "Readiness",
    "ReadinessProbe",
    "ScenarioManifest",
    "ScenarioStep",
]
