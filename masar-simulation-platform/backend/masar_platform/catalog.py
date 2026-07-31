"""Strict scenario catalog loader and static parameter validation."""

from __future__ import annotations

from collections.abc import Iterator, Mapping
from pathlib import Path
from types import MappingProxyType
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, Field, ValidationError

from .commands import COMMAND_REFS
from .contracts import ManifestValidationError, ScenarioManifest
from .profiles import ARTIFACT_PROFILES, DISPLAY_PROFILES, TOPIC_PROFILES


PUBLIC_SCENARIO_IDS = (
    "swarm.webots.ros2swarm",
    "control.gazebo.response-deform",
    "control.gazebo.response-controller",
    "exploration.gazebo.tepper",
    "exploration.webots.robosar",
    "rescue.gazebo.tepper-response",
)


class CatalogError(ValueError):
    """Raised for any schema, registration or reference error in the catalog."""


class _CatalogEntry(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    scenario_id: str = Field(min_length=1)
    manifest: str = Field(min_length=1)


class _CatalogDocument(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)

    schema_version: int
    scenarios: tuple[_CatalogEntry, ...] = Field(min_length=1)


class _UniqueKeyLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader: _UniqueKeyLoader, node: yaml.MappingNode, deep: bool = False) -> dict[Any, Any]:
    result: dict[Any, Any] = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise CatalogError(f"duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueKeyLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping)


class ScenarioCatalog:
    """An immutable catalog containing the six approved MVP scenarios."""

    def __init__(self, manifests: tuple[ScenarioManifest, ...]) -> None:
        ids = tuple(manifest.scenario_id for manifest in manifests)
        if ids != PUBLIC_SCENARIO_IDS:
            raise CatalogError(f"catalog must register exactly these scenarios in order: {PUBLIC_SCENARIO_IDS}")
        self._manifests = manifests
        self._by_id: Mapping[str, ScenarioManifest] = MappingProxyType(
            {manifest.scenario_id: manifest for manifest in manifests}
        )

    @classmethod
    def from_directory(cls, scenario_dir: str | Path) -> "ScenarioCatalog":
        directory = Path(scenario_dir)
        schema_path = directory / "schema.json"
        catalog_path = directory / "catalog.yaml"
        if not schema_path.is_file():
            raise CatalogError(f"manifest schema not found: {schema_path}")
        payload = _load_yaml(catalog_path)
        try:
            document = _CatalogDocument.model_validate(payload)
        except ValidationError as exc:
            raise CatalogError(str(exc)) from exc
        if document.schema_version != 1:
            raise CatalogError("unsupported catalog schema_version")
        ids = tuple(entry.scenario_id for entry in document.scenarios)
        if len(ids) != len(set(ids)):
            raise CatalogError("catalog scenario IDs must be unique")
        if ids != PUBLIC_SCENARIO_IDS:
            raise CatalogError(f"catalog must register exactly these scenarios in order: {PUBLIC_SCENARIO_IDS}")

        registered_files: list[str] = []
        manifests: list[ScenarioManifest] = []
        for entry in document.scenarios:
            relative = Path(entry.manifest)
            if relative.is_absolute() or ".." in relative.parts or relative.name != entry.manifest:
                raise CatalogError(f"invalid manifest path: {entry.manifest}")
            manifest_path = directory / relative
            manifest = cls.from_manifest(manifest_path)
            if manifest.scenario_id != entry.scenario_id:
                raise CatalogError(f"catalog ID does not match manifest: {entry.scenario_id}")
            registered_files.append(relative.name)
            manifests.append(manifest)

        actual_files = sorted(path.name for path in directory.glob("*.yaml") if path.name != "catalog.yaml")
        if actual_files != sorted(registered_files):
            raise CatalogError("scenario directory must contain exactly the six registered manifests")
        return cls(tuple(manifests))

    @classmethod
    def from_manifest(cls, manifest_path: str | Path) -> ScenarioManifest:
        payload = _load_yaml(Path(manifest_path))
        try:
            manifest = ScenarioManifest.model_validate(payload)
        except (ValidationError, ManifestValidationError, ValueError) as exc:
            raise CatalogError(str(exc)) from exc
        cls._validate_references(manifest)
        return manifest

    @staticmethod
    def _validate_references(manifest: ScenarioManifest) -> None:
        if manifest.data_profile not in TOPIC_PROFILES:
            raise CatalogError(f"unknown data_profile: {manifest.data_profile}")
        if manifest.display_profile not in DISPLAY_PROFILES:
            raise CatalogError(f"unknown display_profile: {manifest.display_profile}")
        if manifest.artifact_profile not in ARTIFACT_PROFILES:
            raise CatalogError(f"unknown artifact_profile: {manifest.artifact_profile}")
        artifact = ARTIFACT_PROFILES[manifest.artifact_profile]
        if artifact.data_profiles and manifest.data_profile not in artifact.data_profiles:
            raise CatalogError(
                f"artifact_profile {manifest.artifact_profile} does not allow data_profile {manifest.data_profile}"
            )
        for step in manifest.steps:
            try:
                command = COMMAND_REFS[step.command_ref]
            except KeyError as exc:
                raise CatalogError(f"unknown command_ref: {step.command_ref}") from exc
            if command.agent != step.agent:
                raise CatalogError(f"step {step.id} agent does not match {step.command_ref}")
            incompatible_probes = {
                probe.type
                for probe in step.readiness.all
                if probe.type in {"ros1_topic", "ros2_topic"} and probe.type != f"{step.agent}_topic"
            }
            if incompatible_probes:
                raise CatalogError(f"step {step.id} readiness probes do not match agent {step.agent}")

    @property
    def scenario_ids(self) -> tuple[str, ...]:
        return tuple(self._by_id)

    def get(self, scenario_id: str) -> ScenarioManifest:
        try:
            return self._by_id[scenario_id]
        except KeyError as exc:
            raise CatalogError(f"unknown scenario_id: {scenario_id}") from exc

    def validate_parameters(self, scenario_id: str, values: Mapping[str, Any] | None = None) -> dict[str, Any]:
        try:
            return self.get(scenario_id).resolve_parameters(values)
        except ManifestValidationError as exc:
            raise CatalogError(str(exc)) from exc

    def validate_manifest(self, manifest: ScenarioManifest) -> ScenarioManifest:
        try:
            validated = ScenarioManifest.model_validate(manifest.model_dump())
        except (ValidationError, ManifestValidationError, ValueError) as exc:
            raise CatalogError(str(exc)) from exc
        self._validate_references(validated)
        return validated

    def __iter__(self) -> Iterator[ScenarioManifest]:
        return iter(self._manifests)

    def __len__(self) -> int:
        return len(self._manifests)


def _load_yaml(path: Path) -> Any:
    if not path.is_file():
        raise CatalogError(f"YAML file not found: {path}")
    try:
        with path.open("r", encoding="utf-8") as stream:
            payload = yaml.load(stream, Loader=_UniqueKeyLoader)
    except (OSError, yaml.YAMLError, CatalogError) as exc:
        raise CatalogError(f"cannot load {path}: {exc}") from exc
    if not isinstance(payload, dict):
        raise CatalogError(f"{path} must contain a YAML mapping")
    return payload


def load_catalog(scenario_dir: str | Path | None = None) -> ScenarioCatalog:
    directory = Path(scenario_dir) if scenario_dir is not None else Path(__file__).resolve().parents[2] / "scenarios"
    return ScenarioCatalog.from_directory(directory)


__all__ = ["PUBLIC_SCENARIO_IDS", "CatalogError", "ScenarioCatalog", "load_catalog"]
