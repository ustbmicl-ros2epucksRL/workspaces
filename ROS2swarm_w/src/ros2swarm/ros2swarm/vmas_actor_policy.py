"""Load and run VMAS-trained actor checkpoints locally."""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np


@dataclass(frozen=True)
class LoadedActorInfo:
    """Small summary of a loaded actor artifact."""

    actor_path: str
    kind: str
    obs_dim: int
    action_dim: int
    generation: int | None
    fitness: float | None


class VmasActorPolicy:
    """Runtime wrapper for local ``best_actor.pt`` actor artifacts."""

    def __init__(
        self,
        actor_path: str,
        *,
        device: str = "cpu",
    ) -> None:
        self.actor_path = os.path.abspath(os.path.expanduser(actor_path))
        self.device_name = str(device)
        self._torch = None
        self._actor = None
        self.metadata: dict[str, Any] = {}
        self.info = self._load()

    def _load(self) -> LoadedActorInfo:
        if not os.path.exists(self.actor_path):
            raise FileNotFoundError(
                f"Actor checkpoint not found: {self.actor_path}"
            )

        try:
            import torch
        except Exception as exc:
            raise ImportError(
                "PyTorch is required to load best_actor.pt checkpoints."
            ) from exc

        self._torch = torch
        device = torch.device(self.device_name)
        try:
            payload = torch.load(
                self.actor_path,
                map_location=device,
                weights_only=False,
            )
        except TypeError:
            payload = torch.load(self.actor_path, map_location=device)

        metadata = dict(payload.get("metadata", {}))
        actor_spec_data = (
            metadata.get("actor_spec")
            or payload.get("actor_spec")
        )
        if actor_spec_data is None:
            raise ValueError(
                "Actor checkpoint has no actor_spec metadata: "
                f"{self.actor_path}"
            )

        actor_state = payload.get("actor") or payload.get("best_actor")
        if actor_state is None:
            raise ValueError(
                f"Actor checkpoint has no actor state dict: {self.actor_path}"
            )

        actor = _build_local_actor(actor_spec_data, actor_state, torch, device)
        actor.reset(1)

        self._actor = actor
        self.metadata = metadata
        return LoadedActorInfo(
            actor_path=self.actor_path,
            kind=str(actor_spec_data.get("kind", "")),
            obs_dim=int(actor_spec_data["obs_dim"]),
            action_dim=int(actor_spec_data["action_dim"]),
            generation=_optional_int(metadata.get("generation")),
            fitness=_optional_float(metadata.get("fitness")),
        )

    def reset(self, batch_size: int = 1) -> None:
        """Reset stateful recurrent actors."""
        if self._actor is not None:
            self._actor.reset(int(batch_size))

    def act(self, observation: np.ndarray | list[float]) -> np.ndarray:
        """Return a clipped normalized VMAS action in ``[-1, 1]``."""
        return self.act_batch(observation)[0]

    def act_batch(self, observations: np.ndarray | list[list[float]]) -> np.ndarray:
        """Return clipped normalized VMAS actions for a batch of observations."""
        if self._actor is None or self._torch is None:
            raise RuntimeError("Actor is not loaded.")

        obs = np.asarray(observations, dtype=np.float32)
        if obs.ndim == 1:
            obs = obs.reshape(1, -1)
        if obs.ndim != 2:
            raise ValueError(
                "Observation batch must be 1D or 2D, "
                f"got shape {obs.shape}."
            )
        if obs.shape[1] != self.info.obs_dim:
            raise ValueError(
                "Observation dim mismatch: "
                f"expected {self.info.obs_dim}, got {obs.shape[1]}"
            )

        torch = self._torch
        with torch.no_grad():
            tensor = torch.as_tensor(
                obs,
                dtype=torch.float32,
                device=self._actor.device,
            )
            action = self._actor(tensor).detach().cpu().numpy()
        return np.clip(action.astype(np.float32), -1.0, 1.0)


class _LocalNeatRecurrentActor:
    """Minimal runtime for the exported normalized NEAT recurrent actor."""

    def __init__(self, spec: dict[str, Any], state: dict[str, Any],
                 torch_module: Any, device: Any) -> None:
        metadata = dict(spec.get("metadata", {}))
        self.torch = torch_module
        self.device = device
        self.dtype = torch_module.float32
        self.n_inputs = int(metadata["n_inputs"])
        self.n_hidden = int(metadata["n_hidden"])
        self.n_outputs = int(metadata["n_outputs"])
        self.use_current_activs = bool(
            metadata.get("use_current_activs", False)
        )
        self.n_internal_steps = int(metadata.get("n_internal_steps", 1))
        self.hidden_activation_names = tuple(
            metadata.get("hidden_activation_names", [])
        )
        self.output_activation_names = tuple(
            metadata.get("output_activation_names", [])
        )

        self.input_to_hidden = self._tensor(
            state, "core.input_to_hidden", (self.n_hidden, self.n_inputs)
        )
        self.hidden_to_hidden = self._tensor(
            state, "core.hidden_to_hidden", (self.n_hidden, self.n_hidden)
        )
        self.output_to_hidden = self._tensor(
            state, "core.output_to_hidden", (self.n_hidden, self.n_outputs)
        )
        self.hidden_to_output = self._tensor(
            state, "core.hidden_to_output", (self.n_outputs, self.n_hidden)
        )
        self.input_to_output = self._tensor(
            state, "core.input_to_output", (self.n_outputs, self.n_inputs)
        )
        self.output_to_output = self._tensor(
            state, "core.output_to_output", (self.n_outputs, self.n_outputs)
        )
        self.hidden_responses = self._tensor(
            state, "core.hidden_responses", (self.n_hidden,)
        )
        self.hidden_biases = self._tensor(
            state, "core.hidden_biases", (self.n_hidden,)
        )
        self.output_responses = self._tensor(
            state, "core.output_responses", (self.n_outputs,)
        )
        self.output_biases = self._tensor(
            state, "core.output_biases", (self.n_outputs,)
        )

        self.input_to_hidden_mask = self._tensor(
            state, "core.input_to_hidden_mask", (self.n_hidden, self.n_inputs)
        )
        self.hidden_to_hidden_mask = self._tensor(
            state, "core.hidden_to_hidden_mask", (self.n_hidden, self.n_hidden)
        )
        hidden_output_shape = (self.n_hidden, self.n_outputs)
        output_hidden_shape = (self.n_outputs, self.n_hidden)
        self.output_to_hidden_mask = self._tensor(
            state, "core.output_to_hidden_mask", hidden_output_shape
        )
        self.hidden_to_output_mask = self._tensor(
            state, "core.hidden_to_output_mask", output_hidden_shape
        )
        self.input_to_output_mask = self._tensor(
            state, "core.input_to_output_mask", (self.n_outputs, self.n_inputs)
        )
        self.output_to_output_mask = self._tensor(
            state,
            "core.output_to_output_mask",
            (self.n_outputs, self.n_outputs),
        )
        self.activs = None
        self.outputs = None
        self.batch_size = 0

    def _tensor(self, state: dict[str, Any], name: str,
                fallback_shape: tuple[int, ...]):
        torch = self.torch
        value = state.get(name)
        if value is None:
            return torch.zeros(
                fallback_shape,
                dtype=self.dtype,
                device=self.device,
            )
        return value.to(dtype=self.dtype, device=self.device)

    def reset(self, batch_size: int = 1) -> None:
        """Reset recurrent hidden and output state."""
        self.batch_size = int(batch_size)
        self.activs = self.torch.zeros(
            self.batch_size,
            self.n_hidden,
            dtype=self.dtype,
            device=self.device,
        )
        self.outputs = self.torch.zeros(
            self.batch_size,
            self.n_outputs,
            dtype=self.dtype,
            device=self.device,
        )

    def __call__(self, obs):
        return self.forward(obs)

    def forward(self, obs):
        """Run recurrent NEAT and normalize outputs to ``[-1, 1]``."""
        state_device_changed = self.outputs.device != obs.device
        if obs.shape[0] != self.batch_size or state_device_changed:
            self.device = obs.device
            self.reset(obs.shape[0])

        activs_for_output = self.activs
        if self.n_hidden > 0:
            for _ in range(self.n_internal_steps):
                hidden_inputs = (
                    obs.matmul(
                        (
                            self.input_to_hidden
                            * self.input_to_hidden_mask
                        ).t()
                    )
                    + self.activs.matmul(
                        (
                            self.hidden_to_hidden
                            * self.hidden_to_hidden_mask
                        ).t()
                    )
                    + self.outputs.matmul(
                        (
                            self.output_to_hidden
                            * self.output_to_hidden_mask
                        ).t()
                    )
                )
                hidden_pre = (
                    self.hidden_responses * hidden_inputs
                    + self.hidden_biases
                )
                self.activs = _apply_activation_names(
                    hidden_pre,
                    self.hidden_activation_names,
                    self.torch,
                )
            if self.use_current_activs:
                activs_for_output = self.activs

        input_to_output = self.input_to_output * self.input_to_output_mask
        output_to_output = self.output_to_output * self.output_to_output_mask
        output_inputs = (
            obs.matmul(input_to_output.t())
            + self.outputs.matmul(
                output_to_output.t()
            )
        )
        if self.n_hidden > 0:
            output_inputs = output_inputs + activs_for_output.matmul(
                (self.hidden_to_output * self.hidden_to_output_mask).t()
            )

        output_pre = self.output_responses * output_inputs + self.output_biases
        self.outputs = _apply_activation_names(
            output_pre,
            self.output_activation_names,
            self.torch,
        )
        normalized_unit = _normalize_output_names(
            self.outputs,
            self.output_activation_names,
            self.torch,
        )
        return (normalized_unit * 2.0 - 1.0).clamp(-1.0, 1.0)


def _build_local_actor(
    spec: dict[str, Any],
    state: dict[str, Any],
    torch_module: Any,
    device: Any,
) -> _LocalNeatRecurrentActor:
    kind = str(spec.get("kind", ""))
    if kind != "neat_recurrent":
        raise ValueError(
            "This runtime currently supports neat_recurrent actors only; "
            f"got {kind!r}."
        )
    return _LocalNeatRecurrentActor(spec, state, torch_module, device)


def _apply_activation_names(values, names: Iterable[str], torch_module: Any):
    if values.shape[-1] == 0:
        return values
    names = tuple(names)
    if len(names) != values.shape[-1]:
        raise ValueError(
            f"Activation count {len(names)} does not match tensor width "
            f"{values.shape[-1]}."
        )

    activated = torch_module.zeros_like(values)
    for index, name in enumerate(names):
        activated[..., index] = _activate(
            values[..., index],
            name,
            torch_module,
        )
    return activated


def _activate(value, name: str, torch_module: Any):
    normalized = str(name).lower()
    if normalized == "sigmoid":
        return torch_module.sigmoid(
            torch_module.clamp(
                torch_module.nan_to_num(5.0 * value, nan=0.0),
                min=-60.0,
                max=60.0,
            )
        )
    if normalized == "tanh":
        return torch_module.tanh(
            torch_module.clamp(
                torch_module.nan_to_num(2.5 * value, nan=0.0),
                min=-60.0,
                max=60.0,
            )
        )
    if normalized == "identity":
        return torch_module.clamp(
            torch_module.nan_to_num(value, nan=0.0),
            min=-1e6,
            max=1e6,
        )
    if normalized == "relu":
        return torch_module.relu(torch_module.nan_to_num(value, nan=0.0))
    if normalized == "abs":
        return torch_module.abs(torch_module.nan_to_num(value, nan=0.0))
    if normalized == "sin":
        return torch_module.sin(
            torch_module.clamp(
                torch_module.nan_to_num(5.0 * value, nan=0.0),
                min=-60.0,
                max=60.0,
            )
        )
    if normalized == "gauss":
        x = torch_module.clamp(
            torch_module.nan_to_num(5.0 * value, nan=0.0),
            min=-3.4,
            max=3.4,
        )
        return torch_module.exp(-5.0 * x * x)
    raise ValueError(f"Unsupported NEAT activation: {name!r}")


def _normalize_output_names(values, names: Iterable[str], torch_module: Any):
    normalized = torch_module.zeros_like(values)
    for index, name in enumerate(tuple(names)):
        normalized[..., index] = _normalize_activation_output(
            values[..., index],
            name,
            torch_module,
        )
    return normalized


def _normalize_activation_output(value, name: str, torch_module: Any):
    mode = {
        "sigmoid": "unit",
        "gauss": "unit",
        "tanh": "signed",
        "sin": "signed",
        "identity": "unbounded",
        "relu": "nonnegative",
        "abs": "nonnegative",
    }.get(str(name).lower())
    if mode is None:
        raise ValueError(f"Unsupported output activation: {name!r}")

    value = torch_module.clamp(
        torch_module.nan_to_num(value, nan=0.0),
        min=-1e6,
        max=1e6,
    )
    if mode == "unit":
        return value.clamp(0.0, 1.0)
    if mode == "signed":
        return (value.clamp(-1.0, 1.0) + 1.0) * 0.5
    if mode == "nonnegative":
        return 1.0 - torch_module.exp(-value.clamp_min(0.0))
    if mode == "unbounded":
        return (torch_module.tanh(value) + 1.0) * 0.5
    raise ValueError(f"Unknown activation output mode: {mode}")


def _optional_int(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _optional_float(value: Any) -> float | None:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None
