"""Restricted G checkpoints and a tensor-only TorchScript deployment API.

The scripted actor takes (ego, scan, neighbours, mask, state) and returns
(acceleration, next_state). Shapes are [...,5], [...,32], [...,K,8], [...,K]
(bool), and [...,64]. K may be zero. The caller clears memory on a new mission.
"""
import copy
from dataclasses import asdict
import json
import math
from pathlib import Path
from typing import Tuple
import torch
from torch import Tensor, nn
from .observation import SensorConfig
from .config import PolicyConfig
from .policy import LocalActor
from .artifacts import save_torch

SCHEMA = 'afor_local_g_v1'
VERSION = 1
CONTRACT = ('local compass frame, final goal delta, velocity, normalized metric lidar, '
            'one-hop peers; acceleration in [-1,1], velocity limit per axis 1 m/s')


def _deployment_actor(actor):
    with torch.random.fork_rng(devices=[]):
        deployed = LocalActor(actor.config, actor.sensors)
    deployed.load_state_dict({name: value.detach().cpu() for name, value in actor.state_dict().items()}, strict=True)
    return deployed.eval()


def _interface():
    return {'inputs': ['ego', 'scan', 'neighbours', 'mask', 'state'],
            'shapes': ['[...,5]', '[...,32]', '[...,K,8]', '[...,K]', '[...,64]'],
            'outputs': ['acceleration', 'next_state'], 'output_shapes': ['[...,2]', '[...,64]'],
            'reset': 'caller clears state before an episode', 'empty_neighbours': True,
            'mask_dtype': 'bool', 'contract': CONTRACT}


def actor_payload(model, dt):
    """Return an independent CPU actor-only payload without changing model/RNG."""
    if isinstance(dt, bool) or not isinstance(dt, (float, int)) or not math.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be a finite positive number')
    actor = model if isinstance(model, LocalActor) else model.actor
    deployed = _deployment_actor(actor)
    return {'schema': SCHEMA, 'version': VERSION, 'kind': 'actor', 'policy': asdict(actor.config),
            'sensors': asdict(actor.sensors), 'dt': float(dt),
            'state_dict': deployed.state_dict(), 'interface': _interface(), 'contract': CONTRACT}


def export_actor(model, path, dt):
    """Export only deployed actor weights; accept an actor or ActorCritic."""
    save_torch(path, actor_payload(model, dt))


def load_actor(path, device='cpu'):
    """Accept only the current G schema using the restricted weights-only loader."""
    checkpoint = torch.load(path, map_location=device, weights_only=True)
    if (not isinstance(checkpoint, dict) or checkpoint.get('schema') != SCHEMA
            or checkpoint.get('version') != VERSION or checkpoint.get('kind') != 'actor'):
        raise ValueError('Expected an afor_local_g_v1 actor checkpoint, version 1')
    if not isinstance(checkpoint.get('policy'), dict) or not isinstance(checkpoint.get('sensors'), dict):
        raise ValueError('Invalid actor configuration metadata')
    try:
        config = PolicyConfig(**checkpoint['policy'])
        sensors = SensorConfig(**checkpoint['sensors'])
    except (TypeError, ValueError) as error:
        raise ValueError('Invalid actor configuration metadata') from error
    dt = checkpoint.get('dt')
    if isinstance(dt, bool) or not isinstance(dt, (float, int)) or not math.isfinite(dt) or dt <= 0:
        raise ValueError('Invalid checkpoint timestep')
    if checkpoint.get('interface') != _interface() or checkpoint.get('contract') != CONTRACT:
        raise ValueError('Invalid deployment interface metadata')
    weights = checkpoint.get('state_dict')
    if not isinstance(weights, dict) or not all(isinstance(k, str) and isinstance(v, Tensor)
                                              and torch.isfinite(v).all() for k, v in weights.items()):
        raise ValueError('Invalid actor state dictionary')
    with torch.random.fork_rng(devices=[]):
        actor = LocalActor(config, sensors).to(device)
    try:
        actor.load_state_dict(weights, strict=True)
    except RuntimeError as error:
        raise ValueError('Actor weights do not match deployment architecture') from error
    return actor.eval(), float(dt)


class TensorActor(nn.Module):
    """Inference modules with no dependency on training/configuration objects."""
    def __init__(self, actor):
        super().__init__()
        for name in ('ego', 'compensation', 'graph', 'fuse', 'cell', 'mean'):
            setattr(self, name, copy.deepcopy(getattr(actor, name)))

    def forward(self, ego: Tensor, scan: Tensor, neighbours: Tensor, mask: Tensor,
                state: Tensor) -> Tuple[Tensor, Tensor]:
        own = self.ego(torch.cat((ego, scan), -1))
        own = own + .1 * self.compensation(own)
        pooled, _ = self.graph(own, state, neighbours, mask)
        count = mask.sum(-1, keepdim=True).clamp_max(10).to(own.dtype) / 10
        encoded = self.fuse(torch.cat((own, pooled, count), -1))
        hidden = self.cell(encoded.reshape(-1, 64), state.reshape(-1, 64)).reshape_as(state)
        return self.mean(hidden).tanh(), hidden


def export_scripted_actor(actor, path):
    """Script the dynamic-batch, dynamic-K tensor interface without tracing."""
    scripted = torch.jit.script(TensorActor(_deployment_actor(actor)).eval())
    metadata = {'schema': SCHEMA, 'version': VERSION, 'kind': 'scripted_actor',
                'policy': asdict(actor.config), 'sensors': asdict(actor.sensors), 'interface': _interface()}
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    try:
        torch.jit.save(scripted, str(temporary), _extra_files={'metadata.json': json.dumps(metadata)})
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)
    return scripted
