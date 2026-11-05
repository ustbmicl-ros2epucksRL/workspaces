"""One robot's planar controller. Requires only NumPy, PyTorch and an export.

Vectors supplied in ENU world axes are rotated into the fixed mission axes.
The mission frame is chosen once, with +y along the trained corridor direction;
it is a deployment calibration, never a sensor-derived stage or waypoint.
"""
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Sequence
import numpy as np
import torch


def rotation(yaw):
    return np.array(((math.cos(yaw), -math.sin(yaw)),
                     (math.sin(yaw), math.cos(yaw))), dtype=np.float32)


def vector2(value, name):
    result = np.asarray(value, dtype=np.float32)
    if result.shape != (2,) or not np.isfinite(result).all():
        raise ValueError(f'{name} must be a finite 2D vector')
    return result


@dataclass(frozen=True)
class PeerMeasurement:
    slot_id: int
    # Relative position and relative velocity, in WORLD axes, from local
    # sensing/one-hop reception. A peer's final goal is never supplied.
    relative_position_world: Sequence[float]
    relative_velocity_world: Sequence[float]
    radius: float = .13


@dataclass(frozen=True)
class Command:
    acceleration_world: np.ndarray
    velocity_target_world: np.ndarray


def sample_lidar(ranges, angle_min, angle_increment, body_yaw_world,
                 mission_yaw_world=0., max_range=2.):
    """Convert a full circular body-frame LaserScan to 32 mission-frame rays.

    For exact ray alignment, return that beam. Otherwise take the minimum of
    the two surrounding beams (never linear interpolation). Arbitrary heading
    requires at most one-degree angular spacing. This resampling is an adapter,
    not evidence of robustness to a real scanner's missing/thin objects.
    Infinity means a valid no-return, NaN/negative ranges are rejected.
    Angles use radians, increasing counterclockwise from body +x.
    """
    values = np.asarray(ranges, dtype=np.float64)
    scalars = (angle_min, angle_increment, body_yaw_world, mission_yaw_world, max_range)
    if values.ndim != 1 or len(values)<32 or not all(math.isfinite(v) for v in scalars):
        raise ValueError('Expected a finite circular scan configuration')
    if angle_increment<=0 or max_range<=0 or np.isnan(values).any() or (values<0).any():
        raise ValueError('Invalid lidar ranges or increments')
    turn = 2*math.pi
    if not math.isclose(len(values)*angle_increment, turn, abs_tol=1e-5):
        raise ValueError('Provide one full circle without a duplicate endpoint')
    values = np.minimum(values, max_range)
    desired = np.arange(32)*turn/32 + mission_yaw_world-body_yaw_world
    index = np.mod((desired-angle_min)/angle_increment, len(values))
    nearest = np.rint(index)
    aligned = np.abs(index-nearest)<1e-5
    if not aligned.all() and angle_increment > math.pi/180+1e-8:
        raise ValueError('Use aligned 32 rays or a full scanner with <=1 degree spacing')
    lower = np.floor(index).astype(int)%len(values)
    upper = (lower+1)%len(values)
    sampled = np.minimum(values[lower], values[upper])
    sampled[aligned] = values[nearest[aligned].astype(int)%len(values)]
    return sampled.astype(np.float32)


class LocalController:
    def __init__(self, export_directory, formation, slot_id, mission_yaw_world=0.):
        root = Path(export_directory)
        self.contract = json.loads((root/'contract.json').read_text())
        if (self.contract.get('schema') != 'afor_local_g_v1' or self.contract.get('version') != 1
                or self.contract.get('kind') != 'deployment_bundle'):
            raise ValueError('Expected a G deployment bundle, version 1')
        if (formation not in ('line', 'circle', 'wedge') or formation not in self.contract['formations']
                or not isinstance(slot_id, (int, np.integer)) or isinstance(slot_id, bool)
                or slot_id not in range(5)):
            raise ValueError('Choose an exported formation and one fixed slot_id in [0,4]')
        if not math.isfinite(mission_yaw_world):
            raise ValueError('Mission yaw must be finite')
        if self.contract.get('agents') != 5 or self.contract.get('memory_size') != 64 or self.contract.get('lidar_rays') != 32:
            raise ValueError('Invalid G agent, memory, or lidar dimensions')
        for key in ('dt_s', 'radius_m', 'lidar_range_m', 'neighbour_range_m'):
            value = self.contract.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'Invalid deployment contract field {key}')
        self.slot_id, self.formation = int(slot_id), formation
        self.mission_yaw_world = mission_yaw_world
        self.to_world = rotation(mission_yaw_world)
        self.template = np.array(self.contract['formations'][formation]['template_m'],dtype=np.float32)
        if self.template.shape != (5, 2) or not np.isfinite(self.template).all():
            raise ValueError('Invalid formation template')
        self.edges = {tuple(sorted(edge)) for edge in self.contract['formations'][formation]['edges']}
        if any(len(edge) != 2 or edge[0] not in range(5) or edge[1] not in range(5)
               or edge[0] == edge[1] for edge in self.edges):
            raise ValueError('Invalid formation edges')
        self.dt = float(self.contract['dt_s'])
        entries = [entry for entry in self.contract['exports'] if entry['formation'] == formation]
        if len(entries) != 1:
            raise ValueError('Expected one exported actor per formation')
        entry = entries[0]
        model_path = root / formation / 'actor.ts'
        if hashlib.sha256(model_path.read_bytes()).hexdigest() != entry['actor_ts_sha256']:
            raise ValueError('Scripted actor hash differs from the bundle contract')
        extra = {'metadata.json': ''}
        self.model = torch.jit.load(str(model_path), map_location='cpu', _extra_files=extra).eval()
        metadata = json.loads(extra['metadata.json'])
        expected_inputs = ['ego', 'scan', 'neighbours', 'mask', 'state']
        sensors = {'lidar_rays': 32, 'lidar_range': self.contract['lidar_range_m'],
                   'neighbour_range': self.contract['neighbour_range_m']}
        if (metadata.get('schema') != self.contract['schema'] or metadata.get('version') != 1
                or metadata.get('kind') != 'scripted_actor'
                or metadata.get('policy') != entry.get('policy') or metadata.get('sensors') != sensors
                or metadata.get('policy') != {'hidden': 64, 'capacity_width': 24}
                or metadata.get('interface', {}).get('inputs') != expected_inputs):
            raise ValueError('Scripted actor metadata differs from the G bundle contract')
        self.reset()

    def reset(self):
        """Reset on a new mission/episode or after restarting a stopped robot."""
        self.memory = torch.zeros(1, self.contract['memory_size'], dtype=torch.float32)
        self.last_timestamp = None

    def pack(self, final_goal_world, position_world, velocity_world, scan_mission, peers=()):
        goal_delta = (vector2(final_goal_world,'goal')-vector2(position_world,'position'))@self.to_world
        velocity = vector2(velocity_world,'velocity')@self.to_world
        scan = np.asarray(scan_mission,dtype=np.float32)
        if scan.shape != (32,) or not np.isfinite(scan).all() or (scan<0).any():
            raise ValueError('Provide 32 finite metric ranges aligned to the mission frame')
        ego = np.concatenate((goal_delta/max(float(np.linalg.norm(goal_delta)),1.),
                              velocity, [self.contract['radius_m']])).astype(np.float32)
        records = np.zeros((4,8),dtype=np.float32)
        mask = np.zeros(4,dtype=bool)
        seen = set()
        for peer in peers:
            if (not isinstance(peer.slot_id, (int, np.integer)) or isinstance(peer.slot_id, bool)
                    or peer.slot_id not in range(5) or peer.slot_id == self.slot_id or peer.slot_id in seen):
                raise ValueError('Peer slot_id must be unique, valid and different from ego')
            seen.add(peer.slot_id)
            relative = vector2(peer.relative_position_world,'peer position')@self.to_world
            relative_velocity = vector2(peer.relative_velocity_world,'peer velocity')@self.to_world
            if not math.isfinite(peer.radius) or peer.radius<=0:
                raise ValueError('Invalid peer radius')
            if np.linalg.norm(relative)>=self.contract['neighbour_range_m']:
                continue
            slot = int(mask.sum())
            records[slot] = np.concatenate((relative/self.contract['neighbour_range_m'],
                relative_velocity,self.template[peer.slot_id]-self.template[self.slot_id],
                [float(tuple(sorted((self.slot_id,peer.slot_id))) in self.edges),peer.radius]))
            mask[slot] = True
        return tuple(torch.from_numpy(x[None]) for x in
                     (ego, np.minimum(scan,self.contract['lidar_range_m'])/self.contract['lidar_range_m'], records, mask))

    @torch.inference_mode()
    def step(self, final_goal_world, position_world, velocity_world, scan_mission,
             peers=(), timestamp=None):
        # Variable dt integration would silently change the trained plant.
        if timestamp is not None:
            if not math.isfinite(timestamp):
                raise ValueError('Nonfinite timestamp')
            if self.last_timestamp is not None and not math.isclose(
                    timestamp-self.last_timestamp,self.dt,rel_tol=0,abs_tol=1e-4):
                raise ValueError('Call at the exported timestep or reset after a pause')
        packed = self.pack(final_goal_world,position_world,velocity_world,scan_mission,peers)
        arguments = [*packed, self.memory]
        action, memory = self.model(*arguments)
        if (action.shape != (1, 2) or memory.shape != (1, 64)
                or not torch.isfinite(action).all() or not torch.isfinite(memory).all()):
            raise ValueError('Invalid scripted actor output')
        acceleration = action[0].numpy().clip(-1,1)
        measured_velocity = vector2(velocity_world,'velocity')@self.to_world
        target = np.clip(measured_velocity+self.dt*acceleration,-1,1)
        self.memory = memory
        self.last_timestamp = timestamp
        return Command(acceleration@self.to_world.T,target@self.to_world.T)
