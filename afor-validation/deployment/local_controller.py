"""One robot's planar controller. Requires only NumPy, PyTorch and an export.

Vectors supplied in ENU world axes are rotated into the fixed mission axes.
The mission frame is chosen once, with +y along the trained corridor direction;
it is a deployment calibration, never a sensor-derived stage or waypoint.
"""
from dataclasses import dataclass
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
    role: int
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
    def __init__(self, export_directory, formation, role, mission_yaw_world=0.):
        root = Path(export_directory)
        self.contract = json.loads((root/'contract.json').read_text())
        if formation not in self.contract['formations'] or role not in range(5):
            raise ValueError('Choose line/circle/wedge and one fixed role in [0,4]')
        if not math.isfinite(mission_yaw_world):
            raise ValueError('Mission yaw must be finite')
        self.role, self.formation = role, formation
        self.mission_yaw_world = mission_yaw_world
        self.to_world = rotation(mission_yaw_world)
        self.template = np.array(self.contract['formations'][formation]['template_m'],dtype=np.float32)
        self.edges = {tuple(sorted(edge)) for edge in self.contract['formations'][formation]['edges']}
        self.dt = float(self.contract['dt_s'])
        self.model = torch.jit.load(str(root/formation/'actor.ts'), map_location='cpu').eval()
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
            if peer.role not in range(5) or peer.role == self.role or peer.role in seen:
                raise ValueError('Peer role must be unique, valid and different from ego')
            seen.add(peer.role)
            relative = vector2(peer.relative_position_world,'peer position')@self.to_world
            relative_velocity = vector2(peer.relative_velocity_world,'peer velocity')@self.to_world
            if not math.isfinite(peer.radius) or peer.radius<=0:
                raise ValueError('Invalid peer radius')
            if np.linalg.norm(relative)>=self.contract['neighbour_range_m']:
                continue
            slot = int(mask.sum())
            records[slot] = np.concatenate((relative/self.contract['neighbour_range_m'],
                relative_velocity,self.template[peer.role]-self.template[self.role],
                [float(tuple(sorted((self.role,peer.role))) in self.edges),peer.radius]))
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
                raise ValueError('Call at 20 Hz or reset after a pause; variable dt is not validated')
        packed = self.pack(final_goal_world,position_world,velocity_world,scan_mission,peers)
        action, memory = self.model(*packed,self.memory)
        acceleration = action[0].numpy().clip(-1,1)
        measured_velocity = vector2(velocity_world,'velocity')@self.to_world
        target = np.clip(measured_velocity+self.dt*acceleration,-1,1)
        self.memory = memory
        self.last_timestamp = timestamp
        return Command(acceleration@self.to_world.T,target@self.to_world.T)
