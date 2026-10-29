"""Deployment contract: measurements in a shared compass/odometry frame.

No map, absolute pose, team phase, peer goals, or peer hidden states enter the
policy. A goal displacement is a mission input, not a planned intermediate goal.
"""
from dataclasses import dataclass
import math
import torch


@dataclass(frozen=True)
class SensorConfig:
    lidar_rays: int = 32
    lidar_range: float = 2.0
    neighbour_range: float = 2.0

    def __post_init__(self):
        if (self.lidar_rays < 8 or min(self.lidar_range, self.neighbour_range) <= 0
                or not all(math.isfinite(x) for x in (self.lidar_range,self.neighbour_range))):
            raise ValueError('Use at least eight rays and positive sensing ranges')


def pack_observation(goal_delta, velocity, radius, scan, neighbours, mask,
                     sensors=SensorConfig()):
    """Pack one robot or any batch; scan is metric centre-to-surface range.

    neighbours[..., K, 8] = (relative position xy, relative velocity xy,
    nominal formation displacement xy, formation edge flag, peer radius).
    Only received neighbours may be supplied. Padding values are discarded.
    Ego and neighbour vectors must use the same axes as the ordered lidar rays.
    Ray zero points along +x, then increases counterclockwise over 2*pi.
    Radius can be scalar or have the leading dimensions of velocity.
    """
    if (goal_delta.shape != velocity.shape or velocity.shape[-1] != 2
            or scan.shape != velocity.shape[:-1] + (sensors.lidar_rays,)
            or neighbours.shape[:-2] != velocity.shape[:-1]
            or neighbours.shape[-1] != 8 or mask.shape != neighbours.shape[:-1]):
        raise ValueError('Invalid local sensor tensor shapes')
    if not all(torch.isfinite(x).all() for x in (goal_delta, velocity, scan)):
        raise ValueError('Nonfinite local measurements')
    radius = torch.as_tensor(radius, device=velocity.device, dtype=velocity.dtype)
    radius = torch.broadcast_to(radius, velocity.shape[:-1])
    if not torch.isfinite(radius).all() or (radius <= 0).any() or (scan < 0).any():
        raise ValueError('Radius must be positive and scan ranges nonnegative')
    # Retain direction for distant goals, with a bounded magnitude input.
    goal = goal_delta / goal_delta.norm(dim=-1, keepdim=True).clamp_min(1.)
    ego = torch.cat((goal, velocity, radius[..., None]), -1)
    mask = mask.bool()
    if not torch.isfinite(torch.where(mask[...,None],neighbours,0.)).all():
        raise ValueError('Nonfinite received neighbour measurements')
    # Enforce range even if the caller accidentally provides distant peers.
    mask = mask & (neighbours[..., :2].norm(dim=-1) < sensors.neighbour_range)
    peers = torch.where(mask[..., None], neighbours, 0.)
    if not torch.isfinite(peers).all():
        raise ValueError('Nonfinite visible neighbour measurements')
    peers = peers.clone()
    peers[..., :2] /= sensors.neighbour_range
    return {'ego': ego, 'scan': scan.clamp_max(sensors.lidar_range) / sensors.lidar_range,
            'neighbours': peers, 'mask': mask}


def lidar_scan(pos, radius, vertices, obstacle_mask, bounds, sensors):
    """Ideal finite-range ray sensor, including occluding disks and boundaries.

    vertices: [E,O,V,2], mask: [E,O], bounds: physical rectangular half extents.
    This function simulates a sensor; deployments pass their measured scan to
    pack_observation and need none of these geometry arguments.
    """
    angle = torch.arange(sensors.lidar_rays, device=pos.device) * (2*torch.pi/sensors.lidar_rays)
    direction = torch.stack((angle.cos(), angle.sin()), -1)
    # Ray/finite polygon-edge intersection.
    left = vertices[:, None, None]
    edge = torch.roll(left, -1, -2) - left
    relative = left - pos[:, :, None, None, None]
    ray = direction[None, None, :, None, None]
    cross = lambda a, b: a[..., 0]*b[..., 1]-a[..., 1]*b[..., 0]
    denominator = cross(ray, edge)
    safe = torch.where(denominator.abs() > 1e-8, denominator, 1.)
    time, fraction = cross(relative, edge)/safe, cross(relative, ray)/safe
    valid = ((denominator.abs() > 1e-8) & (time >= 0) & (fraction >= 0)
             & (fraction <= 1) & obstacle_mask[:, None, None, :, None])
    wall = time.masked_fill(~valid, float('inf')).flatten(-2).amin(-1)
    # Boundary faces. An exterior robot terminates; reset states are interior.
    safe_direction = torch.where(direction.abs() > 1e-8, direction, 1.)
    face = torch.where(direction >= 0, bounds, -bounds)
    times = (face[None, None] - pos[:, :, None]) / safe_direction
    times = times.masked_fill((direction.abs() <= 1e-8)[None, None], float('inf'))
    boundary = times.clamp_min(0).amin(-1)
    # Nearest ray hit on any other robot; no identity is encoded in the scan.
    delta = pos[:, None] - pos[:, :, None]
    projection = torch.einsum('eijd,rd->eirj', delta, direction)
    perpendicular = delta.square().sum(-1)[:, :, None] - projection.square()
    discriminant = radius**2-perpendicular
    hit = projection - discriminant.clamp_min(0).sqrt()
    diagonal = torch.eye(pos.shape[1], dtype=torch.bool, device=pos.device)[None, :, None]
    hit = hit.masked_fill((discriminant < 0) | (hit < 0) | diagonal, float('inf')).amin(-1)
    return torch.minimum(torch.minimum(wall, boundary), hit).clamp(0, sensors.lidar_range)
