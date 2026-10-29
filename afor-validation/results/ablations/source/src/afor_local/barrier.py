"""Training loss from local measured geometry and braking distance.

No safety critic, second rollout, map, projection, or deployment intervention.
Ray endpoints approximate static surfaces; tracked peers use relative motion.
This is a sampled auxiliary loss, not a hard safety certificate.
"""
import torch


def local_braking_loss(obs, acceleration, sensors, dt, margin=.03):
    velocity=obs['ego'][...,2:4]
    radius=obs['ego'][...,4:5]
    following_velocity=(velocity+dt*acceleration).clamp(-1,1)
    angle=torch.arange(sensors.lidar_rays,device=velocity.device,dtype=velocity.dtype)*(2*torch.pi/sensors.lidar_rays)
    direction=torch.stack((angle.cos(),angle.sin()),-1)
    points=(obs['scan']*sensors.lidar_range)[...,None]*direction
    peers=obs['neighbours']
    relative=peers[...,:2]*sensors.neighbour_range
    peer_radius=peers[...,7]
    visible=obs['mask']
    # A moving robot return must not simultaneously become a static wall.
    peer_hit=((points[..., :,None,:]-relative[...,None,:,:]).norm(dim=-1)
              <=peer_radius[...,None,:]+.015)&visible[...,None,:]
    static=(obs['scan']<1-1e-5)&~peer_hit.any(-1)

    def slack(displacement, relative_velocity, exclusion):
        distance=displacement.norm(dim=-1).clamp_min(1e-6)
        closing=torch.relu(-(displacement*relative_velocity).sum(-1)/distance)
        # Unit braking acceleration is conservative relative to per-axis 1.
        return distance-exclusion-margin-closing.square()/2

    static_now=slack(points,-velocity[...,None,:],radius)
    static_next=slack(points-dt*following_velocity[...,None,:],-following_velocity[...,None,:],radius)
    # A 1/s contraction rate gives an anticipatory distance/speed condition.
    static_residual=torch.relu(((1-dt)*static_now-static_next)/dt)
    static_loss=static_residual.masked_fill(~static,0).amax(-1)

    relative_velocity=peers[...,2:4]
    next_relative_velocity=relative_velocity-(following_velocity-velocity)[...,None,:]
    pair_now=slack(relative,relative_velocity,radius+peer_radius)
    pair_next=slack(relative+dt*next_relative_velocity,next_relative_velocity,radius+peer_radius)
    pair_residual=torch.relu(((1-dt)*pair_now-pair_next)/dt)
    # An empty neighbour set is valid at deployment and in unit diagnostics.
    pair_loss=(pair_residual.masked_fill(~visible,0).amax(-1) if relative.shape[-2]
               else torch.zeros_like(static_loss))
    return static_loss+pair_loss
