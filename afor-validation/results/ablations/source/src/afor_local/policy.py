"""One-hop local GRU actor and a separate recurrent task critic."""
from dataclasses import dataclass, asdict
import math
import torch
from torch import nn
from torch.nn import functional as F
from torch.distributions import Normal
from .observation import SensorConfig

SCHEMA = 'afor_local_v1'


@dataclass(frozen=True)
class PolicyConfig:
    hidden: int = 64
    memory: bool = True
    neighbours: bool = True

    def __post_init__(self):
        # Keep export-schema fields while accepting only the current recipe.
        if not self.memory or not self.neighbours:
            raise ValueError('The current policy requires local memory and neighbours')


class LocalFeatures(nn.Module):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig()):
        super().__init__()
        self.config, self.sensors = config, sensors
        h = config.hidden
        if h < 8:
            raise ValueError('Hidden dimension must be at least eight')
        self.peer = nn.Sequential(nn.Linear(8,h//2), nn.Tanh(), nn.Linear(h//2,h//2), nn.Tanh())
        self.encoder = nn.Sequential(nn.Linear(5+sensors.lidar_rays+h//2+1,h),nn.Tanh(),nn.Linear(h,h),nn.Tanh())
        self.cell = nn.GRUCell(h,h)
    def initial_state(self, batch_shape=()):
        return self.encoder[0].weight.new_zeros(tuple(batch_shape)+(self.config.hidden,))

    def features(self, obs, state, reset=None):
        shape = obs['ego'].shape[:-1]
        mask = obs['mask']
        # Mask before encoding, so even NaN padding cannot affect a robot.
        peer = self.peer(torch.where(mask[...,None],obs['neighbours'],0.))
        count = mask.sum(-1,keepdim=True)
        aggregate = (peer*mask[...,None]).sum(-2)/count.clamp_min(1)
        encoded = self.encoder(torch.cat((obs['ego'],obs['scan'],aggregate,count.clamp_max(10)/10),-1))
        if reset is not None:
            state = torch.where(reset[...,None],0.,state)
        output = self.cell(encoded.reshape(-1,self.config.hidden),state.reshape(-1,self.config.hidden)).reshape(*shape,self.config.hidden)
        return output, output


class LocalActor(LocalFeatures):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig()):
        super().__init__(config,sensors)
        self.mean = nn.Linear(config.hidden,2)
        self.log_std = nn.Parameter(torch.full((2,),-.7))
        nn.init.orthogonal_(self.mean.weight,.01)
        nn.init.zeros_(self.mean.bias)

    def forward(self, obs, state, reset=None):
        """Deterministic normalized acceleration and next local memory."""
        features, state = self.features(obs,state,reset)
        return self.mean(features).tanh(), state

    def distribution(self, features):
        return Normal(self.mean(features),self.log_std.clamp(-4,1).exp())


def latent_log_prob(distribution, latent):
    # Store Gaussian latent in the rollout. No atanh or clipped tail surrogate.
    jacobian = 2*(math.log(2)-latent-F.softplus(-2*latent))
    return (distribution.log_prob(latent)-jacobian).sum(-1)


class ActorCritic(nn.Module):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig(), independent_value=True):
        super().__init__()
        if not independent_value:
            raise ValueError('The current recipe uses an independent task critic')
        self.actor = LocalActor(config,sensors)
        self.value = nn.Linear(config.hidden,1)
        self.critic = LocalFeatures(config,sensors)

    def initial_state(self, batch_shape=()):
        state=self.actor.initial_state(batch_shape)
        return torch.cat((state,self.critic.initial_state(batch_shape)),-1)

    def features(self, obs, state, reset=None):
        actor_state,critic_state=state.chunk(2,-1)
        actor_features,actor_state=self.actor.features(obs,actor_state,reset)
        critic_features,critic_state=self.critic.features(obs,critic_state,reset)
        return actor_features,critic_features,torch.cat((actor_state,critic_state),-1)

    def forward(self, obs, state, reset=None, latent=None, deterministic=False):
        features, critic_features, state = self.features(obs,state,reset)
        distribution = self.actor.distribution(features)
        if latent is None:
            latent = distribution.mean if deterministic else distribution.sample()
        value = self.value(critic_features).squeeze(-1)
        logprob = latent_log_prob(distribution,latent)
        return latent.tanh(), latent, logprob, value, state


def export_actor(model, path, dt):
    torch.save({'schema':SCHEMA,'kind':'actor','policy':asdict(model.actor.config),
                'sensors':asdict(model.actor.sensors),'dt':dt,'state_dict':model.actor.state_dict(),
                'contract':'local compass frame, final goal delta, velocity, metric lidar, one-hop peers; acceleration in [-1,1], velocity limit per axis 1 m/s'},path)


def load_actor(path, device='cpu'):
    checkpoint = torch.load(path,map_location=device,weights_only=False)
    if checkpoint.get('schema') != SCHEMA or checkpoint.get('kind') != 'actor':
        raise ValueError('Expected local actor export; historical AFOR checkpoints are incompatible')
    actor = LocalActor(PolicyConfig(**checkpoint['policy']),SensorConfig(**checkpoint['sensors'])).to(device)
    actor.load_state_dict(checkpoint['state_dict'])
    return actor.eval(), checkpoint['dt']
