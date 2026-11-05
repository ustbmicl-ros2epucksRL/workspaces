"""Local graph-attention actor, recurrent memory, and an independent critic."""
from typing import Dict, Optional, Tuple
import torch
from torch import Tensor, nn
from torch.distributions import Normal
from .observation import SensorConfig
from .critic import LocalCritic
from .probability import latent_log_prob
from .config import PolicyConfig
from .modules import LocalGraph


class LocalActor(nn.Module):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig()):
        super().__init__()
        if sensors.lidar_rays != 32:
            raise ValueError('The G actor requires 32 lidar rays')
        self.config, self.sensors = config, sensors
        # Keep the verified G layer order, initialization, and state-dict names.
        self.ego = nn.Sequential(nn.Linear(37, 32), nn.Tanh(), nn.Linear(32, 32), nn.Tanh())
        self.compensation = nn.Sequential(nn.Linear(32, 24), nn.Tanh(), nn.Linear(24, 32))
        self.graph = LocalGraph()
        self.fuse = nn.Sequential(nn.Linear(65, 64), nn.Tanh(), nn.Linear(64, 64), nn.Tanh())
        self.cell = nn.GRUCell(64, 64)
        self.mean = nn.Linear(64, 2)
        self.log_std = nn.Parameter(torch.full((2,), -.7))
        nn.init.orthogonal_(self.mean.weight, .01)
        nn.init.zeros_(self.mean.bias)

    def initial_state(self, batch_shape=()):
        return self.mean.weight.new_zeros(tuple(batch_shape) + (64,))

    def features_with_attention(self, obs: Dict[str, Tensor], state: Tensor,
                                reset: Optional[Tensor] = None
                                ) -> Tuple[Tensor, Tensor, Tensor]:
        if reset is not None:
            state = torch.where(reset.unsqueeze(-1), torch.zeros_like(state), state)
        own = self.ego(torch.cat((obs['ego'], obs['scan']), -1))
        own = own + .1 * self.compensation(own)
        pooled, attention = self.graph(own, state, obs['neighbours'], obs['mask'])
        count = obs['mask'].sum(-1, keepdim=True).clamp_max(10).to(own.dtype) / 10
        encoded = self.fuse(torch.cat((own, pooled, count), -1))
        hidden = self.cell(encoded.reshape(-1, 64), state.reshape(-1, 64)).reshape_as(state)
        return hidden, hidden, attention

    def features(self, obs: Dict[str, Tensor], state: Tensor, reset: Optional[Tensor] = None
                 ) -> Tuple[Tensor, Tensor]:
        features, state, _ = self.features_with_attention(obs, state, reset)
        return features, state

    def distribution(self, features):
        return Normal(self.mean(features), self.log_std.clamp(-4, 1).exp())

    def forward(self, obs: Dict[str, Tensor], state: Tensor, reset: Optional[Tensor] = None
                ) -> Tuple[Tensor, Tensor]:
        features, state = self.features(obs, state, reset)
        return self.mean(features).tanh(), state

    def parameter_counts(self):
        count = sum(p.numel() for p in self.parameters())
        return {'training': count, 'decision': count}


class ActorCritic(nn.Module):
    def __init__(self, config=PolicyConfig(), sensors=SensorConfig(), seed=0):
        super().__init__()
        # Independent CPU generators preserve caller RNG and critic initialization.
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(seed)
            self.actor = LocalActor(config, sensors)
        with torch.random.fork_rng(devices=[]):
            torch.random.default_generator.manual_seed(seed + 100000)
            self.critic = LocalCritic(config, sensors)
            self.value = nn.Linear(64, 1)

    def initial_state(self, batch_shape=()):
        return torch.cat((self.actor.initial_state(batch_shape),
                          self.critic.initial_state(batch_shape)), -1)

    def features_with_attention(self, obs, state, reset=None):
        actor_state, critic_state = state.chunk(2, -1)
        actor_features, actor_state, attention = self.actor.features_with_attention(obs, actor_state, reset)
        critic_features, critic_state = self.critic.features(obs, critic_state, reset)
        return actor_features, critic_features, torch.cat((actor_state, critic_state), -1), attention

    def features(self, obs, state, reset=None):
        actor_features, critic_features, state, _ = self.features_with_attention(obs, state, reset)
        return actor_features, critic_features, state

    def forward(self, obs, state, reset=None, latent=None, deterministic=False):
        features, critic_features, state = self.features(obs, state, reset)
        distribution = self.actor.distribution(features)
        if latent is None:
            latent = distribution.mean if deterministic else distribution.sample()
        value = self.value(critic_features).squeeze(-1)
        return latent.tanh(), latent, latent_log_prob(distribution, latent), value, state
