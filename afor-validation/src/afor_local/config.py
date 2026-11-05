"""Validated configurations for the G graph-memory PPO controller."""
from dataclasses import dataclass
import math
from .protocols import VALIDATION_SEEDS, VALIDATION_JITTERS


@dataclass(frozen=True)
class PolicyConfig:
    hidden: int = 64
    capacity_width: int = 24

    def __post_init__(self):
        if self.hidden != 64 or self.capacity_width != 24:
            raise ValueError('The G architecture uses hidden=64 and capacity_width=24')


@dataclass(frozen=True)
class ScenarioConfig:
    perturb_probability: float = .5
    max_jitter: float = .10
    max_rejections: int = 100

    def __post_init__(self):
        if not math.isfinite(self.perturb_probability) or not 0 <= self.perturb_probability <= 1:
            raise ValueError('Perturbation probability must be in [0,1]')
        if not math.isfinite(self.max_jitter) or self.max_jitter < 0 or self.max_rejections < 1:
            raise ValueError('Invalid initial-state perturbation configuration')


@dataclass(frozen=True)
class SelectionConfig:
    """Optional, shared validation-only checkpoint selection protocol."""
    enabled: bool = False
    interval: int = 25
    episodes: int = 24

    def __post_init__(self):
        if not isinstance(self.enabled, bool):
            raise ValueError('Selection enabled flag must be a boolean')
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 1
               for value in (self.interval, self.episodes)):
            raise ValueError('Selection interval and episodes must be positive integers')

    @property
    def validation_seeds(self):
        return VALIDATION_SEEDS

    @property
    def jitters(self):
        return VALIDATION_JITTERS


@dataclass(frozen=True)
class PPOConfig:
    steps: int = 128
    sequence: int = 16
    sequences_per_batch: int = 64
    epochs: int = 4
    lr: float = 3e-4
    gamma: float = .99
    gae_lambda: float = .95
    clip: float = .2
    entropy: float = .001
    barrier_weight: float = 1.

    def __post_init__(self):
        if min(self.steps,self.sequence,self.sequences_per_batch,self.epochs) < 1 or self.steps%self.sequence:
            raise ValueError('Positive PPO dimensions required; steps must be divisible by sequence')
        if not (0 < self.gamma <= 1 and 0 <= self.gae_lambda <= 1 and self.lr > 0):
            raise ValueError('Invalid PPO hyperparameters')
        if self.barrier_weight < 0:
            raise ValueError('Barrier weight must be nonnegative')
