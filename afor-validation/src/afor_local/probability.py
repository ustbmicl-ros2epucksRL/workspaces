"""Tanh-Gaussian likelihood used by the single PPO implementation."""
import math
from torch.nn import functional as F


def latent_log_prob(distribution, latent):
    # Store Gaussian latent in the rollout. No atanh or clipped tail surrogate.
    jacobian = 2*(math.log(2)-latent-F.softplus(-2*latent))
    return (distribution.log_prob(latent)-jacobian).sum(-1)
