"""One-hop edge messages with permutation-invariant masked attention."""
from typing import Tuple
import torch
from torch import Tensor, nn


def masked_attention(scores: Tensor, mask: Tensor) -> Tensor:
    """Support empty neighbour axes and finite all-masked rows."""
    masked = scores.masked_fill(~mask, float('-inf'))
    safe = torch.where(mask.any(-1, keepdim=True), masked, torch.zeros_like(masked))
    weights = safe.softmax(-1) * mask.to(scores.dtype)
    return weights / weights.sum(-1, keepdim=True).clamp_min(1e-6)


class LocalGraph(nn.Module):
    def __init__(self):
        super().__init__()
        self.peer = nn.Sequential(nn.Linear(8, 32), nn.Tanh(), nn.Linear(32, 32), nn.Tanh())
        self.score = nn.Sequential(nn.Linear(128, 32), nn.Tanh(), nn.Linear(32, 1))

    def forward(self, own: Tensor, state: Tensor, neighbours: Tensor,
                mask: Tensor) -> Tuple[Tensor, Tensor]:
        # Remove padding before the first affine layer, including padded NaNs.
        edge = self.peer(torch.where(mask.unsqueeze(-1), neighbours, 0.))
        local = own.unsqueeze(-2).expand_as(edge)
        context = state.unsqueeze(-2).expand(list(edge.shape[:-1]) + [64])
        scores = self.score(torch.cat((local, context, edge), -1)).squeeze(-1)
        weights = masked_attention(scores, mask)
        return (edge * weights.unsqueeze(-1)).sum(-2), weights
