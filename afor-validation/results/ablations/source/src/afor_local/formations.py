"""Fixed mission templates and role edges, independent of passage geometry."""
from dataclasses import dataclass
import math
import torch


@dataclass(frozen=True)
class Formation:
    template: torch.Tensor
    edges: tuple


def make_formation(name, device):
    if name == 'line':
        coordinates = [(-1., 0.), (-.5, 0.), (0., 0.), (.5, 0.), (1., 0.)]
        edges = ((0, 1), (1, 2), (2, 3), (3, 4))
    elif name == 'circle':
        coordinates = [(.6*math.cos(math.pi/2+2*math.pi*i/5),
                        .6*math.sin(math.pi/2+2*math.pi*i/5)) for i in range(5)]
        edges = ((0, 1), (1, 2), (2, 3), (3, 4), (4, 0))
    elif name == 'wedge':
        coordinates = [(-.7, -.4), (-.35, .1), (0., .6), (.35, .1), (.7, -.4)]
        edges = ((0, 1), (1, 2), (2, 3), (3, 4))
    else:
        raise ValueError(f'Unknown formation: {name}')
    return Formation(torch.tensor(coordinates, dtype=torch.float32, device=device), edges)
