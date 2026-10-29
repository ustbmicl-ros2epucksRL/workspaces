"""Verify the original narrowing scene and all three task formations."""
import argparse
from pathlib import Path
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from afor_local.environment import LocalFormationEnv, TaskConfig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    torch.set_num_threads(1)
    fig, axes = plt.subplots(1, 3, figsize=(10, 7.3), constrained_layout=True)
    colors = plt.get_cmap('tab10').colors
    for ax, formation in zip(axes, ('line', 'circle', 'wedge')):
        env = LocalFormationEnv(TaskConfig(num_envs=1, formation=formation), seed=41001)
        for vertices, active in zip(env.vertices[0], env.obstacle_mask[0]):
            if active:
                ax.add_patch(Polygon(vertices.numpy(), facecolor='#c6cbd0', edgecolor='#808992'))
        for i, (start, goal) in enumerate(zip(env.pos[0], env.goal[0])):
            ax.add_patch(Circle(start.numpy(), env.radius, facecolor=colors[i], zorder=3))
            ax.scatter(*goal.numpy(), color=colors[i], marker='x', s=45, zorder=3)
        for i, j in env.adjacency[0].triu(1).nonzero().tolist():
            for positions in (env.pos[0], env.goal[0]):
                ax.plot(positions[[i,j],0], positions[[i,j],1], ':', color='#777777', lw=.8)
        ax.annotate('', xy=(-.6, 1.52), xytext=(.6, 1.52),
                    arrowprops={'arrowstyle': '<->', 'color': '#30363d'})
        ax.text(0, 1.66, '1.20 m opening', ha='center', fontsize=9)
        ax.set(xlim=(-2,2), ylim=(-4,4), xlabel='x (m)', ylabel='y (m)', title=formation)
        ax.set_aspect('equal'); ax.grid(alpha=.15)
    fig.suptitle('Original narrowing corridor: three formations\nFilled disks: starts; crosses: final goals (no planned route)', fontsize=12)
    output = Path(args.output); output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180); fig.savefig(output.with_suffix('.pdf'))
    plt.close(fig)
    print(output)


if __name__ == '__main__':
    main()
