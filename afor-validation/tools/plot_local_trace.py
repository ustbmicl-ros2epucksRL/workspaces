"""Show unsmoothed local-policy motion and the actual visible task geometry."""
import argparse
from pathlib import Path
import torch
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon,Circle


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('traces',nargs='+')
    parser.add_argument('--output',required=True)
    parser.add_argument('--episode',type=int,default=0)
    parser.add_argument('--labels',nargs='+',help='Optional panel labels in the same order as traces')
    args=parser.parse_args()
    if args.labels and len(args.labels)!=len(args.traces):
        parser.error('Supply one label for each trace')
    fig,axes=plt.subplots(1,len(args.traces),figsize=(4.2*len(args.traces),5),squeeze=False,constrained_layout=True)
    for panel,(ax,name) in enumerate(zip(axes[0],args.traces)):
        trace=torch.load(name,weights_only=False)
        i=args.episode
        active=trace['active'][:,i].numpy()
        count=int(active.sum())
        points=trace['position'][:count+1,i].numpy()
        goals=trace['goal'][i].numpy()
        radius=trace['task']['radius']
        for polygon,valid in zip(trace['vertices'][i],trace['obstacle_mask'][i]):
            if valid:ax.add_patch(Polygon(polygon.numpy(),facecolor='#c6c9cd',edgecolor='#888b90',lw=.7))
        colors=plt.get_cmap('tab10').colors
        for robot in range(points.shape[1]):
            color=colors[robot]
            ax.plot(points[:,robot,0],points[:,robot,1],color=color,lw=1.6)
            ax.scatter(*goals[robot],marker='x',color=color,s=35,zorder=3)
            ax.add_patch(Circle(points[0,robot],radius,fill=False,edgecolor=color,lw=1))
            ax.add_patch(Circle(points[-1,robot],radius,facecolor=color,edgecolor='white',lw=.5,zorder=4))
        order=[0,1,2,3,4,0] if trace['task']['formation']=='circle' else [0,1,2,3,4]
        ax.plot(points[-1,order,0],points[-1,order,1],color='#444444',ls=':',lw=.7)
        final_distance=np.linalg.norm(points[-1]-goals,axis=-1).mean()
        label=args.labels[panel] if args.labels else Path(name).parent.name.replace('independent_','indep. ')
        ax.set_title(f'{label}\n{count*trace["task"]["dt"]:.2f} s; mean final distance {final_distance:.2f} m',fontsize=10)
        bounds=trace.get('bounds',torch.tensor([5.5,5.5])).tolist()
        ax.set_aspect('equal');ax.set(xlabel='x (m)',ylabel='y (m)',xlim=(-bounds[0],bounds[0]),ylim=(-bounds[1],bounds[1]))
        ax.grid(alpha=.15)
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(output,dpi=180)
    fig.savefig(output.with_suffix('.pdf'))
    plt.close(fig)
    print(output)


if __name__=='__main__':main()
