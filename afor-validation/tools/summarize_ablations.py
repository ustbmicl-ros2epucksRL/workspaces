"""Build tables, PNG/PDF figures and a concise report from raw ablation runs."""
import argparse
from collections import defaultdict
import csv
import json
from pathlib import Path
import sys
import numpy as np
import torch
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle
from ablation_support import LABELS, CN_LABELS, save_json

FORMS = ['line','circle','wedge']
COLORS = ['#202C39','#0072B2','#E69F00','#009E73','#D55E00']
plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                     'axes.spines.top': False, 'axes.spines.right': False,
                     'savefig.dpi': 180, 'pdf.fonttype': 42})


def average(values):
    values=[v for v in values if v is not None]
    return float(np.mean(values)) if values else None


def mean_sd(values):
    values=[v for v in values if v is not None]
    return (float(np.mean(values)),float(np.std(values,ddof=1)) if len(values)>1 else None,len(values)) if values else (None,None,0)


def percent(values):
    mean,sd,n=mean_sd(values)
    if mean is None:return '—'
    return f'{100*mean:.1f} ± {100*sd:.1f}%' if sd is not None else f'{100*mean:.1f}% (n={n})'


def decimal(values,digits=2):
    mean,sd,n=mean_sd(values)
    if mean is None:return '—'
    return f'{mean:.{digits}f} ± {sd:.{digits}f}' if sd is not None else f'{mean:.{digits}f} (n={n})'


def save_csv(path,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(path).open('w',newline='') as out:
        writer=csv.DictWriter(out,fieldnames=keys);writer.writeheader();writer.writerows(rows)


def savefig(fig,path):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(path,bbox_inches='tight')
    fig.savefig(path.with_suffix('.pdf'),bbox_inches='tight')
    plt.close(fig)


def collect_rows(root):
    rows=[]; runs={};episode_rows=[]
    for done in sorted(root.glob('*/*/seed_*/complete.json')):
        run=done.parent
        report=json.loads((run/'evaluation.json').read_text())
        cfg=report['config'];variant=cfg['variant'];form=cfg['formation'];seed=cfg['seed']
        runs[(variant,form,seed)]={'path':run,'validation':json.loads((run/'validation.json').read_text())}
        for case in report['cases']:
            episodes=case['results'];success=[r for r in episodes if r['success']]
            base={'variant':variant,'formation':form,'training_seed':seed,'width_m':case['width_m'],
                  'environment_seed':case['environment_seed']}
            for r in episodes:
                flat={**base,**{k:v for k,v in r.items() if not isinstance(v,(dict,list))}}
                for i,key in enumerate(('pair','wall','boundary')):flat[f'min_{key}_clearance_m']=r['min_clearance_m'][i]
                for group in ('collision_causes','timeout_causes'):
                    for key,value in r[group].items():flat[group+'_'+key]=value
                episode_rows.append(flat)
            rows.append({**base,'episodes':case['episodes'],'successes':case['successes'],
                         'collisions':case['collisions'],'timeouts':case['timeouts'],'bypasses':case['bypasses'],
                         'success_duration_s':average([r['duration_s'] for r in success]),
                         'success_formation_error_m':average([r['mean_formation_error_m'] for r in success]),
                         'success_path_m':average([r['mean_robot_path_m'] for r in success]),
                         'success_acceleration_rms':average([r['rms_command_acceleration_m_s2'] for r in success]),
                         'success_jerk_rms':average([r['rms_command_jerk_m_s3'] for r in success]),
                         'all_formation_error_m':average([r['mean_formation_error_m'] for r in episodes]),
                         **{f'min_{key}_clearance_m':min(r['min_clearance_m'][i] for r in episodes) for i,key in enumerate(('pair','wall','boundary'))}})
    # Aggregate environment seeds; training seeds remain separate replications.
    grouped=defaultdict(list)
    for row in rows:grouped[(row['variant'],row['formation'],row['training_seed'],row['width_m'])].append(row)
    per_seed=[]
    for (variant,form,seed,width),cases in sorted(grouped.items()):
        count=sum(c['episodes'] for c in cases);wins=sum(c['successes'] for c in cases)
        row={'variant':variant,'formation':form,'training_seed':seed,'width_m':width,'episodes':count,
             **{key:sum(c[key] for c in cases) for key in ('successes','collisions','timeouts','bypasses')},
             'success_rate':wins/count,'collision_rate':sum(c['collisions'] for c in cases)/count,
             'timeout_rate':sum(c['timeouts'] for c in cases)/count}
        for key in ('success_duration_s','success_formation_error_m','success_path_m','success_acceleration_rms','success_jerk_rms'):
            row[key]=sum(c[key]*c['successes'] for c in cases if c[key] is not None)/wins if wins else None
        row['all_formation_error_m']=sum(c['all_formation_error_m']*c['episodes'] for c in cases)/count
        for key in ('pair','wall','boundary'):row[f'min_{key}_clearance_m']=min(c[f'min_{key}_clearance_m'] for c in cases)
        per_seed.append(row)
    return per_seed,runs,episode_rows


def select(rows,variant=None,form=None,width=None):
    return [r for r in rows if (variant is None or r['variant']==variant) and
            (form is None or r['formation']==form) and (width is None or r['width_m']==width)]


def aggregate_variants(rows,variants):
    result=[]
    for variant in variants:
        for form in FORMS:
            for scope in ('training_width','all_widths'):
                part=select(rows,variant,form,1.2 if scope=='training_width' else None)
                if not part:continue
                seeds=sorted({r['training_seed'] for r in part});srs=[]
                for seed in seeds:
                    rr=[r for r in part if r['training_seed']==seed]
                    srs.append(sum(r['successes'] for r in rr)/sum(r['episodes'] for r in rr))
                mean,sd,n=mean_sd(srs)
                result.append({'variant':variant,'formation':form,'scope':scope,'training_seeds':n,
                               'success_rate_mean':mean,'success_rate_sd':sd,
                               **{key:sum(r[key] for r in part) for key in ('episodes','successes','collisions','timeouts','bypasses')}})
    return result


def success_heatmap(rows,variants,path):
    data=np.full((len(variants),3),np.nan);labels={}
    for i,v in enumerate(variants):
        for j,f in enumerate(FORMS):
            part=select(rows,v,f,1.2)
            if not part:continue
            mean,sd,n=mean_sd([r['success_rate'] for r in part]);data[i,j]=100*mean
            labels[i,j]=f'{100*mean:.1f} ± {100*sd:.1f}%' if sd is not None else f'{100*mean:.1f}% (n={n})'
    fig,ax=plt.subplots(figsize=(9,6.5),layout='constrained')
    im=ax.imshow(data,vmin=0,vmax=100,cmap='YlGnBu',aspect='auto')
    for (i,j),label in labels.items():ax.text(j,i,label,ha='center',va='center',color='white' if data[i,j]>65 else '#202C39',fontsize=9)
    ax.set(xticks=range(3),xticklabels=FORMS,yticks=range(len(variants)),yticklabels=[LABELS[v] for v in variants],title='Team success at the training opening (1.20 m)\nMean ± sample SD across training seeds')
    fig.colorbar(im,ax=ax,label='Success (%)',shrink=.7)
    savefig(fig,path)


def learning_plot(runs,variants,path,compare=None):
    fig,axes=plt.subplots(1,3,figsize=(13.5,3.6),sharey=True,layout='constrained')
    used=['full',compare] if compare and compare!='full' else variants
    for ax,form in zip(axes,FORMS):
        for v in used:
            points=defaultdict(list)
            for (variant,f,seed),run in runs.items():
                if variant!=v or f!=form:continue
                for r in run['validation']:points[r['update']].append(r['successes']/r['episodes'])
            if not points:continue
            xs=sorted(points);mean=np.array([np.mean(points[x]) for x in xs])*100
            sd=np.array([np.std(points[x],ddof=1) if len(points[x])>1 else 0 for x in xs])*100
            color=COLORS[variants.index(v)]
            ax.plot(xs,mean,label=LABELS[v],color=color,lw=1.6)
            ax.fill_between(xs,np.clip(mean-sd,0,100),np.clip(mean+sd,0,100),color=color,alpha=.09)
        ax.set(title=form,xlabel='PPO updates',ylim=(-3,103));ax.grid(alpha=.18)
    axes[0].set_ylabel('Validation team success (%)')
    handles,labels=axes[-1].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside lower center',ncol=2 if compare else 5,fontsize=7)
    savefig(fig,path)


def width_plot(rows,variants,path):
    fig,axes=plt.subplots(1,3,figsize=(13.5,3.6),sharey=True,layout='constrained')
    for ax,form in zip(axes,FORMS):
        for i,v in enumerate(variants):
            part=select(rows,v,form);xs=sorted({r['width_m'] for r in part})
            if not xs:continue
            means=[100*np.mean([r['success_rate'] for r in part if r['width_m']==x]) for x in xs]
            ax.plot(xs,means,color=COLORS[i],label=LABELS[v],marker='o',ms=3,lw=1.3,alpha=.9)
        ax.axvline(1.2,color='#808080',lw=.8,ls=':');ax.set(title=form,xlabel='Opening width (m)',ylim=(-3,103));ax.grid(alpha=.18)
    axes[0].set_ylabel('Mean team success (%)')
    fig.legend(*axes[-1].get_legend_handles_labels(),loc='outside lower center',ncol=5,fontsize=7)
    savefig(fig,path)


def width_heatmaps(rows,variants,widths,path):
    fig,axes=plt.subplots(1,3,figsize=(15,6.3),sharey=True,layout='constrained')
    for ax,form in zip(axes,FORMS):
        data=np.full((len(variants),len(widths)),np.nan)
        for i,v in enumerate(variants):
            for j,w in enumerate(widths):
                values=[r['success_rate'] for r in select(rows,v,form,w)]
                if values:data[i,j]=100*np.mean(values)
        im=ax.imshow(data,vmin=0,vmax=100,cmap='YlGnBu',aspect='auto')
        for i in range(len(variants)):
            for j in range(len(widths)):
                if np.isfinite(data[i,j]):
                    label=str(int(data[i,j])) if data[i,j] in (0,100) else f'{data[i,j]:.1f}'
                    ax.text(j,i,label,ha='center',va='center',fontsize=8,color='white' if data[i,j]>65 else '#202C39')
        ax.set(title=form,xticks=range(len(widths)),xticklabels=[f'{w:.1f}' for w in widths],
               yticks=range(len(variants)),yticklabels=[LABELS[v] for v in variants],xlabel='Opening width (m)')
        if 1.2 in widths:
            j=widths.index(1.2)
            ax.add_patch(plt.Rectangle((j-.5,-.5),1,len(variants),fill=False,edgecolor='#E69F00',lw=1.7))
    fig.colorbar(im,ax=axes,label='Mean team success (%)',shrink=.75)
    fig.suptitle('Same final weights across openings; orange column = training opening (1.20 m)',fontsize=11)
    savefig(fig,path)


def seed_sensitivity_plot(rows,root,widths):
    """Show each retained control against the full model for every training seed."""
    controls=['no_braking','no_neighbours','shared_value','no_goal_cost']
    fig,axes=plt.subplots(1,len(controls),figsize=(15,3.8),sharey=True,layout='constrained')
    markers={2:'o',3:'s',4:'^'}
    for ax,v in zip(axes,controls):
        for variant,color,style in [('full','#202C39','-'),(v,COLORS[1+controls.index(v)],'-')]:
            for seed in sorted({r['training_seed'] for r in rows if r['variant']==variant}):
                points=[]
                for width in widths:
                    item=select(rows,variant,'line',width)
                    item=[r for r in item if r['training_seed']==seed]
                    points.append(100*item[0]['success_rate'] if item else np.nan)
                ax.plot(widths,points,marker=markers.get(seed,'o'),color=color,ls=style,
                        alpha=.8 if variant=='full' else .9,lw=1.3,ms=3.5,
                        label=f'{"Full" if variant=="full" else LABELS[variant]}, seed {seed}')
        ax.axvline(1.2,color='#808080',lw=.8,ls=':')
        ax.set(title=LABELS[v],xlabel='Opening width (m)',xticks=widths,ylim=(-3,103));ax.grid(alpha=.2)
    axes[0].set_ylabel('Line formation: team success (%)')
    handles,labels=axes[-1].get_legend_handles_labels()
    if handles:fig.legend(handles,labels,loc='outside lower center',ncol=4,frameon=False,fontsize=7)
    fig.suptitle('Every training seed; fixed weights across the four openings',fontsize=11)
    savefig(fig,root/'figures/seed_sensitivity.png')


def failure_plot(rows,variants,path):
    fig,ax=plt.subplots(figsize=(10,5.5),layout='constrained')
    left=np.zeros(len(variants))
    for key,label,color in [('successes','Success','#009E73'),('collisions','Collision','#D55E00'),('timeouts','Timeout','#E69F00'),('bypasses','Bypass','#CC79A7')]:
        values=[]
        for v in variants:
            part=select(rows,v);n=sum(r['episodes'] for r in part)
            values.append(100*sum(r[key] for r in part)/n if n else 0)
        ax.barh(range(len(variants)),values,left=left,label=label,color=color)
        for i,val in enumerate(values):
            if val>=6:ax.text(left[i]+val/2,i,f'{val:.1f}%',va='center',ha='center',color='white' if key!='timeouts' else '#222',fontsize=8)
        left+=values
    ax.set(yticks=range(len(variants)),yticklabels=[LABELS[v] for v in variants],xlim=(0,100),xlabel='Episodes (%)',title='Outcomes across four openings and three formations')
    ax.invert_yaxis();ax.legend(ncol=4,loc='lower center',bbox_to_anchor=(.5,1.02),frameon=False)
    savefig(fig,path)


def quality_plot(rows,variants,path):
    fig,axes=plt.subplots(1,2,figsize=(11,6),layout='constrained')
    for j,(key,label) in enumerate([('success_duration_s','Successful-episode duration (s)'),('success_formation_error_m','Mean formation error during successful episodes (m)')]):
        ax=axes[j]
        for i,v in enumerate(variants):
            for f,offset,color,marker in [('line',-.2,'#0072B2','o'),('circle',0,'#009E73','s'),('wedge',.2,'#D55E00','^')]:
                values=[r[key] for r in select(rows,v,f,1.2)]
                mean,sd,n=mean_sd(values)
                if mean is None:continue
                ax.errorbar(mean,i+offset,xerr=sd or 0,fmt=marker,color=color,capsize=2,ms=4,label=f if i==0 else None)
        ax.set(yticks=range(len(variants)),yticklabels=[LABELS[v] for v in variants] if j==0 else [],xlabel=label)
        ax.invert_yaxis();ax.grid(axis='x',alpha=.2)
    if axes[0].get_legend_handles_labels()[0]:
        fig.legend(*axes[0].get_legend_handles_labels(),loc='outside lower center',ncol=3,frameon=False)
    fig.suptitle('At 1.20 m; mean ± SD of training-seed means; missing points mean no successful episodes',fontsize=10)
    savefig(fig,path)


def draw_trace(ax,trace,label):
    active=trace['active'][:,0].numpy();count=int(active.sum())
    points=trace['position'][:count+1,0].numpy();goals=trace['goal'][0].numpy();radius=trace['task']['radius']
    for polygon,valid in zip(trace['vertices'][0],trace['obstacle_mask'][0]):
        if valid:ax.add_patch(Polygon(polygon.numpy(),facecolor='#D3D6D9',edgecolor='#7D8288',lw=.6))
    colors=['#0072B2','#E69F00','#009E73','#D55E00','#CC79A7']
    for robot,color in enumerate(colors):
        ax.plot(points[:,robot,0],points[:,robot,1],color=color,lw=1.1)
        ax.add_patch(Circle(points[0,robot],radius,fill=False,edgecolor=color,lw=.8))
        ax.add_patch(Circle(points[-1,robot],radius,facecolor=color,edgecolor='white',lw=.4))
        ax.scatter(*goals[robot],marker='x',color=color,s=20,lw=.8)
    row=trace['episode_results'][0]
    status='SUCCESS' if row['success'] else ('COLLISION' if row['collision'] else ('TIMEOUT' if row['timeout'] else 'BYPASS'))
    if row['collision']:
        proposal=trace['proposal'][count-1,0].numpy()
        for r in range(5):ax.plot([points[-1,r,0],proposal[r,0]],[points[-1,r,1],proposal[r,1]],'r--',lw=1)
    ax.set_title(f'{label}\n{status}; {count*.05:.2f} s',fontsize=8,color='#992D22' if not row['success'] else '#263D32')
    ax.set(xlim=(-2,2),ylim=(-3.6,3.5),xlabel='x (m)',ylabel='y (m)');ax.set_aspect('equal');ax.grid(alpha=.12)


def trace_plots(root,runs,variants,widths):
    for v in variants:
        fig,axes=plt.subplots(3,len(widths),figsize=(3.4*len(widths),12),layout='constrained');available=False
        for i,f in enumerate(FORMS):
            for j,w in enumerate(widths):
                p=root/v/f/'seed_2'/'traces'/f'width_{w:.1f}.pt'
                if p.exists():draw_trace(axes[i,j],torch.load(p,weights_only=False),f'{f}; opening {w:.1f} m');available=True
                else:axes[i,j].set_visible(False)
        fig.suptitle(f'{LABELS[v]} | fixed training seed 2, test seed 61001, episode 0',fontsize=11)
        if available:savefig(fig,root/v/'figures/trajectories.png')
        else:plt.close(fig)
    for f in FORMS:
        fig,axes=plt.subplots(len(variants),len(widths),figsize=(3.4*len(widths),3.1*len(variants)),layout='constrained',squeeze=False);available=False
        for i,v in enumerate(variants):
            for j,w in enumerate(widths):
                p=root/v/f/'seed_2'/'traces'/f'width_{w:.1f}.pt'
                if p.exists():draw_trace(axes[i,j],torch.load(p,weights_only=False),f'{LABELS[v]}; {w:.1f} m');available=True
                else:axes[i,j].set_visible(False)
        fig.suptitle(f'{f}: fixed training seed 2, test seed 61001, episode 0',fontsize=12)
        if available:savefig(fig,root/'figures'/f'trajectories_{f}.png')
        else:plt.close(fig)


def failure_cause_plot(episodes,variants,path):
    fig,axes=plt.subplots(1,2,figsize=(12,6),sharey=True,layout='constrained')
    groups=[('collision_causes_',('pair','wall','boundary'),'Collision causes (% of all episodes)'),
            ('timeout_causes_',('distance','speed','shape'),'Timeout causes (% of all episodes)')]
    for ax,(prefix,causes,title) in zip(axes,groups):
        for j,cause in enumerate(causes):
            values=[]
            for v in variants:
                rr=[r for r in episodes if r['variant']==v]
                values.append(100*sum(r[prefix+cause] for r in rr)/len(rr) if rr else 0)
            ax.barh(np.arange(len(variants))+(j-1)*.23,values,height=.21,label=cause,color=['#0072B2','#D55E00','#009E73'][j])
        ax.set(yticks=range(len(variants)),yticklabels=[LABELS[v] for v in variants],xlabel=title,xlim=(0,100));ax.invert_yaxis();ax.grid(axis='x',alpha=.15);ax.legend(frameon=False)
    fig.suptitle('Causes may overlap within an episode; rates must not be added',fontsize=11)
    savefig(fig,path)


def convergence_rows(runs):
    rows=[]
    for (variant,form,seed),run in sorted(runs.items()):
        history=run['validation']
        perfect=[r['successes']==r['episodes'] for r in history]
        first=next((r['update'] for r,ok in zip(history,perfect) if ok),None)
        stable=next((history[i]['update'] for i in range(len(history)) if all(perfect[i:])),None)
        metrics=[json.loads(line) for line in (run['path']/'metrics.jsonl').read_text().splitlines()]
        late=metrics[-50:];count=sum(r['completed_episodes'] for r in late)
        rows.append({'variant':variant,'formation':form,'training_seed':seed,
                     'first_perfect_validation_update':first,
                     'perfect_on_all_later_validation_since':stable,
                     'last_three_validations_all_perfect':all(perfect[-3:]),
                     'mean_validation_success_rate':average([r['successes']/r['episodes'] for r in history]),
                     'last_50_training_episodes':count,
                     'last_50_stochastic_training_success_rate':sum((r['success'] or 0)*r['completed_episodes'] for r in late)/count if count else None,
                     'last_50_stochastic_training_collision_rate':sum((r['collision'] or 0)*r['completed_episodes'] for r in late)/count if count else None})
    return rows


def learning_speed_plot(runs,variants,path):
    """Use all predeclared checkpoints, including models that never converge."""
    data=convergence_rows(runs)
    fig,axes=plt.subplots(1,2,figsize=(12,6),sharey=True,layout='constrained')
    for j,(metric,xlabel) in enumerate([
        ('mean_validation_success_rate','Mean success over all validation checkpoints (%)'),
        ('last_50_stochastic_training_collision_rate','Last 50 updates: stochastic training collision (%)')]):
        ax=axes[j]
        for i,v in enumerate(variants):
            for f,offset,color,marker in [('line',-.2,'#0072B2','o'),('circle',0,'#009E73','s'),('wedge',.2,'#D55E00','^')]:
                values=[100*r[metric] for r in data if r['variant']==v and r['formation']==f and r[metric] is not None]
                mean,sd,n=mean_sd(values)
                if mean is not None:ax.errorbar(mean,i+offset,xerr=sd or 0,fmt=marker,color=color,capsize=2,ms=4,label=f if i==0 else None)
        ax.set(yticks=range(len(variants)),yticklabels=[LABELS[v] for v in variants],xlabel=xlabel,xlim=(-3,103));ax.invert_yaxis();ax.grid(axis='x',alpha=.2)
    if axes[0].get_legend_handles_labels()[0]:axes[0].legend(frameon=False,loc='lower right')
    fig.suptitle('Mean ± SD across training seeds; exploration collisions are separate from deterministic evaluation',fontsize=10)
    savefig(fig,path)


def write_reports(root,rows,runs,variants,protocol,partial):
    widths=protocol['widths_m']
    expected=len(variants)*len(protocol['formations'])*len(protocol['training_seeds'])
    all_episodes=sum(r['episodes'] for r in rows)
    header='# 当前算法消融实验结果\n\n'
    if partial:header+='> **实验进行中：以下仅统计已完成任务，不是最终结论。**\n\n'
    seed_text='/'.join(map(str,protocol['training_seeds']))
    header+=f'完成 **{len(runs)}/{expected}** 个训练任务，评测 **{all_episodes:,}** 个团队回合。五组方案覆盖三种队形、训练种子 {seed_text}，每次 {protocol["updates"]} 次更新；统一在 1.20 m 开口训练，并以同一最终权重评测 {"、".join(f"{w:.1f}" for w in widths)} m 四档开口。每档使用 {len(protocol["test_seeds"])} 个环境种子、每种子 {protocol["episodes_per_case"]} 回合。表中 ± 为训练种子间样本标准差。\n\n'
    table=['| 方案 | 横排 1.20 m | 圆形 1.20 m | 楔形 1.20 m | 四档总成功数 | 碰撞 | 超时 |',
           '|---|---:|---:|---:|---:|---:|---:|']
    for v in variants:
        cells=[percent([r['success_rate'] for r in select(rows,v,f,1.2)]) for f in FORMS]
        part=select(rows,v);n=sum(r['episodes'] for r in part);wins=sum(r['successes'] for r in part)
        table.append(f'| [{CN_LABELS[v]}]({v}/README.md) | '+' | '.join(cells)+f' | {wins}/{n} | {sum(r["collisions"] for r in part)} | {sum(r["timeouts"] for r in part)} |')
    text=header+'\n'.join(table)+'\n\n'
    text+='![训练开口成功率](figures/success_by_formation.png)\n\n![四档开口表现](figures/width_heatmaps.png)\n\n'
    text+='## 结果\n\n'
    if not partial and all(v in variants for v in ('full','no_braking','no_neighbours','shared_value','no_goal_cost')):
        rate=lambda v,f:percent([r['success_rate'] for r in select(rows,v,f,1.2)])
        total=lambda v:f'{sum(r["successes"] for r in select(rows,v))}/{sum(r["episodes"] for r in select(rows,v))}'
        text+=f'- **局部制动损失增益最大**：完整方案四档成功 {total("full")}；关闭制动损失后为 {total("no_braking")}。在 1.20 m 横排／楔形，关闭制动后的成功率分别为 {rate("no_braking","line")}／{rate("no_braking","wedge")}。\n'
        text+=f'- **邻居输入、独立价值编码器和目标距离代价均带来可测差异**：关闭 actor 邻居输入、共享价值编码器、关闭目标距离代价后，横排 1.20 m 成功率分别为 {rate("no_neighbours","line")}、{rate("shared_value","line")}、{rate("no_goal_cost","line")}；完整方案为 {rate("full","line")}。去邻居组的训练 critic 与局部制动教师保持原始输入。\n'
        convergence=convergence_rows(runs)
        full_runs=[r for r in convergence if r['variant']=='full']
        late=sum(r['last_three_validations_all_perfect'] for r in full_runs)
        text+=f'- **训练后期表现**：完整方案 {late}/{len(full_runs)} 个运行在最后三次定期验证均满成功。此为经验稳定性指标，不是理论收敛保证。\n'
        text+='\n这些结果支持保留四项机制；共享价值组同时减少了训练参数，不能把观察到的差异单独归因于梯度冲突。单项控制不等于把多个机制同时删去后的效果。\n'
    else:
        text+='数据未齐时仅展示表格，不按部分种子给机制排名。\n'
    text+='\n![横排各训练种子的跨开口表现](figures/seed_sensitivity.png)\n\n'
    text+='\n![失败构成](figures/failure_breakdown.png)\n\n![训练过程](figures/learning_curves.png)\n\n'
    text+='[学习效率与训练碰撞](figures/learning_speed.png) · [碰撞／超时原因](figures/failure_causes.png) · [开口变化曲线](figures/width_success.png)。\n\n'
    text+='## 轨迹与数据\n\n'
    text+='轨迹固定为训练 seed=2、测试 seed=61001、第 0 回合；各方案和开口使用相同索引。空心圆为起点，实心圆为终点，叉号为目标，碰撞候选步用红色虚线表示。\n\n'
    for f in FORMS:text+=f'- [{f}：五组方案在四档开口的轨迹](figures/trajectories_{f}.png)\n'
    text+='\n![成功回合的时间与队形误差](figures/time_and_formation.png)\n\n'
    text+='仅对成功回合比较通行时间和过程队形误差；失败率须同时参考，不能把提前碰撞造成的短轨迹当作效率提升。命令加速度／jerk 指标不代表实物振动。\n\n'
    text+='## 数据与复现\n\n'
    text+='- [实验协议](protocol.json)；[按训练种子与开口统计](per_seed_width.csv)；[逐回合数据](episodes.csv)；[收敛统计](convergence.csv)；[完整性核验](final_verification.json)。\n'
    text+='- 每组目录下：`<formation>/seed_<n>/` 包含配置、逐更新指标、验证历史、最终权重、评测和原始轨迹；`source/` 保存这轮源码快照。\n'
    text+='- 结果覆盖原渐缩场景；不代表任意未知环境、噪声／延迟或实物验证，也不构成 AFOR 公平基线。\n\n'
    text+='```bash\npython tools/summarize_ablations.py --root results/ablations\npython tools/verify_ablations.py --root results/ablations\n```\n'
    (root/'README.md').write_text(text)
    for v in variants:
        folder=root/v;folder.mkdir(exist_ok=True)
        part=select(rows,v)
        save_csv(folder/'per_seed_width.csv',part)
        desc={'full':'完整方案对照。','no_neighbours':'屏蔽部署用 actor 的邻居特征和数量；训练 critic 与局部制动教师仍使用相同原始量测。激光仍能感知其他机器人。',
              'no_braking':'制动辅助损失权重置零。',
              'shared_value':'价值头使用 actor 特征，移除独立 critic 编码器；actor 容量不变。',
              'no_goal_cost':'每步目标距离代价置零；保留目标进展和近目标速度项。'}[v]
        s=f'# {CN_LABELS[v]}\n\n{desc}\n\n'
        s+='| 队形 | 1.20 m 成功率 | 四档成功数 | 碰撞 | 超时 | 成功耗时（s，1.20 m） | 成功回合过程队形误差（m） |\n|---|---:|---:|---:|---:|---:|---:|\n'
        for f in FORMS:
            rr=select(part,form=f);at=select(rr,width=1.2)
            s+=f'| {f} | {percent([r["success_rate"] for r in at])} | {sum(r["successes"] for r in rr)}/{sum(r["episodes"] for r in rr)} | {sum(r["collisions"] for r in rr)} | {sum(r["timeouts"] for r in rr)} | {decimal([r["success_duration_s"] for r in at])} | {decimal([r["success_formation_error_m"] for r in at],3)} |\n'
        s+='\n![与完整方案的学习曲线](figures/learning_curves.png)\n\n![固定回合轨迹](figures/trajectories.png)\n\n[逐种子、逐开口数据](per_seed_width.csv) · [总体总结](../README.md)\n'
        (folder/'README.md').write_text(s)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('results/ablations'))
    parser.add_argument('--allow-partial',action='store_true')
    args=parser.parse_args();root=args.root.resolve()
    protocol=json.loads((root/'protocol.json').read_text())['protocol'];variants=list(protocol['variants'])
    rows,runs,episodes=collect_rows(root)
    expected=len(variants)*len(protocol['formations'])*len(protocol['training_seeds'])
    partial=len(runs)!=expected
    if partial and not args.allow_partial:raise RuntimeError(f'Only {len(runs)}/{expected} jobs completed; explicitly use --allow-partial for a labelled draft')
    if not rows:raise RuntimeError('No complete evaluation yet')
    figures=root/'figures';figures.mkdir(exist_ok=True)
    widths=protocol['widths_m']
    agg=aggregate_variants(rows,variants)
    save_csv(root/'per_seed_width.csv',rows);save_csv(root/'episodes.csv',episodes)
    save_csv(root/'summary.csv',agg);save_json(root/'summary.json',agg)
    save_csv(root/'convergence.csv',convergence_rows(runs))
    success_heatmap(rows,variants,figures/'success_by_formation.png')
    learning_plot(runs,variants,figures/'learning_curves.png')
    width_plot(rows,variants,figures/'width_success.png')
    width_heatmaps(rows,variants,widths,figures/'width_heatmaps.png')
    seed_sensitivity_plot(rows,root,widths)
    failure_plot(rows,variants,figures/'failure_breakdown.png')
    failure_cause_plot(episodes,variants,figures/'failure_causes.png')
    quality_plot(rows,variants,figures/'time_and_formation.png')
    learning_speed_plot(runs,variants,figures/'learning_speed.png')
    for v in variants:
        learning_plot(runs,variants,root/v/'figures/learning_curves.png',compare=v)
    trace_plots(root,runs,variants,widths)
    write_reports(root,rows,runs,variants,protocol,partial)
    print(json.dumps({'completed_jobs':len(runs),'expected_jobs':expected,'episodes':len(episodes),'partial':partial,'report':str(root/'README.md')}))


if __name__=='__main__':main()
