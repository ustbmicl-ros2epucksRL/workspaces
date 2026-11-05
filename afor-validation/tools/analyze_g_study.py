"""Generate paper-ready G tables and fixed, validation-chosen trajectory figures."""
import argparse
import json
from pathlib import Path

import numpy as np
import torch

from afor_local.artifacts import file_hash, save_json
from afor_local.checkpoints import load_actor
from afor_local.evaluate import evaluate_case
from afor_local.protocols import ANALYSIS_SEEDS, FORMATIONS
from afor_local.reporting import summarize


def mean_std(values, percent=False):
    values = [value for value in values if value is not None]
    if not values:
        return '—'
    scale = 100 if percent else 1
    mean = np.mean(values) * scale
    std = np.std(values, ddof=1) * scale if len(values) > 1 else None
    text = f'{mean:.2f}' + (f' ± {std:.2f}' if std is not None else '')
    return text + ('%' if percent else '')


def write_tables(root, report):
    lines = ['# G 三队形重新训练结果', '',
             f"状态：{report['status']}；完成{report['completed_runs']}/{report['expected_runs']}次训练与独立测试，共{report['test_episodes']:,}回合。",
             '每次从零训练500更新；actor.pt由验证集选择。下列±均为独立训练种子间样本标准差，不是回合级置信区间。', '',
             '| 队形 | 训练种子数 | 常规成功率 | 主挑战成功率 | 0.20 m压力 | 主挑战惩罚耗时/s |',
             '|---|---:|---:|---:|---:|---:|']
    for group in report['groups']:
        formation = group['formation']
        rows = [row for row in report['per_run'] if row['formation'] == formation]
        def scores(family, metric=None):
            return [r['families'][family]['success_rate'] if metric is None else r['families'][family]['all'][metric] for r in rows]
        lines.append(f"| {formation} | {len(rows)} | {mean_std(scores('nominal_regression'), True)} | {mean_std(scores('main_challenge'), True)} | {mean_std(scores('stronger_perturbation'), True)} | {mean_std(scores('main_challenge', 'failure_penalized_duration'))} |")
    lines += ['', '失败惩罚耗时：成功取实际完成时间，失败按37.5 s计。主挑战为宽1.2 m、扰动0.05/0.10/0.15 m三档。', '',
              '## 全部独立训练种子', '',
              '| 队形 | 训练seed | 所选更新 | 主挑战成功数 | 主挑战成功率 | 惩罚耗时/s |',
              '|---|---:|---:|---:|---:|---:|']
    for row in report['per_run']:
        score = row['families']['main_challenge']
        lines.append(f"| {row['formation']} | {row['seed']} | {row['selected_update']} | {score['successes']}/{score['episodes']} | {score['success_rate'] * 100:.2f}% | {score['all']['failure_penalized_duration']:.2f} |")
    lines += ['', '## 宽度迁移与联合压力', '',
              '| 队形 | 宽度/m | 扰动/m | 成功率均值±种子标准差 |',
              '|---|---:|---:|---:|']
    for formation in FORMATIONS:
        rows = [r for r in report['per_run'] if r['formation'] == formation]
        for width, jitter in [(w, 0.) for w in (1.1, 1.2, 1.3, 1.4, 1.5, 1.6)] + [(1.1, .15)]:
            scores = []
            for row in rows:
                cases = json.loads((Path(row['path']) / 'evaluation.json').read_text())['cases']
                selected = [c for c in cases if c['passage_width'] == width and c['jitter'] == jitter]
                if selected:
                    scores.append(sum(c['successes'] for c in selected) / sum(c['episodes'] for c in selected))
            if scores:
                lines.append(f'| {formation} | {width:.2f} | {jitter:.2f} | {mean_std(scores, True)} |')
    lines += ['', '## 主挑战成功回合中的运动与形状诊断', '',
              '| 队形 | 每机器人里程/m | 控制努力代理 | 过程模板误差/m | 恢复时间/s |',
              '|---|---:|---:|---:|---:|']
    for formation in FORMATIONS:
        rows = [r for r in report['per_run'] if r['formation'] == formation]
        if rows:
            values = [mean_std([r['families']['main_challenge']['success'][key] for r in rows])
                      for key in ('mean_path_length', 'mean_control_effort', 'mean_process_formation_error', 'recovery_time')]
            lines.append('| ' + formation + ' | ' + ' | '.join(values) + ' |')
    lines += ['', '控制努力是实际加速度平方积分，单位m²/s³，不是实机能耗。过程误差以固定世界轴模板为参照，合理变形也会增加误差；低误差不能单独证明导航更好。无成功回合的种子不产生success-only均值，原始JSON保留各子组分母。', '',
              '固定轨迹/动画使用分析seed184001、episode0、宽1.2 m、扰动0.15 m；代表网络依据开发验证选择。失败案例不替换，不参与训练与选型。', '',
              '本轮只研究G在三队形任务中的能力；没有实现外部对照，不声称超过原AFOR或验证新机制。固定五圆盘机器人与理想量测的结果不等于实机、任意未知地图或规模泛化证据。', '']
    (root / 'analysis' / 'RESULTS.md').write_text('\n'.join(lines))


def fixed_figures(root, animate=True):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter
    from matplotlib.patches import Circle, Polygon
    recommendation_path = root / 'recommended_models.json'
    study = json.loads((root / 'study.json').read_text())
    if file_hash(recommendation_path) != study.get('recommendations_sha256'):
        raise ValueError('Validation recommendations differ from the pre-test freeze')
    recommendations = json.loads(recommendation_path.read_text())
    if (recommendations.get('schema') != 'afor_g_recommended_models_v1'
            or recommendations.get('source_hash') != study['source_hash']):
        raise ValueError('Recommended model registration differs from the study')
    traces, identities = [], []
    for formation in FORMATIONS:
        candidate = recommendations['formations'].get(formation)
        if candidate is None:
            continue
        path = Path(candidate['source_actor'])
        if file_hash(path) != candidate['actor_sha256']:
            raise ValueError('Recommended actor hash has changed')
        expected_path = root / formation / f"seed_{candidate['training_seed']}" / 'actor.pt'
        run = json.loads(path.with_name('run.json').read_text())
        if (path.resolve() != expected_path.resolve()
                or run['config']['task']['formation'] != formation
                or run['config']['seed'] != candidate['training_seed']
                or run['source_hash'] != study['source_hash']
                or run['actor_sha256'] != candidate['actor_sha256']
                or run['selection']['selected_update'] != candidate['selected_update']):
            raise ValueError('Illustration actor differs from its frozen selected run')
        actor, dt = load_actor(path)
        report, trace = evaluate_case(actor, dt, formation, jitter=.15, seed=ANALYSIS_SEEDS[0], episodes=1, trace=True)
        traces.append((formation, trace, report['results'][0]))
        identities.append({'formation': formation, 'actor_sha256': candidate['actor_sha256'],
                           'training_seed': candidate['training_seed'], 'selected_update': candidate['selected_update'],
                           'analysis_seed': ANALYSIS_SEEDS[0], 'episode': 0,
                           'case_hash': report['case_hash'], 'result': report['results'][0]})
    if not traces:
        raise ValueError('No validation-selected actors to illustrate')
    output = root / 'analysis'
    save_json(output / 'illustration.json', {'selection': 'Fixed analysis case; models selected using validation only', 'cases': identities})
    fig, axes = plt.subplots(1, len(traces), figsize=(4.2 * len(traces), 6), squeeze=False, constrained_layout=True)
    colors = ('#0072B2', '#D55E00', '#009E73', '#CC79A7', '#E69F00')
    disks, histories = [], []
    for ax, (formation, trace, result) in zip(axes[0], traces):
        for vertices, active in zip(trace['vertices'][0], trace['obstacle_mask'][0]):
            if active:
                ax.add_patch(Polygon(vertices.numpy(), facecolor='#D9DDE1', edgecolor='#737B82', lw=.8))
        positions = trace['position'][:, 0].numpy()
        panel_disks, panel_histories = [], []
        for i, color in enumerate(colors):
            ax.plot(positions[:, i, 0], positions[:, i, 1], color=color, lw=1.4)
            ax.scatter(*trace['goal'][0, i].numpy(), color=color, marker='x', s=35)
            ax.add_patch(Circle(positions[0, i], .13, edgecolor=color, fill=False))
            disk = Circle(positions[-1, i], .13, facecolor=color, edgecolor='white', lw=.5, zorder=4)
            ax.add_patch(disk)
            panel_disks.append(disk)
        state = 'success' if result['success'] else result['failure_type']
        ax.set(title=f"{formation}: {state}, {result['duration']:.2f} s", xlabel='x (m)', ylabel='y (m)', xlim=(-2, 2), ylim=(-4, 4))
        ax.set_aspect('equal')
        ax.grid(alpha=.15)
        disks.append(panel_disks)
        histories.append(panel_histories)
    fig.suptitle('G policy — fixed held-out analysis cases, no outcome-based selection')
    fig.savefig(output / 'trajectories.png', dpi=200)
    fig.savefig(output / 'trajectories.pdf')
    if animate:
        for panel, (ax, (formation, trace, _)) in enumerate(zip(axes[0], traces)):
            for line in list(ax.lines):
                line.remove()
            histories[panel] = [ax.plot([], [], color=color, lw=1.1)[0] for color in colors]
        maximum = max(trace['position'].shape[0] for _, trace, _ in traces)
        frames = list(range(0, maximum, 4))
        if frames[-1] != maximum - 1:
            frames.append(maximum - 1)
        def update(frame):
            artists = []
            for panel, (formation, trace, result) in enumerate(traces):
                positions = trace['position'][:, 0].numpy()
                index = min(frame, len(positions) - 1)
                for robot in range(5):
                    disks[panel][robot].center = positions[index, robot]
                    histories[panel][robot].set_data(positions[:index + 1, robot, 0], positions[:index + 1, robot, 1])
                    artists.extend((disks[panel][robot], histories[panel][robot]))
                axes[0, panel].set_title(f"{formation}: {index * trace['task']['dt']:.2f} s")
            return artists
        animation = FuncAnimation(fig, update, frames=frames, interval=100, blit=False)
        animation.save(output / 'formation_navigation.gif', writer=PillowWriter(fps=10), dpi=95)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--no-animation', action='store_true')
    args = parser.parse_args()
    torch.set_num_threads(1)
    root = args.root.resolve()
    report = summarize(root, root / 'analysis' / 'report.json', root / 'analysis' / 'summary.png')
    write_tables(root, report)
    fixed_figures(root, animate=not args.no_animation)
    print(root / 'analysis' / 'RESULTS.md')


if __name__ == '__main__':
    main()
