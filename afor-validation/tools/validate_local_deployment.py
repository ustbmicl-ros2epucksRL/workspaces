"""Check G export equality and isolated inference, independent of task score."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import torch
from afor_local.artifacts import file_hash, save_json, source_hash
from afor_local.checkpoints import SCHEMA, VERSION, load_actor
from afor_local.evaluate import evaluate_case
if __package__:
    from .export_local_deployment import actor_arguments
else:
    from export_local_deployment import actor_arguments


class ExportedActor(torch.nn.Module):
    """Run the scripted tensor interface and check every eager actor output."""
    def __init__(self, model, reference):
        super().__init__()
        self.model, self.reference = model, reference
        self.sensors, self.config = reference.sensors, reference.config
        self.max_action_error, self.max_memory_error = 0., 0.

    def initial_state(self, batch_shape=()):
        return self.reference.initial_state(batch_shape)

    def forward(self, obs, state):
        args = [obs[key] for key in ('ego', 'scan', 'neighbours', 'mask')] + [state]
        action, memory = self.model(*args)
        expected_action, expected_memory = self.reference(obs, state)
        torch.testing.assert_close(action, expected_action, rtol=1e-5, atol=1e-6)
        torch.testing.assert_close(memory, expected_memory, rtol=1e-5, atol=1e-6)
        self.max_action_error = max(self.max_action_error, float((action-expected_action).abs().max()))
        self.max_memory_error = max(self.max_memory_error, float((memory-expected_memory).abs().max()))
        return action, memory


def standalone_check(bundle):
    """An isolated interpreter needs only copied bundle code, NumPy and Torch."""
    script = '''
import json, sys
sys.path.insert(0, sys.argv[1])
import numpy as np
import torch
torch.set_num_threads(1)
from local_controller import LocalController, PeerMeasurement
from pathlib import Path
contract = json.loads((Path(sys.argv[1]) / 'contract.json').read_text())
commands = {}
for formation in contract['formations']:
    controller = LocalController(sys.argv[1], formation, slot_id=0)
    command = controller.step([0., 3.], [0., -3.], [0., 0.], np.full(32, 2.),
        [PeerMeasurement(1, [.5, 0.], [0., 0.])], timestamp=0.)
    assert np.isfinite(command.velocity_target_world).all()
    commands[formation] = command.velocity_target_world.tolist()
assert not any(key.startswith('afor_local') for key in sys.modules)
print(json.dumps({'project_package_imported': False, 'velocity_commands': commands}))
'''
    with tempfile.TemporaryDirectory() as folder:
        process = subprocess.run([sys.executable, '-I', '-c', script, str(Path(bundle).resolve())],
                                 cwd=folder, text=True, capture_output=True, check=True)
    return json.loads(process.stdout)


def validate_bundle(bundle, actors, episodes=8, widths=(1.2,), jitters=(0.,), seed=184001, horizon=750):
    bundle = Path(bundle)
    if episodes < 1 or horizon < 1 or not widths or not jitters:
        raise ValueError('Positive episode/horizon budgets and nonempty conditions are required')
    contract = json.loads((bundle / 'contract.json').read_text())
    if contract.get('schema') != SCHEMA or contract.get('version') != VERSION or contract.get('kind') != 'deployment_bundle':
        raise ValueError('Expected a G deployment bundle')
    if set(actors) != set(contract['formations']):
        raise ValueError('Provide one eager source actor for every exported formation')
    report = {'schema': SCHEMA, 'kind': 'deployment_validation', 'source_hash': source_hash(),
              'scope': 'Eager/script equality and isolated inference; task success is descriptive, not a passing criterion.',
              'standalone': standalone_check(bundle), 'models': []}
    for entry in contract['exports']:
        formation = entry['formation']
        source, exported = Path(actors[formation]), bundle / formation / 'actor.ts'
        if file_hash(source) != entry['actor_pt_sha256'] or file_hash(exported) != entry['actor_ts_sha256']:
            raise ValueError('Deployment/source actor identity mismatch')
        reference, dt = load_actor(source)
        if dt != contract['dt_s'] or reference.config.__dict__ != entry['policy']:
            raise ValueError('Deployment/source actor configuration differs')
        actor = ExportedActor(torch.jit.load(str(exported), map_location='cpu').eval(), reference).eval()
        row = {'formation': formation,
               'actor_pt_sha256': entry['actor_pt_sha256'], 'actor_ts_sha256': entry['actor_ts_sha256'], 'cases': []}
        for width in widths:
            for jitter in jitters:
                case, _ = evaluate_case(actor, dt, formation, passage_width=width, jitter=jitter,
                                        seed=seed, episodes=episodes, horizon=horizon)
                row['cases'].append(case)
        row.update(max_action_error=actor.max_action_error, max_memory_error=actor.max_memory_error)
        report['models'].append(row)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle', type=Path, required=True)
    parser.add_argument('--actor', action='append', required=True, metavar='FORMATION=CHECKPOINT')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--episodes', type=int, default=8)
    parser.add_argument('--horizon', type=int, default=750)
    parser.add_argument('--widths', type=float, nargs='+', default=[1.2])
    parser.add_argument('--jitters', type=float, nargs='+', default=[0.])
    parser.add_argument('--seed', type=int, default=184001)
    args = parser.parse_args(argv)
    if args.output.exists():
        raise FileExistsError('Choose a new validation report to preserve evidence')
    torch.set_num_threads(1)
    report = validate_bundle(args.bundle, actor_arguments(args.actor), args.episodes,
                             args.widths, args.jitters, args.seed, args.horizon)
    save_json(args.output, report)
    print(json.dumps({'output': str(args.output), 'export_equivalence': True}))


if __name__ == '__main__':
    main()
