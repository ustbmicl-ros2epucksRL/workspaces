"""Validate the fixed export without training or adding a control mechanism."""
import hashlib
import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import torch
from afor_local.evaluate import evaluate
from afor_local.policy import load_actor


ROOT = Path(__file__).resolve().parents[1]


class ExportedActor(torch.nn.Module):
    """Flatten the evaluator's team batch to independent robot calls."""
    def __init__(self, model, reference):
        super().__init__()
        self.model = model
        self.sensors = reference.sensors
        self.config = reference.config
        self.register_parameter('device_marker', torch.nn.Parameter(torch.zeros(()), requires_grad=False))
        self.max_action_error = 0.
        self.max_memory_error = 0.
        self.reference = reference

    def initial_state(self, batch_shape=()):
        return torch.zeros(*batch_shape, self.config.hidden)

    def forward(self, obs, state):
        shape = state.shape[:-1]
        action, memory = self.model(
            obs['ego'].reshape(-1, 5), obs['scan'].reshape(-1, 32),
            obs['neighbours'].reshape(-1, obs['neighbours'].shape[-2], 8),
            obs['mask'].reshape(-1, obs['mask'].shape[-1]), state.reshape(-1, 64))
        action, memory = action.reshape(*shape, 2), memory.reshape(*shape, 64)
        expected_action, expected_memory = self.reference(obs, state)
        torch.testing.assert_close(action, expected_action, rtol=1e-5, atol=1e-6)
        torch.testing.assert_close(memory, expected_memory, rtol=1e-5, atol=1e-6)
        self.max_action_error = max(self.max_action_error, float((action-expected_action).abs().max()))
        self.max_memory_error = max(self.max_memory_error, float((memory-expected_memory).abs().max()))
        return action, memory


def standalone_check(bundle):
    # -I removes PYTHONPATH and the repository from the isolated interpreter.
    script = '''
import json, sys
sys.path.insert(0, sys.argv[1])
import numpy as np
import torch
torch.set_num_threads(1)
from local_controller import LocalController, PeerMeasurement
commands = {}
for formation in ('line', 'circle', 'wedge'):
    controller = LocalController(sys.argv[1], formation, 0)
    command = controller.step([0., 3.], [0., -3.], [0., 0.],
        np.full(32, 2.), [PeerMeasurement(1, [.5, 0.], [0., 0.])], timestamp=0.)
    assert np.isfinite(command.velocity_target_world).all()
    commands[formation] = command.velocity_target_world.tolist()
assert not any(key.startswith(('afor_local', 'afor_repaired')) for key in sys.modules)
print(json.dumps({'trainer_or_environment_imported': False, 'velocity_commands': commands}))
'''
    with tempfile.TemporaryDirectory() as folder:
        process = subprocess.run([sys.executable, '-I', '-c', script, str(bundle.resolve())],
                                 cwd=folder, text=True, capture_output=True, check=True)
    return json.loads(process.stdout)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle',type=Path,default=ROOT/'deployment')
    parser.add_argument('--models',type=Path,default=ROOT/'models')
    parser.add_argument('--output',type=Path,default=ROOT/'runs/deployment_validation.json')
    parser.add_argument('--episodes',type=int,default=16)
    args=parser.parse_args()
    if args.episodes<1:
        parser.error('Positive episode count required')
    torch.set_num_threads(1)
    bundle = args.bundle
    contract = json.loads((bundle/'contract.json').read_text())
    destination = args.output
    destination.parent.mkdir(parents=True,exist_ok=True)
    if destination.exists():
        raise FileExistsError('Choose a new output to preserve evidence')
    report = {'protocol': {'widths_m': [1.1, 1.2, 1.6], 'environment_seed': 59002,
                          'episodes_per_formation_width': args.episodes, 'horizon': 750,
                          'weights': 'Fixed recommended seed2 final500; no training',
                          'scope': 'Export regression on original narrowing, not a 3D/hardware test',
                          'filter': 'none', 'independent_swept_oracle': True},
              'standalone': standalone_check(bundle), 'models': []}
    for entry in contract['exports']:
        formation = entry['formation']
        original = args.models/formation/'actor.pt'
        exported = bundle/formation/'actor.ts'
        if (hashlib.sha256(original.read_bytes()).hexdigest() != entry['actor_pt_sha256'] or
                hashlib.sha256(exported.read_bytes()).hexdigest() != entry['actor_ts_sha256']):
            raise RuntimeError('Actor identity mismatch')
        reference, dt = load_actor(original)
        actor = ExportedActor(torch.jit.load(str(exported)).eval(), reference).eval()
        row = {'formation': formation, 'actor_pt_sha256': entry['actor_pt_sha256'],
               'actor_ts_sha256': entry['actor_ts_sha256'], 'cases': []}
        for width in report['protocol']['widths_m']:
            case, _ = evaluate(actor, dt, 'narrowing', formation, args.episodes, 59002, 750, False, width)
            case['passage_width'] = width
            row['cases'].append(case)
            print(json.dumps({key: case[key] for key in (
                'formation', 'passage_width', 'episodes', 'successes', 'collisions', 'timeouts')}), flush=True)
        row['max_action_error_on_rollouts'] = actor.max_action_error
        row['max_memory_error_on_rollouts'] = actor.max_memory_error
        report['models'].append(row)
        destination.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    assert all(case['successes'] == case['episodes'] for row in report['models'] for case in row['cases'])


if __name__ == '__main__':
    main()
