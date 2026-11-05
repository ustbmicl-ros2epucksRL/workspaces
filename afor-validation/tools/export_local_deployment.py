"""Package explicitly selected G actors as a standalone scripted bundle."""
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import shutil
import torch
from afor_local.artifacts import file_hash, source_hash
from afor_local.checkpoints import SCHEMA, VERSION, export_scripted_actor, load_actor
from afor_local.formations import make_formation


ROOT = Path(__file__).resolve().parents[1]
FORMATIONS = ('line', 'circle', 'wedge')


def actor_arguments(values):
    """Parse repeatable FORMATION=CHECKPOINT arguments without model defaults."""
    actors = {}
    for value in values:
        formation, separator, filename = value.partition('=')
        if not separator or formation not in FORMATIONS or not filename or formation in actors:
            raise ValueError('Supply each formation once as --actor line=/path/to/actor.pt')
        actors[formation] = Path(filename)
    if not actors:
        raise ValueError('At least one explicit G actor is required')
    return actors


def example(batch, neighbours, generator):
    mask = torch.rand(batch, neighbours, generator=generator) > .3
    peers = torch.randn(batch, neighbours, 8, generator=generator)
    peers = torch.where(mask[..., None], peers, float('nan'))
    return {'ego': torch.randn(batch, 5, generator=generator),
            'scan': torch.rand(batch, 32, generator=generator),
            'neighbours': peers, 'mask': mask}


def tensor_arguments(actor, observation, state):
    arguments = [observation[key] for key in ('ego', 'scan', 'neighbours', 'mask')] + [state]
    return arguments


def export_bundle(actors, output, radius=.13):
    """Export one actor per selected formation; never choose a checkpoint."""
    if (not actors or any(key not in FORMATIONS for key in actors)
            or not math.isfinite(radius) or radius <= 0):
        raise ValueError('Choose explicit formation actors and a finite positive radius')
    loaded = {formation: (Path(filename).resolve(), *load_actor(filename))
              for formation, filename in actors.items()}
    first = next(iter(loaded.values()))
    sensors, dt = first[1].sensors, first[2]
    if any(actor.sensors != sensors or step != dt for _, actor, step in loaded.values()):
        raise ValueError('Actors in a bundle must share their sensing contract and timestep')
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if any(output.iterdir()):
        raise FileExistsError('Choose an empty output directory; existing bundles are preserved')
    contract = {'schema': SCHEMA, 'version': VERSION, 'kind': 'deployment_bundle',
                'source_hash': source_hash(), 'dt_s': dt, 'radius_m': float(radius),
                'lidar_rays': sensors.lidar_rays, 'lidar_range_m': sensors.lidar_range,
                'neighbour_range_m': sensors.neighbour_range, 'memory_size': 64, 'agents': 5,
                'frame': 'Fixed mission axes, +y along the trained corridor direction; world vectors use ENU metres.',
                'observation': 'Final goal delta, ego velocity/radius, lidar and measured one-hop relations. The network receives no explicit ego slot ID.',
                'action': 'Acceleration per mission axis in [-1,1]; velocity target clips measured_velocity+dt*acceleration per axis.',
                'scope': 'Export fidelity only; the source actor determines task performance.',
                'formations': {}, 'exports': []}
    generator = torch.Generator().manual_seed(59001)
    for formation, (filename, actor, _) in loaded.items():
        folder = output / formation
        folder.mkdir()
        script_path = folder / 'actor.ts'
        export_scripted_actor(actor, script_path)
        scripted = torch.jit.load(str(script_path), map_location='cpu').eval()
        errors = []
        with torch.inference_mode():
            for batch in (1, 5, 16):
                for neighbours in (0, 1, 5):
                    observation = example(batch, neighbours, generator)
                    state = torch.randn(batch, 64, generator=generator)
                    expected = actor(observation, state)
                    actual = scripted(*tensor_arguments(actor, observation, state))
                    for left, right in zip(expected, actual):
                        torch.testing.assert_close(left, right, rtol=1e-5, atol=1e-6)
                        errors.append(float((left-right).abs().max()))
        template = make_formation(formation, 'cpu')
        contract['formations'][formation] = {'template_m': template.template.tolist(),
                                             'edges': template.edges}
        contract['exports'].append({'formation': formation,
                                    'policy': asdict(actor.config), 'sensors': asdict(actor.sensors),
                                    'source_actor': str(filename), 'actor_pt_sha256': file_hash(filename),
                                    'actor_ts_sha256': file_hash(script_path), 'max_export_error': max(errors)})
    shutil.copyfile(ROOT / 'deployment' / 'local_controller.py', output / 'local_controller.py')
    shutil.copyfile(ROOT / 'deployment' / 'README.md', output / 'README.md')
    contract['controller_sha256'] = file_hash(output / 'local_controller.py')
    (output / 'contract.json').write_text(json.dumps(contract, indent=2, allow_nan=False) + '\n')
    return contract


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--actor', action='append', required=True, metavar='FORMATION=CHECKPOINT')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--radius', type=float, default=.13, help='Physical radius used for the source policy task')
    args = parser.parse_args(argv)
    torch.set_num_threads(1)
    actors = actor_arguments(args.actor)
    print(json.dumps(export_bundle(actors, args.output, args.radius)['exports'], indent=2))


if __name__ == '__main__':
    main()
