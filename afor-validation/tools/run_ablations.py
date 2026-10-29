"""Run the registered local-policy ablations; preserve all seeds and failures."""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
import hashlib
import json
import multiprocessing
import os
from pathlib import Path
import shutil
import sys
import time

for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
from ablation_support import VARIANTS, run_training_job, research_hash, save_json
from afor_local.train import source_hash


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('results/ablations'))
    parser.add_argument('--workers', type=int, default=48)
    parser.add_argument('--updates', type=int, default=500)
    parser.add_argument('--seeds', nargs='+', type=int, default=[2,3,4])
    parser.add_argument('--formations', nargs='+', default=['line','circle','wedge'])
    parser.add_argument('--variants', nargs='+', choices=VARIANTS, default=list(VARIANTS))
    parser.add_argument('--episodes', type=int, default=32)
    args = parser.parse_args()
    root = args.output.resolve(); root.mkdir(parents=True, exist_ok=True)
    if min(args.workers,args.updates,args.episodes)<1: parser.error('Positive counts required')
    protocol = {
        'updates': args.updates, 'training_seeds': args.seeds, 'formations': args.formations,
        'variants': {name:VARIANTS[name] for name in args.variants},
        'training_width_m': 1.2, 'widths_m': [1.1,1.2,1.3,1.4],
        'trajectory_widths_m': [1.1,1.2,1.3,1.4],
        'test_seeds': [61001,61002,61003], 'episodes_per_case': args.episodes,
        'validation_every': 50, 'validation_seed': 41001, 'validation_episodes': 16,
        'checkpoint_selection': 'Fixed final update; no model or seed selection',
        'research_hash': research_hash(), 'production_source_hash': source_hash(),
        'acceptance': 'Production strict swept collision, all five cross opening, assigned goals, final shape, speed and dwell',
        'observations': 'The neighbour control masks the deployed actor input; the training critic and local braking teacher retain identical raw measurements',
        'shared_value': 'Use actor features for the value head; remove independent critic encoder; actor capacity unchanged, total training capacity smaller',
        'primary_metrics': ['team_success_rate','collision_rate','timeout_rate','success_duration_s','mean_formation_error_m','min_clearance_m'],
        'secondary_metrics': ['mean_robot_path_m','rms_command_acceleration_m_s2','rms_command_jerk_m_s3'],
        'uncertainty': 'Training seed is replication unit; report seed mean and sample SD, never treat all episodes as independent trained models',
        'trajectory_selection': 'Fixed test seed 61001, episode 0, training seed 2, widths 1.1/1.2/1.4; no success-based selection',
        'limits': 'Original narrowing geometry only; not arbitrary environments, AFOR superiority or hard safety',
    }
    if (root/'protocol.json').exists():
        old = json.loads((root/'protocol.json').read_text())
        if old['protocol']!=protocol: raise ValueError('Existing protocol differs; use another directory')
    else:
        save_json(root/'protocol.json', {'registered_utc': datetime.now(timezone.utc).isoformat(), 'protocol': protocol})
        snapshot = root/'source'; snapshot.mkdir()
        project = Path(__file__).resolve().parents[1]
        files = list((project/'src/afor_local').glob('*.py')) + [project/'tools'/name for name in ('ablation_support.py','run_ablations.py')]
        manifest = {}
        for file in files:
            target = snapshot/file.relative_to(project)
            target.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(file,target)
            manifest[str(file.relative_to(project))] = hashlib.sha256(file.read_bytes()).hexdigest()
        save_json(snapshot/'manifest.json', manifest)
    jobs=[(v,f,s,str(root),protocol) for s in args.seeds for f in args.formations for v in args.variants]
    completed=[]; errors=[]; begin=time.monotonic()
    print(json.dumps({'jobs':len(jobs),'workers':args.workers,'output':str(root)}), flush=True)
    with ProcessPoolExecutor(max_workers=args.workers, mp_context=multiprocessing.get_context('spawn')) as pool:
        futures={pool.submit(run_training_job,job):job for job in jobs}
        for future in as_completed(futures):
            job=futures[future]
            try:
                result=future.result(); completed.append(result)
                print(json.dumps(result),flush=True)
            except Exception as exc:
                errors.append({'job':list(job[:3]),'error':repr(exc)})
                print(json.dumps(errors[-1]),flush=True)
            save_json(root/'status.json', {'total_jobs':len(jobs),'completed':completed,'errors':errors,
                                           'elapsed_s':time.monotonic()-begin})
    if errors: raise RuntimeError(f'{len(errors)} jobs failed; see status.json; rerun resumes only identical configurations')


if __name__=='__main__': main()
