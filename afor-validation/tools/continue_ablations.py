"""Resume the frozen protocol with one fresh process per training job.

A PyTorch interop-thread setting may only be initialized once in a process.
Fresh child processes avoid worker reuse without changing frozen learning code,
RNG, observations, optimizer or recorded research_hash.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor, as_completed
import json
import multiprocessing
import os
from pathlib import Path

for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'
from ablation_support import run_training_job, save_json, research_hash


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('results/ablations'))
    parser.add_argument('--workers',type=int,default=30)
    parser.add_argument('--unstarted-only',action='store_true',help='Do not touch jobs owned by an active runner')
    args=parser.parse_args();root=args.root.resolve()
    protocol=json.loads((root/'protocol.json').read_text())['protocol']
    if protocol['research_hash']!=research_hash():raise ValueError('Frozen learning code mismatch')
    jobs=[]
    for seed in protocol['training_seeds']:
        for form in protocol['formations']:
            for variant in protocol['variants']:
                out=root/variant/form/f'seed_{seed}'
                if (out/'complete.json').exists():continue
                if args.unstarted_only and (out/'config.json').exists():continue
                jobs.append((variant,form,seed,str(root),protocol))
    print(json.dumps({'pending_jobs':len(jobs),'workers':args.workers,'fresh_process_per_job':True}),flush=True)
    finished=[];errors=[]
    with ProcessPoolExecutor(max_workers=args.workers,mp_context=multiprocessing.get_context('spawn'),max_tasks_per_child=1) as pool:
        futures={pool.submit(run_training_job,job):job for job in jobs}
        for future in as_completed(futures):
            try:
                row=future.result();finished.append(row);print(json.dumps(row),flush=True)
            except Exception as exc:
                row={'job':list(futures[future][:3]),'error':repr(exc)};errors.append(row);print(json.dumps(row),flush=True)
            save_json(root/'continuation_status.json',{'scheduled':len(jobs),'completed':finished,'errors':errors})
    if errors:raise RuntimeError(f'{len(errors)} pending jobs failed')


if __name__=='__main__':main()
