"""Reliable entry point for the frozen ablation protocol.

Use --output results/ablations --workers 54. One fresh process per job avoids
PyTorch interop-thread reinitialization when a worker would otherwise be reused.
The learning implementation and source identity remain the registered version.
"""
from concurrent.futures import ProcessPoolExecutor
from functools import partial
import run_ablations

if __name__=='__main__':
    run_ablations.ProcessPoolExecutor=partial(ProcessPoolExecutor,max_tasks_per_child=1)
    run_ablations.main()
