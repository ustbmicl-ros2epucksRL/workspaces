"""Small, atomic run records with one rolling recovery checkpoint."""
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import torch


def utc_timestamp():
    return datetime.now(timezone.utc).isoformat()


def source_hash():
    root = Path(__file__).resolve().parent
    digest = hashlib.sha256()
    for path in sorted(root.rglob('*.py')):
        digest.update(path.relative_to(root).as_posix().encode())
        digest.update(path.read_bytes())
    return digest.hexdigest()


def file_hash(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def tensor_state_hash(state):
    """Hash named tensor values without pickle/container serialization details."""
    digest = hashlib.sha256()
    for name, value in sorted(state.items()):
        tensor = value.detach().contiguous().cpu()
        digest.update(name.encode())
        digest.update(str(tensor.dtype).encode())
        digest.update(str(tuple(tensor.shape)).encode())
        digest.update(tensor.reshape(-1).view(torch.uint8).numpy().tobytes())
    return digest.hexdigest()


def save_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    try:
        temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def save_torch(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + '.tmp')
    try:
        torch.save(value, temporary)
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


class RunArtifacts:
    """Never deletes unrelated files or historical experiments."""
    def __init__(self, directory, resume=False):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=True)
        self.manifest = self.root / 'run.json'
        self.metrics = self.root / 'metrics.jsonl'
        self.work = self.root / '.work'
        self.checkpoint = self.work / 'latest.pt'
        if resume:
            if not self.manifest.is_file() or not self.checkpoint.is_file():
                raise FileNotFoundError('Resume requires run.json and .work/latest.pt; use --keep-resume to extend completed runs')
            record = json.loads(self.manifest.read_text())
            if record.get('implementation') != 'afor_local_g':
                raise ValueError('This directory is not a G-policy run')
        elif any(self.root.iterdir()):
            raise FileExistsError('Choose an empty run directory; existing evidence is never overwritten')
        self.work.mkdir(exist_ok=True)

    def append_metrics(self, row):
        with self.metrics.open('a') as stream:
            stream.write(json.dumps(row, allow_nan=False) + '\n')

    def rewind_metrics(self, update):
        if self.metrics.exists():
            rows = [json.loads(line) for line in self.metrics.read_text().splitlines() if line.strip()]
            temporary = self.metrics.with_suffix('.jsonl.tmp')
            temporary.write_text(''.join(json.dumps(row, allow_nan=False) + '\n'
                                         for row in rows if row['update'] <= update))
            temporary.replace(self.metrics)

    def finish(self, keep_resume):
        # Only our known disposable checkpoint is removed, after actor export.
        if not keep_resume:
            self.checkpoint.unlink(missing_ok=True)
        for path in self.work.glob('*.tmp'):
            path.unlink()
        if not any(self.work.iterdir()):
            self.work.rmdir()
