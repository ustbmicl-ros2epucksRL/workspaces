"""Safe run artifact registration and lookup."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import sqlite3
from typing import Any
import uuid

from .db import Database, datetime_from_text, datetime_to_text


class ArtifactNotFound(LookupError):
    pass


@dataclass(frozen=True, slots=True)
class Artifact:
    artifact_id: str
    run_id: str
    kind: str
    path: Path
    size_bytes: int
    checksum: str
    created_at: datetime

    def as_dict(self) -> dict[str, Any]:
        return {
            "artifact_id": self.artifact_id,
            "run_id": self.run_id,
            "kind": self.kind,
            "filename": self.path.name,
            "size_bytes": self.size_bytes,
            "checksum": self.checksum,
            "created_at": datetime_to_text(self.created_at),
        }


class ArtifactStore:
    def __init__(self, database: Database, runs_root: str | Path) -> None:
        self.database = database
        self.runs_root = Path(runs_root).resolve()

    def run_dir(self, run_id: str) -> Path:
        if not isinstance(run_id, str) or not run_id or "/" in run_id or ".." in run_id:
            raise ValueError("run_id is invalid")
        return self.runs_root / run_id

    def write_json(self, run_id: str, kind: str, filename: str, payload: Mapping[str, Any]) -> Artifact:
        if Path(filename).name != filename or not filename.endswith(".json"):
            raise ValueError("artifact filename must be a plain JSON filename")
        encoded = json.dumps(dict(payload), indent=2, sort_keys=True, ensure_ascii=False).encode("utf-8") + b"\n"
        directory = self.run_dir(run_id)
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / filename
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_bytes(encoded)
        os.replace(temporary, path)
        return self.register(run_id, kind, path)

    def register(self, run_id: str, kind: str, path: str | Path) -> Artifact:
        candidate = Path(path).resolve()
        run_dir = self.run_dir(run_id).resolve()
        try:
            candidate.relative_to(run_dir)
        except ValueError as exc:
            raise ValueError("artifact path must remain inside the run directory") from exc
        if not candidate.is_file():
            raise ValueError("artifact path must be an existing file")
        data = candidate.read_bytes()
        created_at = datetime.now(timezone.utc)
        artifact = Artifact(
            artifact_id="artifact-" + uuid.uuid4().hex,
            run_id=run_id,
            kind=kind,
            path=candidate,
            size_bytes=len(data),
            checksum="sha256:" + hashlib.sha256(data).hexdigest(),
            created_at=created_at,
        )
        with self.database.transaction(immediate=True) as connection:
            existing = connection.execute(
                "SELECT artifact_id FROM artifacts WHERE run_id = ? AND path = ?",
                (run_id, str(candidate)),
            ).fetchone()
            if existing is not None:
                connection.execute("DELETE FROM artifacts WHERE artifact_id = ?", (existing["artifact_id"],))
            connection.execute(
                """
                INSERT INTO artifacts (artifact_id, run_id, kind, path, size_bytes, checksum, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    artifact.artifact_id,
                    run_id,
                    kind,
                    str(candidate),
                    artifact.size_bytes,
                    artifact.checksum,
                    datetime_to_text(created_at),
                ),
            )
        return artifact

    def list(self, run_id: str) -> tuple[Artifact, ...]:
        with self.database.connect() as connection:
            rows = connection.execute(
                "SELECT * FROM artifacts WHERE run_id = ? ORDER BY created_at, artifact_id",
                (run_id,),
            ).fetchall()
        return tuple(_artifact_from_row(row) for row in rows)

    def get(self, run_id: str, artifact_id: str) -> Artifact:
        with self.database.connect() as connection:
            row = connection.execute(
                "SELECT * FROM artifacts WHERE run_id = ? AND artifact_id = ?",
                (run_id, artifact_id),
            ).fetchone()
        if row is None:
            raise ArtifactNotFound(artifact_id)
        artifact = _artifact_from_row(row)
        try:
            artifact.path.resolve().relative_to(self.run_dir(run_id).resolve())
        except ValueError as exc:
            raise ArtifactNotFound(artifact_id) from exc
        if not artifact.path.is_file():
            raise ArtifactNotFound(artifact_id)
        return artifact


def _artifact_from_row(row: sqlite3.Row) -> Artifact:
    return Artifact(
        artifact_id=row["artifact_id"],
        run_id=row["run_id"],
        kind=row["kind"],
        path=Path(row["path"]),
        size_bytes=int(row["size_bytes"] or 0),
        checksum=row["checksum"] or "",
        created_at=datetime_from_text(row["created_at"]),
    )


__all__ = ["Artifact", "ArtifactNotFound", "ArtifactStore"]
