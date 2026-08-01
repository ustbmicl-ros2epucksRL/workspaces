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
import stat
from typing import Any

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


@dataclass(frozen=True, slots=True)
class OpenedArtifact:
    artifact: Artifact
    file_descriptor: int


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
        _atomic_write(path, encoded)
        return self.register(run_id, kind, path)

    def register(self, run_id: str, kind: str, path: str | Path) -> Artifact:
        candidate = self._safe_file(run_id, path, error_type=ValueError)
        digest, size_bytes = _hash_file(candidate)
        created_at = datetime.now(timezone.utc)
        artifact = Artifact(
            artifact_id="sha256:" + digest[:32],
            run_id=run_id,
            kind=kind,
            path=candidate,
            size_bytes=size_bytes,
            checksum="sha256:" + digest,
            created_at=created_at,
        )
        with self.database.transaction(immediate=True) as connection:
            connection.execute(
                """
                INSERT INTO artifacts (artifact_id, run_id, kind, path, size_bytes, checksum, created_at)
                VALUES (?, ?, ?, ?, ?, ?, ?)
                ON CONFLICT(run_id, path) DO UPDATE SET
                    artifact_id = excluded.artifact_id,
                    kind = excluded.kind,
                    size_bytes = excluded.size_bytes,
                    checksum = excluded.checksum,
                    created_at = excluded.created_at
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
                "SELECT * FROM artifacts WHERE run_id = ? ORDER BY created_at, artifact_key",
                (run_id,),
            ).fetchall()
        return tuple(_artifact_from_row(row) for row in rows)

    def get(self, run_id: str, artifact_id: str) -> Artifact:
        with self.database.connect() as connection:
            row = connection.execute(
                "SELECT * FROM artifacts WHERE run_id = ? AND artifact_id = ? ORDER BY artifact_key LIMIT 1",
                (run_id, artifact_id),
            ).fetchone()
        if row is None:
            raise ArtifactNotFound(artifact_id)
        try:
            safe_path = self._safe_file(run_id, row["path"], error_type=ArtifactNotFound)
        except ArtifactNotFound as exc:
            raise ArtifactNotFound(artifact_id) from exc
        artifact = _artifact_from_row(row)
        return Artifact(
            artifact_id=artifact.artifact_id,
            run_id=artifact.run_id,
            kind=artifact.kind,
            path=safe_path,
            size_bytes=artifact.size_bytes,
            checksum=artifact.checksum,
            created_at=artifact.created_at,
        )

    def open(self, run_id: str, artifact_id: str) -> OpenedArtifact:
        with self.database.connect() as connection:
            row = connection.execute(
                "SELECT * FROM artifacts WHERE run_id = ? AND artifact_id = ? ORDER BY artifact_key LIMIT 1",
                (run_id, artifact_id),
            ).fetchone()
        if row is None:
            raise ArtifactNotFound(artifact_id)
        artifact = _artifact_from_row(row)
        run_dir = self.run_dir(run_id)
        lexical = Path(os.path.abspath(artifact.path))
        try:
            relative = lexical.relative_to(run_dir)
        except ValueError as exc:
            raise ArtifactNotFound(artifact_id) from exc
        if not relative.parts:
            raise ArtifactNotFound(artifact_id)

        directory_flags = os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW | os.O_CLOEXEC
        file_flags = os.O_RDONLY | os.O_NOFOLLOW | os.O_CLOEXEC
        descriptors: list[int] = []
        file_descriptor: int | None = None
        try:
            root_descriptor = os.open(self.runs_root, directory_flags)
            descriptors.append(root_descriptor)
            current_descriptor = os.open(run_id, directory_flags, dir_fd=root_descriptor)
            descriptors.append(current_descriptor)
            for component in relative.parts[:-1]:
                current_descriptor = os.open(component, directory_flags, dir_fd=current_descriptor)
                descriptors.append(current_descriptor)
            file_descriptor = os.open(relative.parts[-1], file_flags, dir_fd=current_descriptor)
            info = os.fstat(file_descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_size != artifact.size_bytes:
                raise ArtifactNotFound(artifact_id)
            digest = hashlib.sha256()
            while chunk := os.read(file_descriptor, 1024 * 1024):
                digest.update(chunk)
            os.lseek(file_descriptor, 0, os.SEEK_SET)
            if artifact.checksum != "sha256:" + digest.hexdigest():
                raise ArtifactNotFound(artifact_id)
            return OpenedArtifact(artifact, file_descriptor)
        except ArtifactNotFound:
            if file_descriptor is not None:
                os.close(file_descriptor)
                file_descriptor = None
            raise
        except OSError as exc:
            if file_descriptor is not None:
                os.close(file_descriptor)
                file_descriptor = None
            raise ArtifactNotFound(artifact_id) from exc
        finally:
            for descriptor in reversed(descriptors):
                os.close(descriptor)

    def write_checksums(self, run_id: str) -> Artifact:
        lines = []
        for artifact in self.list(run_id):
            if artifact.path.name == "checksums.sha256":
                continue
            relative = artifact.path.relative_to(self.run_dir(run_id)).as_posix()
            lines.append(f"{artifact.checksum.removeprefix('sha256:')}  {relative}")
        path = self.run_dir(run_id) / "checksums.sha256"
        _atomic_write(path, ("\n".join(lines) + ("\n" if lines else "")).encode("utf-8"))
        return self.register(run_id, "checksums", path)

    def _safe_file(
        self,
        run_id: str,
        path: str | Path,
        *,
        error_type: type[Exception],
    ) -> Path:
        run_dir = self.run_dir(run_id).resolve()
        candidate = Path(path)
        lexical = candidate if candidate.is_absolute() else Path.cwd() / candidate
        lexical = Path(os.path.abspath(lexical))
        try:
            relative = lexical.relative_to(run_dir)
        except ValueError as exc:
            raise error_type("artifact path must remain inside the run directory") from exc
        current = run_dir
        for component in relative.parts:
            current = current / component
            if current.is_symlink():
                raise error_type("artifact paths must not contain symlinks")
        try:
            resolved = lexical.resolve(strict=True)
            resolved.relative_to(run_dir)
        except (FileNotFoundError, ValueError, OSError) as exc:
            raise error_type("artifact path must be an existing run file") from exc
        if not resolved.is_file():
            raise error_type("artifact path must be an existing file")
        return resolved


def _hash_file(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    size_bytes = 0
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
            size_bytes += len(chunk)
    return digest.hexdigest(), size_bytes


def _atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".partial")
    with temporary.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


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


__all__ = ["Artifact", "ArtifactNotFound", "ArtifactStore", "OpenedArtifact"]
