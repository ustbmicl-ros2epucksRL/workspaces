# MASAR Unified Simulation Platform Acceptance Ledger

This ledger is the authoritative implementation gate record. A task may move to
`PASS` only after its narrow tests, Spec compliance review, and code-quality
review all pass. The next task must not start before that point.

| Task | Status | Narrow acceptance | Spec review | Quality review | Integrated commits |
|---|---|---|---|---|---|
| 0. Scaffold and Tooling | PASS | `7 passed`; `make compile` passed | APPROVED | APPROVED after fixes | `8f7b3b4`, `a3133b2` |
| 1. Scenario Schema, Profiles and Static Commands | PASS | `25 passed`; full unit suite `32 passed`; schema, compile, Ruff, container dry-runs passed | APPROVED | APPROVED after fixes | `cc45188`, `ea83a16`, `84427db`, `4c79639`, `cb28cd4`, `f2be4c6` |
| 2. SQLite State, Lease and Events | PASS | `35 passed`; full unit suite `67 passed`; schema, compile, Ruff passed | APPROVED after fixes | APPROVED after fixes | `bc1e0b4`, `bcade1a`, `d553693` |
| 3. Runtime Agent and Process Ownership | PASS | `23 passed`; full suite `90 passed`; Ruff, compileall, diff check and both container py_compile/--help checks passed | APPROVED | APPROVED after Python 3.8 fix | `16b56e6`, `043fc10`, `a9df88a` |
| 4. Orchestrator and REST/WebSocket API | PENDING | - | - | - | - |
| 5. ROS1/ROS2 Web Sidecars and Reverse Proxy | PENDING | - | - | - | - |
| 6. Webots Stream Compatibility | PENDING | - | - | - | - |
| 7. Gazebo/RViz/noVNC Displays | PENDING | - | - | - | - |
| 8. React Workbench | PENDING | - | - | - | - |
| 9. Robot Web Tools Adapters | PENDING | - | - | - | - |
| 10. Artifact Recording and Download | PENDING | - | - | - | - |
| 11. Scenario Integration and Regression | PENDING | - | - | - | - |
| 12. Security and Release Gate | PENDING | - | - | - | - |

## Task 0: Scaffold and Tooling

Date: 2026-07-31

- TDD evidence: configuration test first failed with `ModuleNotFoundError` before implementation.
- Automated acceptance: `python3 -m pytest tests/unit/test_config.py tests/unit/test_main.py -q` returned `7 passed`.
- Compile acceptance: `make compile` returned success without the former missing-directory warning.
- Spec compliance: approved against `IMPLEMENTATION_PLAN.md` Task 0 and the frozen port/config contract.
- Code quality: initial review rejected hard-coded Uvicorn bind settings and empty-path handling. Both issues and two minor findings were fixed; re-review approved with no Critical or Important findings.
- Scope: no files in `ROS2swarm_w`, `masar_ws`, `multi-agent-neat`, or `masar-unified-platform` changed.
- Residual tooling note: Ruff and a real FastAPI smoke test were not run because those dependencies are not installed in the active host Python environment. They remain pinned in `pyproject.toml` and will be exercised by later environment/release tasks.

## Task 1: Scenario Schema, Profiles and Static Commands

Date: 2026-08-01

- TDD evidence: catalog, command-rendering, and profile tests first failed because the corresponding modules and manifests did not exist.
- Automated acceptance: `python3 -m pytest tests/unit/test_catalog.py tests/unit/test_commands.py tests/unit/test_profiles.py -q` returned `25 passed`; `python3 -m pytest tests/unit -q` returned `32 passed`.
- Contract acceptance: exactly six scenario manifests passed Draft 2020-12 JSON Schema validation; the catalog exposes exactly the six approved scenario IDs and seven static command references.
- Tooling acceptance: `make compile` and `PYTHONPATH=/tmp/masar-platform-lint python3 -m ruff check backend tests` both returned success.
- Container acceptance: `run_response_deform_demo.sh --dry-run --shared-master` and `run_response_controller_demo.sh transport --dry-run --shared-master` both returned exit code 0 and reported `ROS_MASTER_URI=<caller-default>`.
- Spec compliance: approved after corrections to defaults, command argument order, shared ROS graph handling, real readiness topics, ROS1 topic types, and Webots stream metadata.
- Code quality: initial review rejected mutable manifest contracts and Schema/Pydantic divergence. Both were fixed; re-review approved with no Critical or Important findings.
- Integrated commits: `cc45188`, `ea83a16`, `84427db`, `4c79639`, `cb28cd4`, and `f2be4c6`.
- Scope: no platform dependency on `masar-unified-platform`, its Agent, or port `18088`; no integration of `multi-agent-neat`; direct-run defaults remain unchanged.

## Task 2: SQLite State, Lease and Events

Date: 2026-08-01

- TDD evidence: the prescribed narrow command first failed collection with three `ModuleNotFoundError` errors because `db.py`, `state_machine.py`, `leases.py`, and `events.py` did not exist. Retention and archive regression tests separately failed before their fixes.
- Automated acceptance: `python3 -m pytest tests/unit/test_state_machine.py tests/unit/test_leases.py tests/unit/test_events.py -q` returned `35 passed`; `python3 -m pytest tests/unit -q` returned `67 passed`.
- Database acceptance: SQLite uses WAL and foreign keys; exactly `runs`, `run_steps`, `leases`, `events`, and `artifacts` are created with primary keys and `run_id` indexes; `(run_id, sequence)` is unique; acquisition uses `BEGIN IMMEDIATE`.
- State and lease acceptance: every approved and rejected public transition is tested; state and event persist atomically; one of two concurrent acquisitions wins; heartbeat is 2 seconds and expiry is 10 seconds; all frozen terminal reason codes are covered.
- Event and archive acceptance: per-run sequences remain monotonic across concurrent writers and after live-row retention; `schema_version` is durable; JSONL writes happen after commit; `prepare_archive()` repairs committed mirror gaps; retention leaves the archive unchanged.
- Spec compliance: initial review found sequence reuse after full live retention and a missing persisted `schema_version`. Both were fixed and covered by regression tests; the final line-by-line re-review passed.
- Code quality: Ruff found an unresolved `sqlite3.Row` annotation import. It was fixed; final Ruff, `make compile`, `compileall`, and `git diff --check` all passed with no remaining Critical or Important findings.
- Integrated commits: `bc1e0b4`, `bcade1a`, and `d553693`.
- Scope: only the seven Task 2 platform files changed; no existing ROS project, abandoned platform, or training project was modified.

## Task 3: Runtime Agent and Process Ownership

Date: 2026-08-01

- TDD evidence: the prescribed Agent/process narrow tests were implemented before the runtime Agent and process manager; the completed narrow suite returned `23 passed`.
- Protocol acceptance: HMAC signatures cover method, target, sequence and body; missing, invalid, stale and reordered requests are rejected; health responses and errors never contain the master secret; the host client retries health/probe only and never retries start.
- Ownership acceptance: every started process receives a new session/PGID, persisted run/process ownership records, stdout/stderr paths and cursors; stop uses owned-PGID SIGINT, SIGTERM, SIGKILL escalation and refuses unverifiable ownership.
- Tooling acceptance: full suite returned `90 passed`; Ruff, `compileall`, and `git diff --check` passed.
- Container acceptance: `masar_ros1_srv-g` Python `3.8.10` and `masar_ros2_bridge-w` Python `3.10.12` both passed `py_compile` for all Agent modules and `main.py --help`. Temporary loopback Agents in both containers passed health and signed probe checks; temporary secrets, processes, ports and run directories were removed.
- Compatibility regression: Ruff had emitted Python 3.10-only parenthesized multi-context `with` syntax. The process launch code now uses nested `with` blocks, and a source guard prevents reintroduction; this was verified with the real ROS1 Python 3.8 compiler.
- Scope: no files in `ROS2swarm_w`, `masar_ws`, `multi-agent-neat`, or abandoned `masar-unified-platform` were modified; no package installation or image rebuild was required.
- Integrated commits: `16b56e6`, `043fc10`, and `a9df88a`.
