# MASAR Unified Simulation Platform Acceptance Ledger

This ledger is the authoritative implementation gate record. A task may move to
`PASS` only after its narrow tests, Spec compliance review, and code-quality
review all pass. The next task must not start before that point.

| Task | Status | Narrow acceptance | Spec review | Quality review | Integrated commits |
|---|---|---|---|---|---|
| 0. Scaffold and Tooling | PASS | `7 passed`; `make compile` passed | APPROVED | APPROVED after fixes | `8f7b3b4`, `a3133b2` |
| 1. Scenario Schema, Profiles and Static Commands | PASS | `25 passed`; full unit suite `32 passed`; schema, compile, Ruff, container dry-runs passed | APPROVED | APPROVED after fixes | `cc45188`, `ea83a16`, `84427db`, `4c79639`, `cb28cd4`, `f2be4c6` |
| 2. SQLite State, Lease and Events | PENDING | - | - | - | - |
| 3. Runtime Agent and Process Ownership | PENDING | - | - | - | - |
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
