# Portable Workspace and SSH Entry Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove host-specific workspace and alternate SSH URL assumptions while keeping the three-project layout, container paths, and platform behavior unchanged.

**Architecture:** `deploy/platform.sh` resolves one absolute workspace parent from its checkout location or `MASAR_WORKSPACE_ROOT`, validates the three sibling projects, and exports the result to Compose. The browser gateway remains the only entry at `127.0.0.1:18090`; remote access uses a one-to-one SSH `LocalForward 18090 127.0.0.1:18090`. The root `README.md` is a short project entry point, while development, architecture, operations, planning, specification, and acceptance documents live under `docs/`.

**Tech Stack:** Bash, Docker Compose, Python/pytest contract tests, Markdown, Make.

## Global Constraints

- Keep the supported host layout as `<workspace>/masar-simulation-platform`, `<workspace>/masar_ws`, and `<workspace>/ROS2swarm_w`.
- Never require or emit `/home/zjc/workspace` as a runtime path.
- Keep container paths `/root/workspace/masar-simulation-platform`, `/root/workspace/masar_ws`, and `/root/workspace/ROS2swarm_w` unchanged.
- Keep the public gateway at `127.0.0.1:18090`; do not add `18089` or use `7897`.
- Keep ROS2 Domain `42`, all eight Compose services, image digests, Run lifecycle, and scenario startup order unchanged.
- Do not modify `masar_ws`, `ROS2swarm_w`, `multi-agent-neat`, or unrelated user changes.
- Do not stop containers or kill host processes during static verification; live checks must use existing platform lifecycle commands.
- Every task writes a failing or contract test before the implementation change, runs the narrow test, then records a focused commit.

---

### Task 1: Normalize the Portable Workspace Root

**Files:**
- Modify: `deploy/platform.sh:1-26,98-131`
- Modify: `tests/unit/test_platform_startup.py:20-190`
- Modify: `tests/unit/test_deploy_contract.py:82-114`
- Modify: `.env.example:1-8`

**Interfaces:**
- Consumes: optional `MASAR_WORKSPACE_ROOT`, the platform checkout path, and the three sibling directories.
- Produces: exported absolute `MASAR_WORKSPACE_ROOT`, a stable `workspace_root` shell variable, and a doctor log line identifying the resolved root.

- [x] **Step 1: Add failing path-contract tests.**

Extend the existing fake Docker harness with two tests:

```python
def test_doctor_normalizes_an_explicit_workspace_symlink(tmp_path: Path) -> None:
    environment, _, _ = _prepare_harness(tmp_path)
    workspace = Path(environment["FAKE_WORKSPACE"])
    alias = tmp_path / "portable-checkout-root"
    alias.symlink_to(workspace, target_is_directory=True)
    environment["MASAR_WORKSPACE_ROOT"] = str(alias)

    result = _run_platform(environment, "doctor")

    assert result.returncode == 0, result.stdout + result.stderr
    assert f"workspace root: {workspace.resolve()}" in result.stdout


def test_invalid_workspace_root_fails_before_docker(tmp_path: Path) -> None:
    environment, docker_log, _ = _prepare_harness(tmp_path)
    environment["MASAR_WORKSPACE_ROOT"] = str(tmp_path / "missing-root")

    result = _run_platform(environment, "doctor")

    assert result.returncode != 0
    assert "workspace root" in result.stderr
    assert docker_log.read_text(encoding="utf-8") == ""
```

Update the Compose contract assertion to retain the environment-driven mount and add a
test that the runtime command still uses the container-internal platform path. Add a short
`.env.example` assertion only if the file's wording changes in this task.

- [x] **Step 2: Run the narrow tests and confirm the new tests fail.**

Run:

```bash
.venv/bin/python -m pytest tests/unit/test_platform_startup.py::test_doctor_normalizes_an_explicit_workspace_symlink tests/unit/test_platform_startup.py::test_invalid_workspace_root_fails_before_docker -q
```

Expected: the symlink test cannot find a normalized-root log line and the invalid-root test
does not fail before the fake Docker checks.

- [x] **Step 3: Implement root resolution and validation.**

In `deploy/platform.sh`, add a `resolve_workspace_root` function before its first call:

```bash
resolve_workspace_root() {
  local candidate="${MASAR_WORKSPACE_ROOT:-$(cd -- "$platform_root/.." && pwd -P)}"
  [[ -d "$candidate" ]] || die "workspace root is not a directory: $candidate"
  workspace_root="$(cd -- "$candidate" && pwd -P)"
  export MASAR_WORKSPACE_ROOT="$workspace_root"
}
```

Call it after `die` is defined and before any command dispatch. Keep the existing
`$workspace_root/masar_ws`, `$workspace_root/ROS2swarm_w`, and
`$workspace_root/masar-simulation-platform` checks in `doctor_impl`. Add:

```bash
log "workspace root: $workspace_root"
```

at the start of `doctor_impl`, so every `init`, `start`, and `doctor` invocation exposes the
resolved host path without embedding a particular user's path in source or docs. Update
`.env.example` to say the override is only needed when the sibling workspace is elsewhere.

- [x] **Step 4: Run the narrow path and Compose tests.**

Run:

```bash
.venv/bin/python -m pytest tests/unit/test_platform_startup.py tests/unit/test_deploy_contract.py -q
docker compose -f deploy/compose.yaml config --quiet
bash -n deploy/platform.sh
```

Expected: all selected tests pass, Compose returns zero, and Bash syntax is valid.

- [x] **Step 5: Commit the portable-path change.**

```bash
git add deploy/platform.sh tests/unit/test_platform_startup.py tests/unit/test_deploy_contract.py .env.example
git commit -m "fix: resolve workspace paths independently of host location"
```

### Task 2: Make the SSH Entry Explicitly One-to-One

**Files:**
- Modify: `deploy/platform.sh:390-470`
- Modify: `tests/unit/test_platform_startup.py:350-390`
- Modify: `docs/OPERATIONS.md:1-245`
- Modify: `docs/superpowers/specs/2026-08-04-workspace-path-and-ssh-entry-design.md` only if wording differs from the approved design

**Interfaces:**
- Consumes: existing `gateway_url=http://127.0.0.1:18090`.
- Produces: a stable startup/status access hint containing `LocalForward 18090 127.0.0.1:18090`.

- [x] **Step 1: Add the failing access-output test.**

Add to the existing fake-harness tests:

```python
def test_start_reports_one_to_one_gateway_and_ssh_entry(tmp_path: Path) -> None:
    environment, _, _ = _prepare_harness(tmp_path)
    state = Path(environment["FAKE_STATE"])
    for marker in ("ros1-image", "ros2-image", "gateway", "compose-running"):
        (state / marker).touch()

    result = _run_platform(environment, "start")

    assert result.returncode == 0, result.stdout + result.stderr
    assert "http://127.0.0.1:18090" in result.stdout
    assert "LocalForward 18090 127.0.0.1:18090" in result.stdout
    assert "18089" not in result.stdout
    assert "7897" not in result.stdout
```

- [x] **Step 2: Run the focused test and confirm it fails.**

```bash
.venv/bin/python -m pytest tests/unit/test_platform_startup.py::test_start_reports_one_to_one_gateway_and_ssh_entry -q
```

Expected: the current idempotent start only prints the old healthy message and no SSH hint.

- [x] **Step 3: Add one access-hint helper and use it on start/status.**

Add a shell helper next to `log`:

```bash
print_access_hint() {
  log "browser: $gateway_url"
  log "remote SSH: LocalForward 18090 127.0.0.1:18090"
}
```

Call it when `command_start` returns from either the already-healthy branch or the final
ready branch, and when `command_status` reports a healthy gateway. Do not add a second
listener, SSH process, or environment variable.

- [x] **Step 4: Replace the alternate SSH URL in the operator manual.**

Keep the canonical block:

```sshconfig
LocalForward 18090 127.0.0.1:18090
```

Delete the `18089` alternative and state that both local and remote browser URLs remain
`http://127.0.0.1:18090`; only the SSH configuration differs. Keep the warning that Agent,
rosbridge, noVNC, Webots, and Docker ports are not individually forwarded.

- [x] **Step 5: Run focused tests and commit.**

```bash
.venv/bin/python -m pytest tests/unit/test_platform_startup.py -q
git diff --check
git add deploy/platform.sh tests/unit/test_platform_startup.py docs/OPERATIONS.md
git commit -m "docs: standardize the 18090 ssh entry"
```

### Task 3: Remove Stale and Duplicate Deployment Documentation

**Files:**
- Move: `PLATFORM_DESIGN.md` -> `docs/PLATFORM_DESIGN.md`
- Move: `SPEC.md` -> `docs/SPEC.md`
- Move: `IMPLEMENTATION_PLAN.md` -> `docs/IMPLEMENTATION_PLAN.md`
- Move: `ACCEPTANCE.md` -> `docs/ACCEPTANCE.md`
- Move: `OPERATIONS.md` -> `docs/OPERATIONS.md`
- Create: `README.md`
- Modify: `docs/PLATFORM_DESIGN.md:37-49,54-60`
- Modify: `docs/SPEC.md:575-600,704-712`
- Modify: `docs/IMPLEMENTATION_PLAN.md:1-25,645-690`
- Modify: `docs/ACCEPTANCE.md:1-10,418-430`
- Modify: `docs/OPERATIONS.md:1-245`
- Modify: `.env.example:1-8`
- Delete: `docs/superpowers/specs/2026-08-03-compose-managed-runtime-design.md`
- Delete: `docs/superpowers/plans/2026-08-03-compose-managed-runtime-implementation.md`
- Create: `tests/unit/test_document_contract.py`

**Interfaces:**
- Consumes: the approved path/SSH design and the current Compose closure in `docs/ACCEPTANCE.md`.
- Produces: one README entry point, one operator manual, current architecture/spec/plan contracts, and historical acceptance evidence without executable superseded instructions.

- [x] **Step 1: Add the failing document contract tests.**

Create `tests/unit/test_document_contract.py`:

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
ACTIVE_DOCS = ("docs/OPERATIONS.md", "docs/SPEC.md", "docs/PLATFORM_DESIGN.md", "docs/IMPLEMENTATION_PLAN.md", "docs/ACCEPTANCE.md", ".env.example")


def test_active_docs_use_the_single_gateway_mapping() -> None:
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    operations = (ROOT / "docs/OPERATIONS.md").read_text(encoding="utf-8")
    assert "LocalForward 18090 127.0.0.1:18090" in operations
    assert "LocalForward 18089" not in operations
    assert "127.0.0.1:18089" not in operations
    assert "http://127.0.0.1:18090" in readme
    assert "docs/OPERATIONS.md" in readme


def test_active_runtime_docs_have_no_host_specific_workspace_path() -> None:
    for relative in ACTIVE_DOCS:
        text = (ROOT / relative).read_text(encoding="utf-8")
        assert "/home/zjc/workspace" not in text, relative


def test_duplicate_compose_documents_are_removed() -> None:
    assert not (ROOT / "docs/superpowers/specs/2026-08-03-compose-managed-runtime-design.md").exists()
    assert not (ROOT / "docs/superpowers/plans/2026-08-03-compose-managed-runtime-implementation.md").exists()
```

The host-path test excludes the new design document because its approved removal rule
explicitly names the historical value as an example; runtime and operator documents must
remain path-neutral.

- [x] **Step 2: Run the document tests and confirm they fail.**

```bash
python3 -m pytest tests/unit/test_document_contract.py -q
```

Expected: the existing `18089` block, host-specific plan/design references, and duplicate
Compose documents cause failures.

- [x] **Step 3: Consolidate active documentation.**

Apply these exact edits:

1. In `docs/PLATFORM_DESIGN.md`, replace the fixed `/home/zjc/workspace/masar-unified-platform`
   exclusion section with a short statement that the deprecated project is outside this
   repository and is never used by Compose; describe the workspace as `<workspace>`.
2. In `docs/SPEC.md`, add the portable sibling-layout and one-to-one `18090` contract near the
   port section, and remove the completed “Spec Review Checklist” asking for approval.
3. In `docs/IMPLEMENTATION_PLAN.md`, replace the fixed implementation-root wording with
   repository-relative paths and add the three tasks from this plan under the active
   Compose closure section. Keep historical tasks only when explicitly marked superseded.
4. In `docs/ACCEPTANCE.md`, state that E4's operator guide and current deployment contract are
   path-neutral and use the one-to-one gateway mapping; leave measured Run IDs and counts
   untouched.
5. In `README.md`, add the project summary and a quick-start section that points to
   `docs/OPERATIONS.md`; do not duplicate the full service table or troubleshooting guide.
6. In `.env.example`, document the automatic parent-directory derivation and the optional
   `MASAR_WORKSPACE_ROOT` override without an example tied to a real user's home path.
7. Update cross-document links and all current-document references to the new `docs/` paths.
8. Delete the two dated Compose documents after confirming their unique active decisions are
   represented in the five current `docs/` documents and the approved design.

- [x] **Step 4: Run stale-reference and Markdown contract scans.**

```bash
python3 -m pytest tests/unit/test_document_contract.py -q
rg -n --hidden --glob '!**/.git/**' --glob '!**/node_modules/**' --glob '!**/dist/**' \
  '/home/zjc/workspace|18089|7897' deploy README.md docs/OPERATIONS.md docs/SPEC.md \
  docs/PLATFORM_DESIGN.md docs/IMPLEMENTATION_PLAN.md docs/ACCEPTANCE.md .env.example
```

Expected: the test passes. The scan returns no active implementation or operator-guide
matches; any permitted historical mention must be outside the scanned active paths or be
removed rather than retained as an instruction.

- [x] **Step 5: Commit documentation consolidation.**

```bash
git add README.md docs/OPERATIONS.md docs/SPEC.md docs/PLATFORM_DESIGN.md docs/IMPLEMENTATION_PLAN.md docs/ACCEPTANCE.md .env.example tests/unit/test_document_contract.py
git add -u docs/superpowers/specs/2026-08-03-compose-managed-runtime-design.md docs/superpowers/plans/2026-08-03-compose-managed-runtime-implementation.md
git commit -m "docs: remove stale deployment and ssh instructions"
```

### Task 4: Run the Full Acceptance Gate and Record Closure

**Files:**
- Modify: `docs/ACCEPTANCE.md` end section with the dated path/SSH closure result
- Modify: `docs/IMPLEMENTATION_PLAN.md` checkboxes for this plan's completed tasks

**Interfaces:**
- Consumes: passing Task 1-3 tests, the current Compose stack, and the existing browser gateway.
- Produces: reproducible acceptance evidence and a clean working tree except for pre-existing user changes.

- [ ] **Step 1: Run the complete static and application test suite.**

```bash
python3 -m pytest -q
npm --prefix frontend run test:unit
npm --prefix frontend run typecheck
npm --prefix frontend run build
ruff check backend tests
python3 -m compileall -q backend agents
bash -n deploy/platform.sh deploy/agent-bootstrap.sh deploy/sidecars/run-ros1-sidecar deploy/sidecars/run-ros2-sidecar
docker compose -f deploy/compose.yaml config --quiet
git diff --check
```

Expected: all existing tests and new path/document contracts pass; frontend and Python
quality gates remain green.

- [ ] **Step 2: Verify the live stack without changing Run state.**

```bash
make docker-doctor
make docker-status
curl --noproxy '*' -fsS http://127.0.0.1:18090/gateway-healthz
curl --noproxy '*' -fsS http://127.0.0.1:18090/api/v1/runs/current
```

Expected: doctor passes, all eight Compose services are healthy, gateway health is `ok`,
and the current Run is `null` or is reported without being stopped by this gate.

- [ ] **Step 3: Record the final acceptance evidence.**

Append a dated entry to `docs/ACCEPTANCE.md` containing the exact test counts, `docker compose
config` result, resolved workspace-root behavior, gateway URL, and the one-to-one SSH snippet.
Mark this plan's path and documentation stages `PASS`; do not claim a new simulation Run if
none was executed.

- [ ] **Step 4: Update the plan checkboxes and review the diff.**

```bash
git diff --stat
git status --short
rg -n '/home/zjc/workspace|18089|7897' deploy README.md docs/OPERATIONS.md docs/SPEC.md \
  docs/PLATFORM_DESIGN.md docs/IMPLEMENTATION_PLAN.md docs/ACCEPTANCE.md .env.example || true
```

Expected: only intentional files from Tasks 1-4 are changed by this work, no active stale
references remain, and unrelated ROS project changes are untouched.

- [ ] **Step 5: Commit the acceptance record.**

```bash
git add docs/ACCEPTANCE.md docs/IMPLEMENTATION_PLAN.md
git commit -m "docs: record portable workspace acceptance"
```
