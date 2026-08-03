# Compose-Managed Runtime Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move both ROS business runtimes into the platform Compose project and provide one idempotent Make-based deployment flow that works with local cached images or a fresh workspace.

**Architecture:** Keep the existing eight responsibility boundaries: add `ros1-runtime` and `ros2-runtime` to the existing six-service Compose stack, keep ROS Web sidecars separate, and run each Runtime Agent as its business container's main process. A single `deploy/platform.sh` owns checks, image preparation, first-build overlays, legacy migration, lifecycle commands, and health waits; a root `Makefile` is only a command alias layer.

**Tech Stack:** Docker Compose v2, Bash, GNU Make, Docker Hub OCI images, ROS1 Noetic/catkin, ROS2 Humble/colcon, Python 3/pytest/PyYAML, Node 22/Vite, Caddy, Playwright.

## Global Constraints

- Use one `deploy/compose.yaml`; do not create separate source-mount and source-baked modes.
- Mount the computed workspace root at `/root/workspace:rw`; never hardcode `/home/zjc/workspace` in runtime configuration.
- Pin ROS1 to `talent18/masar_ros1_srv-g:v2@sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59`.
- Pin ROS2 to `talent18/masar_ros2_bridge-w:v2@sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c`.
- Support `linux/amd64` with NVIDIA GPU, NVIDIA Container Toolkit, X11, `/dev/dri`, host networking, and host IPC.
- Keep ROS2 Domain `42` for runtime, sidecar, readiness, and scenarios.
- Do not mount the Docker socket into API, frontend, Agent, or sidecar services.
- Preserve one module as one Run; a Run stop closes every Run-owned process without stopping the business runtime container.
- Only whole-platform stop may stop Compose-managed runtime containers.
- Do not alter scenario behavior, UI panels, Robot Web Tools contracts, or direct ROS script defaults in this deployment change.
- The worktree already contains uncommitted platform and ROS changes. Do not stage or commit pre-existing changes; use test and diff checkpoints unless a newly created file can be isolated safely.
- Update `SPEC.md`, `PLATFORM_DESIGN.md`, `IMPLEMENTATION_PLAN.md`, `OPERATIONS.md`, and `ACCEPTANCE.md` in the same stage as the behavior they describe.

---

### Task 1: Replace The External-Container Contract With Failing Deployment Tests

**Files:**
- Modify: `tests/unit/test_deploy_contract.py`
- Replace: `tests/unit/test_platform_startup.py`

**Interfaces:**
- Consumes: existing YAML loader and subprocess-based fake executable helpers.
- Produces: executable contract for Compose service names, immutable image references, Make targets, local-image reuse, overlay bootstrap, legacy migration, start health wait, and graceful stop.

- [ ] **Step 1: Add failing Compose runtime assertions**

Add constants and assertions equivalent to:

```python
ROS1_IMAGE = (
    "talent18/masar_ros1_srv-g:v2@"
    "sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59"
)
ROS2_IMAGE = (
    "talent18/masar_ros2_bridge-w:v2@"
    "sha256:5550e15e39fa60749b67724a991645a714148ad9853d9e87483b5154d2431c6c"
)

def test_compose_owns_both_business_runtimes():
    document = yaml.safe_load((ROOT / "deploy/compose.yaml").read_text())
    ros1 = document["services"]["ros1-runtime"]
    ros2 = document["services"]["ros2-runtime"]
    assert ros1["image"] == ROS1_IMAGE
    assert ros2["image"] == ROS2_IMAGE
    assert ros1["container_name"] == "masar_ros1_srv-g"
    assert ros2["container_name"] == "masar_ros2_bridge-w"
    assert ros1["network_mode"] == ros2["network_mode"] == "host"
    assert ros1["ipc"] == ros2["ipc"] == "host"
    assert ros1["restart"] == ros2["restart"] == "unless-stopped"
    assert ros1["init"] is ros2["init"] is True
```

Also assert GPU access, `/dev/dri`, video group, X11/Xauthority mounts, workspace mount, read-only secret mount, `ROS_DOMAIN_ID=42`, Agent health checks, and absence of the `dev-api` profile.

- [ ] **Step 2: Replace old Agent injection tests with command-contract tests**

Create fake `docker`, `curl`, `uname`, `nvidia-smi`, `xauth`, and `ss` executables. Add these exact test functions:

- `test_init_skips_pull_when_both_local_digests_match`: seed two digest files, run `platform.sh init`, and assert the Docker log contains two `image inspect` calls and no `pull` call.
- `test_init_pulls_only_the_missing_runtime_image`: seed only ROS1, run init, and assert the only pull target is the full ROS2 `tag@digest` reference.
- `test_start_builds_missing_ros_overlays_before_compose_up`: omit both setup files, run start, and assert ROS1 `catkin_make`, ROS2 `colcon build`, then Compose `up` order.
- `test_start_skips_existing_overlays`: create both setup files and assert no ephemeral build command occurs.
- `test_start_migrates_only_exact_legacy_container_names`: seed two unlabeled exact containers plus one unrelated container and assert only the exact two receive stop/remove calls.
- `test_start_refuses_unrelated_port_owner`: report an unknown PID on `18101`, assert exit is nonzero, and assert no stop, remove, build, or up call occurs.
- `test_stop_requests_active_run_stop_before_compose_down`: serve one active Run then null, and assert the stop POST precedes Compose down.
- `test_makefile_exposes_only_supported_deployment_targets`: parse `make -n` output for the six documented targets and assert each delegates to `deploy/platform.sh`.

Use concrete fake-state files for each image, overlay, container label, port owner, and current Run response. The Docker log must prove no invocation contains `exec -d` and no command references `--agents-only`.

- [ ] **Step 3: Run the focused tests and record RED evidence**

Run:

```bash
python3 -m pytest -q tests/unit/test_deploy_contract.py tests/unit/test_platform_startup.py
```

Expected: failures name missing `ros1-runtime`, missing `ros2-runtime`, missing `Makefile`, and missing `deploy/platform.sh`; no unrelated existing test fails.

- [ ] **Step 4: Check the test-only diff**

Run `git diff --check` for the two modified test files. Expected: no whitespace errors.

### Task 2: Add Compose-Managed ROS Runtime Services And Container-Built Frontend

**Files:**
- Modify: `deploy/compose.yaml`
- Modify: `deploy/agent-bootstrap.sh`
- Create: `deploy/caddy/Dockerfile`
- Modify: `tests/unit/test_deploy_contract.py`

**Interfaces:**
- Consumes: `MASAR_WORKSPACE_ROOT`, `MASAR_XAUTHORITY`, `MASAR_HOST_DISPLAY`, `MASAR_ROS2_DOMAIN_ID`, and `.runtime/agent.secret` exported by `deploy/platform.sh`.
- Produces: healthy `ros1-runtime:18101`, `ros2-runtime:18102`, and a Caddy image containing `/srv/frontend/index.html` without a host Node dependency.

- [ ] **Step 1: Add both runtime services to Compose**

Define both services with the fixed image references and this contract:

```yaml
ros1-runtime:
  image: talent18/masar_ros1_srv-g:v2@sha256:3639f637a9be46d61b96bac90bcd0be75a73cfec0a7126915e416d75346a9a59
  container_name: masar_ros1_srv-g
  init: true
  network_mode: host
  ipc: host
  gpus: all
  group_add: [video]
  devices: [/dev/dri:/dev/dri]
  command:
    - /root/workspace/masar-simulation-platform/deploy/agent-bootstrap.sh
    - ros1
    - --secret-path
    - /run/masar-platform/agent.secret
    - --runs-dir
    - /root/workspace/masar-simulation-platform/runs
  restart: unless-stopped
```

The ROS2 service uses its fixed ROS2 image, `agent-bootstrap.sh ros2`, and `ROS_DOMAIN_ID=${MASAR_ROS2_DOMAIN_ID:-42}`. Both inherit loopback proxy exclusions, set `DISPLAY`, `XAUTHORITY`, and `NVIDIA_DRIVER_CAPABILITIES`, mount workspace/Xauthority/X11, and mount the Agent secret read-only.

- [ ] **Step 2: Make Runtime Agent bootstrap describe container-owned lifecycle**

Change the help text from “inside an existing ROS container” to “as the managed service inside a ROS runtime container”. Keep the overlay source order and final `exec python3` behavior unchanged.

- [ ] **Step 3: Make runtime health a dependency**

Add Python `urllib.request` health checks for ports `18101` and `18102`. Add `depends_on` conditions so `api`, `ros1-web`, and `ros2-web` wait for required runtime health without changing loopback endpoints.

- [ ] **Step 4: Build frontend inside the Caddy image**

Create:

```dockerfile
FROM node:22-alpine@sha256:c610fcdfb1d5b4740dd70c284ed3cb16bb857e0f7166196e36a5501df7a3aa32 AS frontend
WORKDIR /app
COPY frontend/package.json frontend/package-lock.json ./
RUN npm ci
COPY frontend/ ./
RUN npm run build

FROM caddy:2.10.2-alpine@sha256:4c6e91c6ed0e2fa03efd5b44747b625fec79bc9cd06ac5235a779726618e530d
COPY deploy/caddy/Caddyfile /etc/caddy/Caddyfile
COPY --from=frontend /app/dist /srv/frontend
```

Change `caddy` from an upstream image plus host `frontend/dist` mounts to this build. Keep gateway routes and health checks unchanged.

- [ ] **Step 5: Run Compose and focused contract tests**

Run:

```bash
MASAR_WORKSPACE_ROOT=/home/zjc/workspace MASAR_XAUTHORITY=/home/zjc/.Xauthority \
  docker compose -f deploy/compose.yaml config --quiet
python3 -m pytest -q tests/unit/test_deploy_contract.py
```

Expected: Compose config and all deployment contract tests pass. Do not start services in this task.

### Task 3: Implement The Single Deployment Controller And Remove Agent Injection

**Files:**
- Create: `Makefile`
- Create: `deploy/platform.sh`
- Delete: `deploy/start-platform.sh`
- Modify: `tests/unit/test_platform_startup.py`
- Modify: `.env.example`

**Interfaces:**
- Consumes: Compose services from Task 2 and current Run endpoints `GET /api/v1/runs/current`, `POST /api/v1/runs/{run_id}/stop`, and `GET /api/v1/capabilities`.
- Produces: `init`, `start`, `stop`, `status`, `logs`, and `doctor` shell commands exposed as `make docker-*` targets.

- [ ] **Step 1: Add thin Make targets**

```make
.PHONY: docker-init docker-start docker-stop docker-status docker-logs docker-doctor

docker-init:
	@./deploy/platform.sh init
docker-start:
	@./deploy/platform.sh start
docker-stop:
	@./deploy/platform.sh stop
docker-status:
	@./deploy/platform.sh status
docker-logs:
	@./deploy/platform.sh logs $(SERVICE)
docker-doctor:
	@./deploy/platform.sh doctor
```

- [ ] **Step 2: Implement shared path, environment, and secret initialization**

Use `set -euo pipefail`, calculate platform/workspace roots from the script, and export:

```bash
MASAR_WORKSPACE_ROOT="$workspace_root"
MASAR_XAUTHORITY="${MASAR_XAUTHORITY:-${XAUTHORITY:-$HOME/.Xauthority}}"
MASAR_HOST_DISPLAY="${MASAR_HOST_DISPLAY:-${DISPLAY:-:0}}"
MASAR_ROS2_DOMAIN_ID="${MASAR_ROS2_DOMAIN_ID:-42}"
```

Create `.runtime/agent.secret` only when absent, require exactly 32 bytes and mode `0600`, and store bootstrap logs under `.runtime/logs`.

- [ ] **Step 3: Implement read-only doctor checks**

Check `docker`, reachable daemon, Compose, `uname -m == x86_64`, `nvidia-smi`, Docker GPU capability, `/dev/dri`, X11 socket, readable Xauthority, workspace layout, and fixed ports. Classify ports owned by the current Compose project or exact legacy containers as migratable; reject unrelated owners without killing them.

- [ ] **Step 4: Implement digest-aware image preparation**

Use `docker image inspect` against each full `tag@digest`. Skip registry calls when the exact local digest exists. Otherwise print the first-download size warning and pull only the missing image. Pull failure exits before compilation or service creation.

- [ ] **Step 5: Implement first-build overlay preparation**

For missing `masar_ws/devel/setup.bash`, use an ephemeral ROS1 runtime and run `source /opt/ros/noetic/setup.bash; cd /root/workspace/masar_ws; catkin_make`. For missing `ROS2swarm_w/install/setup.bash`, run `source /opt/ros/humble/setup.bash; cd /root/workspace/ROS2swarm_w; colcon build`. Save complete output to `.runtime/logs/bootstrap-ros1.log` and `bootstrap-ros2.log`; print concise stage/duration messages. Restore host UID/GID only on generated directories.

- [ ] **Step 6: Implement exact legacy migration and idempotent start**

Return immediately when the gateway and all eight services are healthy. Otherwise refuse migration when the API reports an active Run. Stop the old six-service Compose stack, then stop/remove only the exact legacy container names when their Compose project label is absent. Never remove either image. Start with:

```bash
docker compose -f "$platform_root/deploy/compose.yaml" up -d --build --remove-orphans
```

Wait for capabilities and eight healthy services before printing `http://127.0.0.1:18090`.

- [ ] **Step 7: Implement graceful stop, status, and logs**

Stop an active Run through the API and poll until current Run is null before Compose down. If API is unavailable, warn and continue. Status combines Compose `ps`, gateway health, and current Run. Logs uses Compose logs with an optional service name validated against the eight-service allowlist.

- [ ] **Step 8: Delete obsolete startup and pass focused tests**

Delete `deploy/start-platform.sh` without a compatibility wrapper. Run:

```bash
bash -n deploy/platform.sh deploy/agent-bootstrap.sh
python3 -m pytest -q tests/unit/test_platform_startup.py tests/unit/test_deploy_contract.py
rg -n "docker exec -d|--agents-only|start-platform\.sh" deploy tests backend frontend scenarios
```

Expected: syntax and tests pass; the final search has no active runtime-code matches.

### Task 4: Stage 1 Automated Acceptance And Documentation Checkpoint

**Files:**
- Modify: `SPEC.md`
- Modify: `PLATFORM_DESIGN.md`
- Modify: `IMPLEMENTATION_PLAN.md`
- Modify: `OPERATIONS.md`
- Modify: `ACCEPTANCE.md`

**Interfaces:**
- Consumes: Tasks 1-3 deployment behavior.
- Produces: current architecture and Stage 1 evidence with no instructions for external business containers.

- [ ] **Step 1: Update current deployment contracts in design documents**

Replace active claims that both business containers must pre-exist, remain outside Compose, or require Agent repair. Record immutable v2 images, eight-service topology, workspace mount, Domain 42, first-build behavior, and Make commands. Preserve dated evidence but mark the old deployment as superseded.

- [ ] **Step 2: Rewrite the operations quick start**

Make the default flow:

```bash
git clone git@github.com:ustbmicl-ros2epucksRL/workspaces.git
cd workspaces/masar-simulation-platform
make docker-start
```

Document local access at `http://127.0.0.1:18090` first. Put SSH `LocalForward` in a later remote-server section. Remove host Node/npm, pre-existing container, `--agents-only`, manual Agent kill, and direct Compose profile instructions.

- [ ] **Step 3: Run the complete Stage 1 gate**

```bash
python3 -m pytest -q tests
python3 -m compileall -q backend agents
python3 -m ruff check backend agents tests
npm --prefix frontend run typecheck
npm --prefix frontend run test:unit
npm --prefix frontend run build
MASAR_WORKSPACE_ROOT=/home/zjc/workspace MASAR_XAUTHORITY=/home/zjc/.Xauthority \
  docker compose -f deploy/compose.yaml config --quiet
git diff --check
```

Expected: every command passes. Record exact counts and tool versions in `ACCEPTANCE.md`. Do not proceed to live migration on any failure.

### Task 5: Stage 2 Real Migration And Restart Recovery

**Files:**
- Modify after evidence: `ACCEPTANCE.md`
- Modify after evidence: `IMPLEMENTATION_PLAN.md`

**Interfaces:**
- Consumes: `make docker-start`, `make docker-stop`, Compose runtime health checks.
- Produces: live proof that current cached images are reused, exact old containers migrate, and both Agents recover after runtime restart.

- [ ] **Step 1: Prove the host is safe to migrate**

Query current Run and require `run == null`. Capture IDs/status for unrelated running containers, both v2 local RepoDigests, and both old business container IDs.

- [ ] **Step 2: Start the managed stack**

Run `make docker-start`. Expected output states both image digests match locally and skip pull, existing overlays skip compilation, only exact legacy containers migrate, all eight services become healthy, and the gateway URL is printed.

- [ ] **Step 3: Verify service and capability state**

Run Compose `ps`, query both Agent health endpoints, query `/api/v1/capabilities`, and verify runtime container labels identify the Compose project. Confirm runtime image digests and workspace/X11/GPU/device settings match the contract.

- [ ] **Step 4: Inject ROS1 runtime restart**

With no active Run, run `docker restart masar_ros1_srv-g`. Poll `18101/healthz` until healthy without invoking bootstrap or repair. Verify API capabilities recover and ROS2 runtime ID/start time does not change.

- [ ] **Step 5: Inject ROS2 runtime restart**

Run `docker restart masar_ros2_bridge-w`. Poll `18102/healthz`, require `webots_preflight`, and verify API capabilities recover without repair. Confirm ROS1 runtime does not restart.

- [ ] **Step 6: Verify whole-platform stop isolation**

Run `make docker-stop`. Confirm all eight platform containers are down, Run-only ports are absent, unrelated container IDs/status remain unchanged, and no host process is signaled by the platform. Start again with `make docker-start` and require all services healthy.

- [ ] **Step 7: Record Stage 2 evidence**

Append exact container IDs, restart counts, health timings, image digests, service count, and unrelated-container checks to `ACCEPTANCE.md`. Mark Stage 2 complete only after every assertion passes.

### Task 6: Stage 3 Real Three-Module Regression

**Files:**
- Modify after each Run: `ACCEPTANCE.md`
- Modify after all Runs: `IMPLEMENTATION_PLAN.md`

**Interfaces:**
- Consumes: six-scenario API, existing Stage B/C/D Playwright live assertions, and Compose-managed runtimes.
- Produces: one passing Run for group behavior, Webots exploration, and complete Gazebo rescue, each with clean stop and archive verification.

- [ ] **Step 1: Run group behavior Webots acceptance**

Create `swarm.webots.ros2swarm` with five `e_puck` robots and `aggregation_pattern`, store the returned identifier in `run_id`, and wait for `RUNNING`. Run:

```bash
MASAR_STAGE_B_LIVE=1 \
MASAR_STAGE_B_ROBOTS=5 \
MASAR_STAGE_B_RUN_ID="$run_id" \
MASAR_STAGE_B_SCREENSHOT_DIR=.superpowers/visual-review/compose-stage-b \
npx --prefix frontend playwright test frontend/tests/stage-b-live.spec.ts --project=chromium
```

Stop through the API, require `COMPLETED/USER_STOP`, verify archive checksums, and prove no Webots/Run process or port `18141` remains.

- [ ] **Step 2: Run Webots exploration acceptance**

Create `exploration.webots.robosar`, store its ID in `run_id`, wait for `RUNNING`, and run:

```bash
MASAR_STAGE_C_LIVE=1 \
MASAR_STAGE_C_SIMULATOR=webots \
MASAR_STAGE_C_RUN_ID="$run_id" \
MASAR_STAGE_C_SCREENSHOT_DIR=.superpowers/visual-review/compose-stage-c \
npx --prefix frontend playwright test frontend/tests/stage-c-live.spec.ts --project=chromium
```

Verify process events prove ROS2 `start_robosar_webots.sh` became ready before ROS1 `launch_robosar_w.sh`. Stop and verify reverse cleanup, archive checksums, no Run child, and no `18141` listener.

- [ ] **Step 3: Run complete Gazebo rescue acceptance**

Create `rescue.gazebo.tepper-response`, store its ID in `run_id`, wait for `RUNNING`, and run:

```bash
MASAR_STAGE_D_LIVE=1 \
MASAR_STAGE_D_RUN_ID="$run_id" \
MASAR_STAGE_D_SCREENSHOT_DIR=.superpowers/visual-review/compose-stage-d \
npx --prefix frontend playwright test frontend/tests/stage-d-live.spec.ts --project=chromium
```

Require real allocation, exploration, emergency object, response group, Gazebo, and RViz evidence. Stop, require controlled terminal state, verify all archive checksums and absence of Run processes and Run-only ports.

- [ ] **Step 4: Verify runtimes survived all Run stops**

After all three Runs, require both runtime containers and all six platform services healthy. Verify no runtime restart occurred during ordinary Run stop and no manual Agent repair was used.

- [ ] **Step 5: Record Stage 3 evidence**

Record three Run IDs, screenshots, process order, final states, checksums, cleanup evidence, and runtime IDs/restart counts. Mark Stage 3 complete only after all three pass in sequence.

### Task 7: Stage 4 Fresh-Workspace Bootstrap And Final Documentation Closure

**Files:**
- Modify: `SPEC.md`
- Modify: `PLATFORM_DESIGN.md`
- Modify: `IMPLEMENTATION_PLAN.md`
- Modify: `OPERATIONS.md`
- Modify: `ACCEPTANCE.md`
- Verify: `docs/superpowers/specs/2026-08-03-compose-managed-runtime-design.md`

**Interfaces:**
- Consumes: all prior implementation and live evidence.
- Produces: an unambiguous current user guide and isolated proof that missing overlays build automatically.

- [ ] **Step 1: Exercise missing-overlay bootstrap in an isolated workspace**

Create an isolated checkout under a task-specific temporary directory with `masar_ws/devel` and `ROS2swarm_w/install` absent. Use the cached v2 images and run deployment bootstrap against that workspace. Require both bootstrap logs, successful setup files, host UID/GID ownership of generated directories, and unchanged source ownership. Stop and remove only acceptance containers and the task-specific temporary directory.

- [ ] **Step 2: Make all current documents agree**

Ensure every active instruction uses `make docker-init`, `make docker-start`, `make docker-stop`, `make docker-status`, `make docker-logs`, or `make docker-doctor`. Keep historical external-container evidence explicitly dated and superseded. Document public image digests, AMD64/NVIDIA prerequisites, cached-image skip, first-build logs, direct local access, optional remote SSH forwarding, Run stop versus platform stop, and restart recovery.

- [ ] **Step 3: Run ambiguity and stale-reference scans**

```bash
rg -n "zjc2024/masar_|start-platform\.sh|--agents-only|docker exec -d|dev-api|既有业务容器|预先.*容器" \
  deploy backend frontend tests scenarios OPERATIONS.md SPEC.md PLATFORM_DESIGN.md IMPLEMENTATION_PLAN.md
rg -n "/home/zjc/workspace" deploy Makefile .env.example
```

Expected: no active implementation or current-use instruction retains the old deployment. Dated historical sections may use old terms only when the same paragraph marks them superseded.

- [ ] **Step 4: Run final clean-start and full regression**

Run `make docker-stop`, then `make docker-start`. Require eight healthy services and direct browser access at `http://127.0.0.1:18090`. Run full Python, frontend unit, TypeScript, frontend build, ordinary Chromium, Compose config, shell syntax, Ruff, compileall, image health, and `git diff --check` gates.

- [ ] **Step 5: Finish acceptance documentation**

Record final command outputs, test counts, image digests, service health, isolated bootstrap evidence, browser URL, SSH example, three live Run IDs, archive results, and residual risks. The user guide must start with the local one-command flow and place remote-server forwarding later.
