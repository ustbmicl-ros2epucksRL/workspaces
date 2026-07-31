# MASAR Unified Simulation Platform Implementation Plan

> **Implementation mode:** One Codex agent executes this plan in gated batches. Each batch is implemented, verified, merged into the main workspace, recorded in `ACCEPTANCE.md`, and reported to the user before the next batch starts. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a single-user Web workbench that launches and observes the six approved MASAR simulation scenarios without changing the default behavior of `ROS2swarm_w` or `masar_ws`.

**Architecture:** A host FastAPI control plane owns a SQLite WAL run lease and invokes two standard-library runtime Agents inside the existing ROS containers. Versioned YAML manifests map to static command templates; ROS1/ROS2 Web Tools sidecars provide rosbridge, TF and video; Caddy exposes one loopback origin. A React/TypeScript workbench renders synchronized simulation, RViz, map, sensor, state and log panels.

**Tech Stack:** Python 3.10, FastAPI 0.115.6, Uvicorn 0.34.0, Pydantic 2.10.5, PyYAML 6.0.2, SQLite WAL, Node.js 22.13.1, React 19.0.0, Vite 6.0.7, TypeScript 5.7.2, Playwright 1.49.1, `roslib` 2.1.0, `ros3d` 1.1.0, `ros2d` 0.10.0, Docker Compose and Caddy.

---

## 0. Execution Rules

- The implementation root is `/home/zjc/workspace/masar-simulation-platform`; the user-requested root-level plan location overrides the default `docs/superpowers/plans` location.
- The only existing-project source change allowed by this plan is the default-off Webots `stream`/`port` launch argument and, if required by the actual launch graph, a default-off platform GUI skip argument. Every such change has a before/after dry-run test.
- Do not touch `/home/zjc/workspace/masar-unified-platform`, its Agent, its processes, or port `18088`.
- Do not add `multi-agent-neat` to the platform.
- Do not use `pkill`; every process is owned by a `run_id` and a process group.
- Write tests before implementation in every task. Run the narrow test before the full suite.
- Commit only intentional files for the task. Never stage the whole workspace because the two existing ROS projects may contain unrelated work.
- Do not delegate implementation or review to task agents. Complete one batch locally, report its acceptance result, and wait for user confirmation before starting the next batch.

## 1. File Map and Responsibility

| Path | Responsibility |
|---|---|
| `pyproject.toml` | Python package, test, lint and pinned runtime dependencies |
| `backend/masar_platform/config.py` | Environment and port configuration with loopback defaults |
| `backend/masar_platform/contracts.py` | Pydantic request, response, event and error models |
| `backend/masar_platform/db.py` | SQLite schema, WAL connection and transactions |
| `backend/masar_platform/state_machine.py` | Public run states and transition validation |
| `backend/masar_platform/catalog.py` | Manifest schema loading and static scenario catalog |
| `backend/masar_platform/commands.py` | Seven static command refs and argv rendering |
| `backend/masar_platform/profiles.py` | Data, recording, display and artifact profiles |
| `backend/masar_platform/leases.py` | Single-run lease and Agent ownership records |
| `backend/masar_platform/events.py` | Sequence allocation, JSONL append and broadcaster |
| `backend/masar_platform/agents.py` | HMAC-signed Agent client and probe client |
| `backend/masar_platform/orchestrator.py` | Preflight, dependency start, stop and recovery |
| `backend/masar_platform/artifacts.py` | Manifest, summary, bag index, Range download and checksum |
| `backend/masar_platform/api.py` | FastAPI REST/WebSocket routes |
| `backend/masar_platform/main.py` | Uvicorn entry point and startup/shutdown hooks |
| `agents/runtime_agent/protocol.py` | Agent request/response schema and HMAC verification |
| `agents/runtime_agent/main.py` | Standard-library HTTP Agent executed inside either ROS container |
| `agents/runtime_agent/processes.py` | `setsid`, PGID ownership, logs and bounded signal escalation |
| `scenarios/schema.json` | Strict manifest schema, unknown-field rejection |
| `scenarios/catalog.yaml` | Registration of exactly six public scenario IDs |
| `scenarios/*.yaml` | One complete manifest per approved scenario |
| `deploy/compose.yaml` | Caddy, API support services and sidecars |
| `deploy/ros1-web/Dockerfile` | Noetic rosbridge/TF/video sidecar image |
| `deploy/ros2-web/Dockerfile` | Humble rosbridge/TF/video sidecar image |
| `deploy/caddy/Caddyfile` | Same-origin HTTP, WebSocket and stream routes |
| `deploy/sidecars/*.yaml` | Exact sidecar launch arguments and health checks |
| `deploy/display/display_manager.py` | Xvfb/noVNC Gazebo and RViz display lifecycle |
| `frontend/package.json` | Locked frontend dependencies and scripts |
| `frontend/src/app/` | Router, run state store and top-level workbench |
| `frontend/src/panels/` | Simulation, RViz, map, sensor, mission, health and log panels |
| `frontend/src/ros/` | roslibjs connections, topic registry, TF and video adapters |
| `frontend/src/styles/` | Fixed-height toolbar and responsive CSS grid |
| `tests/unit/` | Pure backend, catalog, state, command and artifact tests |
| `tests/contract/` | API, Agent, event and manifest contract tests |
| `tests/integration/` | Sidecar, ROS graph and process ownership tests |
| `tests/e2e/` | Six scenario smoke tests and failure injection |
| `frontend/tests/` | Playwright layout, panel and reconnect tests |
| `runs/.gitkeep` | Runtime directory marker; run files are ignored |

## 2. Frozen Implementation Decisions

The following decisions close the quality-gate questions and are part of the plan, not choices left to the implementer.

### 2.1 Public states and stop semantics

The public states are `PREFLIGHT`, `STARTING`, `RUNNING`, `STOPPING`, `COMPLETED` and `FAILED`. `IDLE` is the absence of a run, not a database row. `FAILED_NEEDS_INSPECTION` is never emitted as a public state; it maps to `state=FAILED` and `reason_code=NEEDS_INSPECTION`.

- A child that exits with code 0 after all required steps are ready produces `COMPLETED`, `reason_code=NORMAL_EXIT`.
- A child that exits unexpectedly while `RUNNING` produces `FAILED`, `reason_code=UNEXPECTED_PROCESS_EXIT`.
- A user stop produces `COMPLETED`, `reason_code=USER_STOP`, after all owned groups have stopped and archive writing succeeds.
- A readiness timeout or invalid preflight produces `FAILED`, `reason_code=READINESS_TIMEOUT` or `PREFLIGHT_FAILED`.

### 2.2 Runtime event retention versus archive

The live SQLite/event broadcaster retains 30 days or 10,000 events. At run finalization, the complete run event stream is copied to `runs/<run_id>/events.jsonl` before checksum generation. The archive is never truncated by the live retention job.

### 2.3 Agent secret recovery

The host service creates `/run/masar-platform/agent.secret` with mode `0600` at install time. The same secret is mounted read-only into both Agent processes. Each run receives a distinct derived HMAC key using `HMAC(master, run_id)`. On API restart, the master secret is read from this path; if the file is missing or has non-`0600` permissions, the service refuses to start and never sends a cleanup signal.

### 2.4 Sidecar commands

The ROS1 sidecar runs `roslaunch rosbridge_server rosbridge_websocket.launch port:=18111`, `rosrun web_video_server web_video_server _port:=18121`, and the Noetic `tf2_web_republisher` executable on the configured ROS master. The ROS2 sidecar runs `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=18112`, `ros2 run web_video_server web_video_server --ros-args -p port:=18122`, and the Humble `tf2_web_republisher` executable. Each sidecar has a supervisor health endpoint and exits non-zero if its required launch command is unavailable.

### 2.5 Topic profiles

Profiles are explicit, not inferred from arbitrary topic names:

| Profile | Allowed sources and type checks | UI rate cap |
|---|---|---:|
| `swarm_sensor` | `/<robot>/scan` `sensor_msgs/msg/LaserScan`; `/<robot>/odom` `nav_msgs/msg/Odometry`; `/<robot>/cmd_vel` `geometry_msgs/msg/Twist`; optional camera `sensor_msgs/msg/Image`, battery and safety, `/global_info` `std_msgs/msg/String` | 10 Hz |
| `response_demo` | `/tb_i/odom`, `/tb_i/cmd_vel`, `/response_group/status` `robosar_messages/ResponseGroupStatus`, `/response_group/behavior_mode` `std_msgs/String`, `/response_group/task_result` `EmergencyTask` | 10 Hz |
| `exploration` | `/sim_map` `nav_msgs/OccupancyGrid`; `/frontier_filter/filtered_frontiers`; `/task_allocation_image` `sensor_msgs/Image`; `/percent_completed` `std_msgs/Float32`; agent status; emergency objects/events | 10 Hz |
| `exploration_webots` | ROS1 exploration profile is authoritative; ROS2 raw `/<robot>/scan`, `odom`, `cmd_vel` is optional sensor data | 10 Hz |
| `rescue` | Union of `exploration` and `response_demo` with duplicate topic deduplication | 10 Hz |

The recorder profiles use the same topic table with a maximum byte rate of 2 MB/s for sensor profiles and 8 MB/s for rescue. Unknown topics are rejected.

## 3. Task 0: Scaffold and Tooling

**Files:**
- Create: `pyproject.toml`, `.gitignore`, `.env.example`, `Makefile`
- Create: `backend/masar_platform/__init__.py`, `backend/masar_platform/config.py`, `backend/masar_platform/main.py`
- Create: `tests/unit/test_config.py`
- Test: `tests/unit/test_config.py`

- [x] **Step 1: Write the failing configuration tests.** Assert that defaults resolve to `127.0.0.1`, ports `18090/18100/18101/18102/18111/18112/18121/18122/18131/18132/18141`, and `MASAR_PLATFORM_DATA_DIR` resolves under `runs/`.

- [x] **Step 2: Run the narrow test.**

  Run: `python3 -m pytest tests/unit/test_config.py -q`

  Expected: FAIL because the package and configuration object do not exist.

- [x] **Step 3: Add the pinned project metadata and configuration.** `config.py` must expose a frozen `Settings` object with `external_host`, `external_port`, `api_port`, Agent ports, sidecar ports, `data_dir`, `secret_path`, and `event_retention_days=30`, `event_retention_count=10000`.

- [x] **Step 4: Run the narrow test and tool checks.**

  Run: `python3 -m pytest tests/unit/test_config.py -q`  
  Expected: PASS.  
  Run: `python3 -m compileall -q backend agents`  
  Expected: PASS.

- [x] **Step 5: Commit the scaffold.**

  ```bash
  git add masar-simulation-platform/pyproject.toml masar-simulation-platform/.gitignore masar-simulation-platform/.env.example masar-simulation-platform/Makefile masar-simulation-platform/backend masar-simulation-platform/agents masar-simulation-platform/tests/unit/test_config.py
  git commit -m "feat(platform): scaffold control plane"
  ```

**Acceptance:** Passed on 2026-07-31. Narrow tests: `7 passed`; compile target: passed; Spec review: approved; code-quality re-review: approved. Integrated commits: `8f7b3b4`, `a3133b2`. Detailed evidence: [ACCEPTANCE.md](./ACCEPTANCE.md#task-0-scaffold-and-tooling).

## 4. Task 1: Scenario Schema, Profiles and Static Commands

**Files:**
- Create: `scenarios/schema.json`, `scenarios/catalog.yaml`, six scenario YAML files
- Create: `backend/masar_platform/contracts.py`, `catalog.py`, `commands.py`, `profiles.py`
- Create: `tests/unit/test_catalog.py`, `tests/unit/test_commands.py`, `tests/unit/test_profiles.py`

- [x] **Step 1: Write catalog tests.** Assert exactly these scenario IDs: `swarm.webots.ros2swarm`, `control.gazebo.response-deform`, `control.gazebo.response-controller`, `exploration.gazebo.tepper`, `exploration.webots.robosar`, `rescue.gazebo.tepper-response`. Assert unknown fields, unsupported controller mode, `number_robots=0`, and `complete + webots` are rejected.

- [x] **Step 2: Run the tests to capture failure.**

  Run: `python3 -m pytest tests/unit/test_catalog.py tests/unit/test_commands.py tests/unit/test_profiles.py -q`

  Expected: FAIL because no catalog, command registry or profiles exist.

- [x] **Step 3: Write `schema.json` and all six manifests.** Every manifest must include full `parameters`, `steps`, `stop_order`, readiness probes, `data_profile`, `artifact_profile`, `display_profile`, `minimum_free_bytes=2147483648`, and per-step `grace_seconds: {sigint: 10, sigterm: 5, sigkill: 5}`. Do not copy the single example from `SPEC.md` as a seventh partial manifest.

- [x] **Step 4: Implement static command rendering.** `commands.py` must use a mapping such as:

  ```python
  COMMAND_REFS = {
      "ros2.swarm": CommandRef(agent="ros2", executable="ros2", fixed_args=("launch", "launch_webots", "create_enviroment.launch.py")),
      "ros1.response_deform": CommandRef(agent="ros1", executable="./run_response_deform_demo.sh", fixed_args=("--no-rviz", "--no-vglrun")),
      "ros1.response_controller": CommandRef(agent="ros1", executable="./run_response_controller_demo.sh", fixed_args=("--no-rviz", "--no-vglrun")),
      "ros1.exploration_gazebo": CommandRef(agent="ros1", executable="./launch_robosar_g.sh", fixed_args=("--no-rviz", "--no-vglrun")),
      "ros2.exploration_webots": CommandRef(agent="ros2", executable="./start_robosar_webots.sh", fixed_args=("--no-vglrun",)),
      "ros1.exploration_webots": CommandRef(agent="ros1", executable="./launch_robosar_w.sh", fixed_args=("--no-rviz", "--no-vglrun")),
      "ros1.tepper_response": CommandRef(agent="ros1", executable="./run_tepper_response_demo.sh", fixed_args=("--no-rviz", "--no-vglrun")),
  }
  ```

  The renderer validates every substituted value against the manifest before returning argv. It never calls a shell.

- [x] **Step 5: Run catalog and command tests.**

  Run: `python3 -m pytest tests/unit/test_catalog.py tests/unit/test_commands.py tests/unit/test_profiles.py -q`  
  Expected: PASS, including an assertion that no rendered argv contains `18088`, `masar-unified-platform`, `agent.py`, `bash -c`, or shell metacharacters.

- [x] **Step 6: Commit the catalog.**

  ```bash
  git add masar-simulation-platform/scenarios masar-simulation-platform/backend/masar_platform/contracts.py masar-simulation-platform/backend/masar_platform/catalog.py masar-simulation-platform/backend/masar_platform/commands.py masar-simulation-platform/backend/masar_platform/profiles.py masar-simulation-platform/tests/unit/test_catalog.py masar-simulation-platform/tests/unit/test_commands.py masar-simulation-platform/tests/unit/test_profiles.py
  git commit -m "feat(platform): add scenario catalog and command allowlist"
  ```

**Acceptance:** Passed on 2026-08-01. Task-specific tests: `25 passed`; full unit suite: `32 passed`; six manifests passed Draft 2020-12 validation; `make compile` and Ruff passed; both ROS1 control-demo dry-runs preserved the caller's ROS graph with `--shared-master`. Spec review and code-quality re-review approved. Integrated commits: `cc45188`, `ea83a16`, `84427db`, `4c79639`, `cb28cd4`, `f2be4c6`. Detailed evidence: [ACCEPTANCE.md](./ACCEPTANCE.md#task-1-scenario-schema-profiles-and-static-commands).

## 5. Task 2: SQLite State, Lease and Events

**Files:**
- Create: `backend/masar_platform/db.py`, `state_machine.py`, `leases.py`, `events.py`
- Create: `tests/unit/test_state_machine.py`, `tests/unit/test_leases.py`, `tests/unit/test_events.py`

- [x] **Step 1: Write state and concurrency tests.** Cover every transition in `SPEC.md`, duplicate run acquisition, lease heartbeat/expiry, monotonically increasing event sequence, `USER_STOP`, `NORMAL_EXIT`, `UNEXPECTED_PROCESS_EXIT`, and `NEEDS_INSPECTION` reason codes.

- [x] **Step 2: Run the tests and confirm failure.**

  Run: `python3 -m pytest tests/unit/test_state_machine.py tests/unit/test_leases.py tests/unit/test_events.py -q`  
  Expected: FAIL because the SQLite tables and transition functions are absent.

- [x] **Step 3: Implement the five-table SQLite schema.** Use `sqlite3` with `PRAGMA journal_mode=WAL`, foreign keys, `runs`, `run_steps`, `leases`, `events`, and `artifacts`; add indexes on `run_id` and `(run_id, sequence)`. Use `BEGIN IMMEDIATE` for run lease acquisition so two API requests cannot both win.

- [x] **Step 4: Implement state transitions and event append.** Reject invalid transitions with a typed `InvalidTransition` error. Persist state and event in one transaction. Mirror each event to the run-local JSONL file only after the transaction commits.

- [x] **Step 5: Implement retention and archive handoff.** The retention job deletes only live rows older than 30 days or beyond 10000 rows per run; `orchestrator.finalize()` has already copied the full run stream to `runs/<run_id>/events.jsonl` before retention can touch it.

- [x] **Step 6: Run tests and commit.**

  Run: `python3 -m pytest tests/unit/test_state_machine.py tests/unit/test_leases.py tests/unit/test_events.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/backend/masar_platform/db.py masar-simulation-platform/backend/masar_platform/state_machine.py masar-simulation-platform/backend/masar_platform/leases.py masar-simulation-platform/backend/masar_platform/events.py masar-simulation-platform/tests/unit/test_state_machine.py masar-simulation-platform/tests/unit/test_leases.py masar-simulation-platform/tests/unit/test_events.py
  git commit -m "feat(platform): add run state lease and event store"
  ```

**Acceptance:** Passed on 2026-08-01. Task-specific tests: `35 passed`; full unit suite: `67 passed`; SQLite WAL/five-table/foreign-key/schema contract, `make compile`, `compileall`, Ruff, and `git diff --check` passed. Spec and quality findings were fixed and re-verified. Integrated commits: `bc1e0b4`, `bcade1a`, `d553693`. Detailed evidence: [ACCEPTANCE.md](./ACCEPTANCE.md#task-2-sqlite-state-lease-and-events).

## 6. Task 3: Runtime Agent and Process Ownership

**Files:**
- Create: `agents/runtime_agent/protocol.py`, `agents/runtime_agent/processes.py`, `agents/runtime_agent/main.py`
- Create: `backend/masar_platform/agents.py`
- Create: `tests/unit/test_agent_protocol.py`, `tests/unit/test_process_ownership.py`, `tests/contract/test_agent_http.py`
- Create: `deploy/agent-bootstrap.sh`

- [ ] **Step 1: Write Agent protocol tests.** Assert HMAC rejection for missing, stale and reordered sequence headers; accepted response schema; invalid command ref rejection; log cursor behavior; and no response body containing the master secret.

- [ ] **Step 2: Write process ownership tests using a fake command.** Spawn a Python `sleep` child with `start_new_session=True`; assert the Agent records `pid`, `pgid`, `run_id`, command ref and argv; stop it through SIGINT/SIGTERM/SIGKILL; assert a separate same-name process remains alive.

- [ ] **Step 3: Run the tests before implementation.**

  Run: `python3 -m pytest tests/unit/test_agent_protocol.py tests/unit/test_process_ownership.py tests/contract/test_agent_http.py -q`  
  Expected: FAIL because the Agent server and process owner do not exist.

- [ ] **Step 4: Implement the standard-library HTTP Agent.** Expose `GET /healthz`, `POST /v1/runs/{run_id}/start`, `POST /v1/runs/{run_id}/stop`, `GET /v1/runs/{run_id}/probe`, and `GET /v1/runs/{run_id}/logs?after=<cursor>`. The Agent accepts only a signed `CommandRef` ID and validated argv from the control plane, calls `os.setsid`, writes stdout/stderr to the run directory, and never invokes `shell=True`.

- [ ] **Step 5: Implement the host Agent client.** Use `http.client` or `httpx` with a 3 s connect timeout and 10 s read timeout. Send `X-Platform-Run`, `X-Platform-Signature`, and an increasing sequence. Retry only idempotent health/probe calls; never retry start automatically.

- [ ] **Step 6: Test in both existing containers without changing images.**

  ```bash
  docker exec masar_ros1_srv-g python3 /root/workspace/masar-simulation-platform/agents/runtime_agent/main.py --help
  docker exec masar_ros2_bridge-w python3 /root/workspace/masar-simulation-platform/agents/runtime_agent/main.py --help
  ```

  Expected: both commands exit 0 and do not import code from `masar-unified-platform`.

- [ ] **Step 7: Commit the Agent.**

  ```bash
  git add masar-simulation-platform/agents masar-simulation-platform/backend/masar_platform/agents.py masar-simulation-platform/tests/unit/test_agent_protocol.py masar-simulation-platform/tests/unit/test_process_ownership.py masar-simulation-platform/tests/contract/test_agent_http.py masar-simulation-platform/deploy/agent-bootstrap.sh
  git commit -m "feat(platform): add signed runtime agents"
  ```

## 7. Task 4: Orchestrator and REST/WebSocket API

**Files:**
- Create: `backend/masar_platform/orchestrator.py`, `artifacts.py`, `api.py`, `main.py`
- Create: `tests/unit/test_orchestrator.py`
- Create: `tests/contract/test_api.py`, `tests/contract/test_event_ws.py`

- [ ] **Step 1: Write API contract tests.** Cover `GET /api/v1/capabilities`, `GET /scenarios`, `POST /scenarios/{id}/validate`, `POST /runs`, `GET /runs/current`, stop, mission command endpoints, artifacts, 409 active-run, 422 invalid parameters, 503 Agent unavailable, and the exact error body from `SPEC.md:19.5`.

- [ ] **Step 2: Write orchestrator tests with fake Agents.** Assert PREFLIGHT checks in order, dependency order for Webots ROS2 then ROS1, required readiness timeout, reverse stop order, normal/failed exit reasons, and recovery to `FAILED/NEEDS_INSPECTION` when lease ownership cannot be proven.

- [ ] **Step 3: Run tests to verify failure.**

  Run: `python3 -m pytest tests/unit/test_orchestrator.py tests/contract/test_api.py tests/contract/test_event_ws.py -q`  
  Expected: FAIL because routes and orchestrator are absent.

- [ ] **Step 4: Implement preflight and run creation.** Validate catalog schema, parameter allow-list, container health, ports, secret permissions, disk free space (`minimum_free_bytes`), Agent health, and display prerequisites. Acquire the SQLite lease before any Agent start call. Return HTTP 202 with `run_id` only after the run row and resolved manifest are durable.

- [ ] **Step 5: Implement startup, commands and stop.** Start steps by manifest dependency, poll exact topic/type readiness, publish mission commands only through fixed adapter methods, and stop by run-owned PGID. START/STOP/HOME are separate API routes; DEFORM mission HOME is rejected with `COMMAND_NOT_SUPPORTED`, while `/response_group/return_home` remains available.

- [ ] **Step 6: Implement event WebSocket and cursor recovery.** Send status/event/log envelopes, ping every 15 s, close idle clients, and let the client replay missing events through REST before subscribing live.

- [ ] **Step 7: Run contract tests and commit.**

  Run: `python3 -m pytest tests/unit/test_orchestrator.py tests/contract/test_api.py tests/contract/test_event_ws.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/backend/masar_platform/orchestrator.py masar-simulation-platform/backend/masar_platform/artifacts.py masar-simulation-platform/backend/masar_platform/api.py masar-simulation-platform/backend/masar_platform/main.py masar-simulation-platform/tests/unit/test_orchestrator.py masar-simulation-platform/tests/contract/test_api.py masar-simulation-platform/tests/contract/test_event_ws.py
  git commit -m "feat(platform): add run orchestration API"
  ```

## 8. Task 5: ROS1/ROS2 Web Sidecars and Reverse Proxy

**Files:**
- Create: `deploy/ros1-web/Dockerfile`, `deploy/ros2-web/Dockerfile`
- Create: `deploy/sidecars/ros1-supervisor.yaml`, `deploy/sidecars/ros2-supervisor.yaml`
- Create: `deploy/compose.yaml`, `deploy/caddy/Caddyfile`
- Create: `tests/integration/test_sidecar_configs.py`

- [ ] **Step 1: Write configuration tests.** Parse Compose and Caddy files; assert exact loopback port table, ROS1 Noetic package names, ROS2 Humble package names, WebSocket upgrade headers, noVNC/Webots routes, and absence of `18088`.

- [ ] **Step 2: Run configuration tests before implementation.**

  Run: `python3 -m pytest tests/integration/test_sidecar_configs.py -q`  
  Expected: FAIL because deployment files do not exist.

- [ ] **Step 3: Build sidecar images.** Install the apt packages verified in the containers: `ros-noetic-rosbridge-suite`, `ros-noetic-tf2-web-republisher`, `ros-noetic-web-video-server` for ROS1; corresponding `ros-humble-*` packages for ROS2. Pin base image digests in the generated lock record. Do not `apt-get install` into `masar_ros1_srv-g` or `masar_ros2_bridge-w`.

- [ ] **Step 4: Add supervisor commands and health checks.** Launch rosbridge, TF republisher and video server with the ports in `SPEC.md:19.2`. Health checks query the process supervisor and a local TCP socket; readiness remains the Platform API's ROS topic/type probe.

- [ ] **Step 5: Add Caddy routes.** Preserve `/api/`, `/ws/`, `/ros1/`, `/ros2/`, `/video/ros1/`, `/video/ros2/`, `/display/gazebo/`, `/display/rviz/`, and `/webots/` WebSocket upgrades. Reject requests to Agent ports and any path containing `..`.

- [ ] **Step 6: Build and smoke test sidecars.**

  ```bash
  docker compose -f deploy/compose.yaml config
  docker compose -f deploy/compose.yaml build ros1-web ros2-web
  docker compose -f deploy/compose.yaml up -d ros1-web ros2-web caddy
  curl -fsS http://127.0.0.1:18090/api/v1/capabilities
  ```

  Expected: Compose config succeeds, both sidecars report healthy, capabilities returns JSON, and `ss -ltnp` shows no public bind.

- [ ] **Step 7: Commit deployment support.**

  ```bash
  git add masar-simulation-platform/deploy masar-simulation-platform/tests/integration/test_sidecar_configs.py
  git commit -m "feat(platform): add ROS web sidecars and proxy"
  ```

## 9. Task 6: Webots Streaming Compatibility and Existing-Project Guardrails

**Files:**
- Modify: `/home/zjc/workspace/ROS2swarm_w/start_robosar_webots.sh`
- Modify: `/home/zjc/workspace/ROS2swarm_w/src/launch_webots/launch/start_webots.launch.py`
- Modify: `/home/zjc/workspace/ROS2swarm_w/src/launch_webots/launch/create_enviroment.launch.py` only if argument forwarding requires it
- Modify: `/home/zjc/workspace/ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py`
- Modify: `/home/zjc/workspace/ROS2swarm_w/tests/test_webots_start_scripts.py`
- Create: `backend/masar_platform/webots.py`, `tests/integration/test_webots_proxy.py`

- [ ] **Step 1: Capture the pre-change command baseline.** Run `--dry-run` for `start_simulation.sh` and `start_robosar_webots.sh`; save stdout under `tests/fixtures/baseline/` with the current commit SHA and do not edit the fixture manually.

- [ ] **Step 2: Write failing compatibility tests.** Assert the current default launch still has `stream=false` and the existing port, while `WEBOTS_STREAM=true WEBOTS_PORT=18141 ./start_robosar_webots.sh --dry-run --no-vglrun` forwards explicit `webots_stream:=true webots_port:=18141` launch arguments and both the swarm and RoboSAR launch paths pass `stream=True`, `port='18141'` to `WebotsLauncher`.

- [ ] **Step 3: Implement the smallest default-off argument change.** Make `start_robosar_webots.sh` consume the platform-only `WEBOTS_STREAM`/`WEBOTS_PORT` environment metadata, add declared launch arguments to both Webots launch paths, and forward them. Preserve the current defaults and all existing world, mode, GUI and supervisor behavior when those variables are absent.

- [ ] **Step 4: Run existing ROS2 tests and dry-run comparisons.**

  ```bash
  python3 -m pytest /home/zjc/workspace/ROS2swarm_w/tests/test_webots_start_scripts.py -q
  cd /home/zjc/workspace/ROS2swarm_w
  ./start_simulation.sh --dry-run --no-vglrun
  ./start_robosar_webots.sh --dry-run --no-vglrun
  ```

  Expected: existing tests pass; default business argv remains unchanged except for explicit platform arguments.

- [ ] **Step 5: Verify the Webots path.** Start a stream-only world on port 18141, request `/webots/` through Caddy, assert WebSocket upgrade, non-empty canvas pixels in Chromium, and drag/zoom changes the camera state. If path-prefix handling fails, configure Caddy path stripping and repeat; do not expose a second public port.

- [ ] **Step 6: Commit the compatibility change separately from new platform code.**

  ```bash
  git add /home/zjc/workspace/ROS2swarm_w/start_robosar_webots.sh /home/zjc/workspace/ROS2swarm_w/src/launch_webots/launch/start_webots.launch.py /home/zjc/workspace/ROS2swarm_w/src/launch_webots/launch/create_enviroment.launch.py /home/zjc/workspace/ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py /home/zjc/workspace/ROS2swarm_w/tests/test_webots_start_scripts.py masar-simulation-platform/backend/masar_platform/webots.py masar-simulation-platform/tests/integration/test_webots_proxy.py
  git commit -m "feat(ros2swarm): expose default-off Webots stream"
  ```

## 10. Task 7: Gazebo/RViz Displays and noVNC

**Files:**
- Create: `deploy/display/display_manager.py`, `deploy/display/xstartup-gazebo`, `deploy/display/xstartup-rviz`
- Create: `deploy/novnc/Dockerfile`, `deploy/novnc/supervisord.conf`
- Create: `backend/masar_platform/displays.py`
- Create: `tests/integration/test_displays.py`

- [ ] **Step 1: Write display lifecycle tests.** Assert allocation of `:101` for Gazebo and `:102` for RViz, separate PID/PGID records, bounded shutdown, and no reuse of a display owned by an unrelated process.

- [ ] **Step 2: Run tests and verify failure.**

  Run: `python3 -m pytest tests/integration/test_displays.py -q`  
  Expected: FAIL because the display manager does not exist.

- [ ] **Step 3: Implement display manager.** Start Xvfb and openbox in the GUI sidecar; expose each display through an x11vnc/websockify/noVNC process. Record all PIDs under the current `run_id`. Preserve the existing `run_vnc_gpu.sh` environment path for direct native runs; platform displays are only used by platform-owned processes.

- [ ] **Step 4: Implement platform GUI wrappers.** Start Gazebo-bearing scripts with `--no-rviz`, set `DISPLAY=:101`, and start the repository's RViz config separately on `DISPLAY=:102`. Record the exact RViz command and config file in the resolved manifest.

- [ ] **Step 5: Test both panels through Caddy.** Assert independent noVNC WebSocket connections, non-empty canvas screenshots, and mouse focus isolation. A noVNC panel failure must set a panel error and not stop the ROS run unless its display is a required readiness step.

- [ ] **Step 6: Run tests and commit.**

  Run: `python3 -m pytest tests/integration/test_displays.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/deploy/display masar-simulation-platform/deploy/novnc masar-simulation-platform/backend/masar_platform/displays.py masar-simulation-platform/tests/integration/test_displays.py
  git commit -m "feat(platform): add independent Gazebo and RViz displays"
  ```

## 11. Task 8: React Workbench and Responsive Layout

**Files:**
- Create: `frontend/package.json`, `frontend/package-lock.json`, `frontend/tsconfig.json`, `frontend/vite.config.ts`
- Create: `frontend/src/app/App.tsx`, `frontend/src/app/runStore.ts`, `frontend/src/app/api.ts`
- Create: `frontend/src/panels/SimulationPanel.tsx`, `RvizPanel.tsx`, `MapPanel.tsx`, `SensorPanel.tsx`, `HealthPanel.tsx`, `MissionPanel.tsx`, `EventLogPanel.tsx`, `ProcessLogPanel.tsx`
- Create: `frontend/src/styles/layout.css`, `frontend/src/styles/tokens.css`
- Create: `frontend/tests/layout.spec.ts`, `frontend/tests/panel-state.spec.ts`

- [ ] **Step 1: Write Playwright layout tests.** At 1440x900 assert the four required panels are simultaneously visible for exploration/rescue; at 390x844 assert single-column order, toolbar height at most 56 px, no horizontal overflow and no overlapping bounding boxes. At swarm scenarios assert RViz and generic behavior metric labels are absent.

- [ ] **Step 2: Run the browser tests before implementation.**

  Run: `cd frontend && npm ci && npx playwright test tests/layout.spec.ts --project=chromium`  
  Expected: FAIL because the Vite app and panels are absent.

- [ ] **Step 3: Implement the shell and state store.** Use a single `run_id` store, explicit panel registry, CSS grid areas and panel actions `resize`, `maximize`, `restore`, `popout`. Popout uses the existing run store and never calls `POST /runs`.

- [ ] **Step 4: Implement the three module layouts.** Swarm uses simulation, trajectory, sensor, health and logs; exploration adds RViz, map and progress; rescue adds mission timeline and response status. Render no behavior score fields anywhere in the swarm layouts.

- [ ] **Step 5: Implement missing-data states.** `UNAVAILABLE`, `WAITING`, `ONLINE`, `STALE`, `TYPE_MISMATCH` and `DISCONNECTED` must be rendered from API data. The SensorPanel renders `未提供` for an absent camera and never replaces it with zero or sample values.

- [ ] **Step 6: Run Playwright tests and commit.**

  Run: `cd frontend && npm ci && npx playwright test tests/layout.spec.ts tests/panel-state.spec.ts --project=chromium`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/frontend
  git commit -m "feat(frontend): add synchronized simulation workbench"
  ```

## 12. Task 9: Robot Web Tools Adapters

**Files:**
- Create: `frontend/src/ros/RosConnectionManager.ts`, `TopicRegistry.ts`, `TopicHealth.ts`, `TfClient.ts`, `VideoClient.ts`
- Modify: `frontend/src/panels/MapPanel.tsx`, `SensorPanel.tsx`, `HealthPanel.tsx`
- Create: `frontend/tests/ros-adapters.spec.ts`

- [ ] **Step 1: Write adapter tests with a mock rosbridge server.** Cover one shared ROS1 connection, one shared ROS2 connection, topic type rejection, queue length one, 10 Hz render cap, stale transition at the profile threshold, TF frame selection, and releasing the previous video stream within 5 seconds.

- [ ] **Step 2: Run tests and capture failure.**

  Run: `cd frontend && npx vitest run tests/ros-adapters.spec.ts`  
  Expected: FAIL because adapters do not exist.

- [ ] **Step 3: Implement the singleton connection manager.** Use `roslib` 2.1.0 and route all topic subscriptions through `TopicRegistry`. Components cannot create raw WebSockets. `TopicHealth` records observed frequency and last age as transport health only.

- [ ] **Step 4: Implement map and TF adapters.** Wrap ros3djs 1.1.0 and ros2djs 0.10.0 behind `MapRenderer`; request only visible frames through `tf2_web_republisher`; disable PointCloud2 by default.

- [ ] **Step 5: Implement video and sensor adapters.** Build web-video URLs from the same-origin API capability response; unsubscribe and abort the old stream before selecting a new one. Decode only profile-approved LaserScan, Odometry and Twist messages.

- [ ] **Step 6: Run adapter, browser and TypeScript checks.**

  ```bash
  cd frontend
  npx vitest run tests/ros-adapters.spec.ts
  npm run typecheck
  npx playwright test tests/panel-state.spec.ts --project=chromium
  ```

  Expected: all commands PASS.

- [ ] **Step 7: Commit the adapters.**

  ```bash
  git add masar-simulation-platform/frontend/src/ros masar-simulation-platform/frontend/src/panels masar-simulation-platform/frontend/tests
  git commit -m "feat(frontend): add Robot Web Tools adapters"
  ```

## 13. Task 10: Artifact Recording and Download

**Files:**
- Modify: `backend/masar_platform/artifacts.py`, `orchestrator.py`, `api.py`
- Create: `backend/masar_platform/recording.py`
- Create: `tests/unit/test_artifacts.py`, `tests/contract/test_download.py`

- [ ] **Step 1: Write artifact tests.** Assert fixed manifest/summary keys, SHA-256 artifact IDs, GNU checksum format, complete events copied before retention, path traversal rejection, and Range responses (`206`, `Content-Range`, `Accept-Ranges`).

- [ ] **Step 2: Run tests to verify failure.**

  Run: `python3 -m pytest tests/unit/test_artifacts.py tests/contract/test_download.py -q`  
  Expected: FAIL because artifact and recording implementations are absent.

- [ ] **Step 3: Implement finalization.** Write resolved manifest, complete events, process logs, ROS graph snapshot, summary and checksums atomically through `.partial` files and `os.replace`. Keep the original failed-start reason and missing-data list.

- [ ] **Step 4: Implement recording profiles.** Start only the manifest-declared ROS1/ROS2 bag recorder commands; reject arbitrary topic arguments; stop recorders before process groups and add their bags to the artifact index.

- [ ] **Step 5: Implement secure streaming download.** Resolve an `artifact_id` from SQLite, reject path traversal and symlinks, and support single-range streaming with checksum headers.

- [ ] **Step 6: Run tests and commit.**

  Run: `python3 -m pytest tests/unit/test_artifacts.py tests/contract/test_download.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/backend/masar_platform/artifacts.py masar-simulation-platform/backend/masar_platform/orchestrator.py masar-simulation-platform/backend/masar_platform/api.py masar-simulation-platform/backend/masar_platform/recording.py masar-simulation-platform/tests/unit/test_artifacts.py masar-simulation-platform/tests/contract/test_download.py
  git commit -m "feat(platform): archive runs and stream artifacts"
  ```

## 14. Task 11: Scenario Integration and Regression Gates

**Files:**
- Create: `tests/integration/test_scenario_smoke.py`, `tests/integration/test_failure_injection.py`
- Create: `tests/fixtures/ros_graph/`, `tests/fixtures/processes/`
- Create: `docs/OPERATIONS.md`, `docs/COMPATIBILITY_MATRIX.md`

- [ ] **Step 1: Add deterministic fake ROS graph fixtures.** Provide topic/type/last-message fixtures for each profile, including missing camera, stale odom, wrong type and disconnected bridge cases. Keep physical containers out of unit tests.

- [ ] **Step 2: Add six manifest smoke tests.** For each scenario assert preflight, rendered command refs, readiness probes, panel profile, stop order, final archive and no generic swarm score fields.

- [ ] **Step 3: Add failure injection tests.** Cover occupied port, missing Agent, readiness timeout, Agent disconnect, rosbridge disconnect, disk below 2 GB, unexpected process exit and API restart with missing secret.

- [ ] **Step 4: Run the existing-project regression commands.** Run exactly:

  ```bash
  cd /home/zjc/workspace/masar_ws
  ./launch_robosar_g.sh --dry-run --headless --no-vglrun
  ./launch_robosar_w.sh --dry-run --no-vglrun
  ./run_response_deform_demo.sh --dry-run --headless --no-vglrun
  ./run_response_controller_demo.sh --dry-run transport
  ./run_tepper_response_demo.sh --dry-run --headless --no-vglrun
  python3 -m pytest -q src/robosar/robosar_emergency_response/test
  cd /home/zjc/workspace/ROS2swarm_w
  ./start_robosar_webots.sh --dry-run --no-vglrun
  ./start_simulation.sh --dry-run --no-vglrun
  python3 -m pytest -q tests
  ```

  Expected: all existing dry-runs and tests pass with platform parameters unset.

- [ ] **Step 5: Run one physical smoke per scenario.** Start only after preflight passes; verify the required topics, panel non-empty checks, allowed command actions, bounded stop and archive. Use one scenario at a time and record the run IDs in `docs/OPERATIONS.md`.

- [ ] **Step 6: Commit regression evidence and operations docs.**

  ```bash
  git add masar-simulation-platform/tests/integration masar-simulation-platform/tests/fixtures masar-simulation-platform/docs
  git commit -m "test(platform): add scenario and compatibility gates"
  ```

## 15. Task 12: Security, Readability and Release Gate

**Files:**
- Modify: `deploy/caddy/Caddyfile`, `backend/masar_platform/api.py`, `frontend/src/app/api.ts`
- Create: `tests/contract/test_security.py`, `docs/RELEASE_CHECKLIST.md`

- [ ] **Step 1: Write security tests.** Assert loopback-only listeners, same-origin POST/CSRF enforcement, arbitrary shell/topic rejection, artifact traversal rejection, HMAC replay rejection, no Docker socket mount and no old-project/18088 references in runtime files.

- [ ] **Step 2: Run all local checks.**

  ```bash
  python3 -m pytest -q tests/unit tests/contract tests/integration
  cd frontend && npm run typecheck && npx vitest run && npx playwright test --project=chromium
  cd /home/zjc/workspace
  git diff --check
  ```

  Expected: all checks PASS; failures block release.

- [ ] **Step 3: Run a clean-start check.** Stop only the platform Compose stack and host service, confirm the two existing containers and any unrelated same-name process remain running, then start platform services and verify `GET /api/v1/capabilities`.

- [ ] **Step 4: Generate release checklist.** Record exact image digests, lockfile hashes, ROS package versions, scenario catalog SHA-256, port table, SSH forwarding command and the seven original dry-run outputs.

- [ ] **Step 5: Commit release evidence.**

  ```bash
  git add masar-simulation-platform/tests/contract/test_security.py masar-simulation-platform/docs/RELEASE_CHECKLIST.md masar-simulation-platform/deploy/caddy/Caddyfile masar-simulation-platform/backend/masar_platform/api.py masar-simulation-platform/frontend/src/app/api.ts
  git commit -m "chore(platform): complete release and security gates"
  ```

## 16. Dependency Graph and Sequencing

```text
Task 0 scaffold
  └─> Task 1 catalog/profiles/commands
       ├─> Task 2 state/lease/events
       │    └─> Task 4 orchestrator/API
       │         ├─> Task 10 artifacts
       │         └─> Task 11 scenario integration
       └─> Task 3 runtime Agent
            └─> Task 4 orchestrator/API

Task 5 sidecars/proxy ─┬─> Task 6 Webots stream
                       ├─> Task 7 Gazebo/RViz displays
                       └─> Task 9 Robot Web Tools adapters

Task 8 frontend shell ──> Task 9 adapters ──> Task 11 E2E

Task 10 + Task 11 ──> Task 12 release gate
```

Task 1 precedes all run work because no process may start without a validated scenario. Task 2 and Task 3 can proceed independently after the scaffold, but the API cannot be exposed until both ownership and persistence contracts pass. Webots compatibility is intentionally separate from sidecar deployment because it changes one existing launch file and needs its own regression gate.

## 17. Acceptance Coverage Matrix

| Spec acceptance criteria | Plan tasks |
|---|---|
| 1-6 single run, dependency start, stop ownership, allow-list | 1, 2, 3, 4, 11, 12 |
| 7-14 synchronized UI, missing data, stale, reconnect | 8, 9, 11 |
| 15-19 Webots/noVNC/map/video/backpressure | 5, 6, 7, 9, 11 |
| 20-25 compatibility, Qt retention, loopback and security | 6, 7, 11, 12 |
| 26-28 archive, bag download and no fabricated swarm scores | 10, 11, 12 |

## 18. Definition of Done

1. All tasks 0 through 12 have passing narrow tests and the full regression commands pass.
2. The six approved scenario IDs are the only public catalog entries.
3. A second run is rejected while any first run is active.
4. Every start, stop, failed readiness and unexpected process exit produces a complete archive.
5. Webots, Gazebo and RViz visuals are non-empty through the single SSH-forwarded origin.
6. Group behavior views show only real sensor/topic data and never fabricated generic behavior scores.
7. Direct original commands retain their Qt/default behavior when platform parameters are unset.
8. The release checklist contains image digests, lockfile hashes, port checks and all acceptance evidence.
