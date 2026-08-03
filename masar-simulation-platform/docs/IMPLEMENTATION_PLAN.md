# MASAR Unified Simulation Platform Implementation Plan

> **Implementation mode:** One Codex agent executes this plan in gated batches. Each batch is implemented, verified, merged into the main workspace, recorded in `ACCEPTANCE.md`, and reported to the user before the next batch starts. Steps use checkbox (`- [ ]`) syntax for tracking.

> **Current execution authority (2026-08-03):** Section 20 supersedes the old
> external-container deployment steps in earlier completed task history. Current
> startup and operations are defined only by `docs/OPERATIONS.md` and
> `deploy/platform.sh`.

**Goal:** Build a single-user Web workbench that launches and observes the six approved MASAR simulation scenarios without changing the default behavior of `ROS2swarm_w` or `masar_ws`.

**Architecture:** One Compose project owns two ROS business runtimes and six platform services. Each runtime Agent is its container's main process. Versioned YAML manifests map to static command templates; ROS1/ROS2 Web Tools sidecars provide rosbridge, TF and video; Webots/Gazebo/RViz use independent noVNC desktops behind one Caddy loopback origin. A React/TypeScript workbench renders synchronized simulation, trajectory, mapping/task, sensor, state and log panels.

**Tech Stack:** Python 3.10, FastAPI 0.115.6, Uvicorn 0.34.0, Pydantic 2.10.5, PyYAML 6.0.2, SQLite WAL, Node.js 22.13.1, React 19.0.0, Vite 6.4.3, TypeScript 5.7.2, Playwright 1.62.1, `roslib` 2.1.0, Docker Compose and Caddy. The old ROS2D/EaselJS and ros3djs chains are not production dependencies.

---

## 0. Execution Rules

- Commands run from the `masar-simulation-platform` checkout. Its host absolute path is not part of the implementation contract.
- Existing-project changes must remain platform-scoped or preserve direct-run defaults. The approved scope includes Webots display/VirtualGL arguments, controller timestep alignment, explicit bridge prerequisite handling, the new discussed-dispersion behavior, and default-off platform GUI skip arguments. Every change has a before/after dry-run or focused regression test.
- The deprecated platform, its Agent, processes and old entry port are outside this implementation scope.
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
| `deploy/compose.yaml` | Eight-service runtime, API, sidecar, display and gateway stack |
| `deploy/platform.sh` | Prerequisite checks, first builds, migration and lifecycle commands |
| `deploy/ros1-web/Dockerfile` | Noetic rosbridge/TF/video sidecar image |
| `deploy/ros2-web/Dockerfile` | Humble rosbridge/TF/video sidecar image |
| `deploy/caddy/Caddyfile` | Same-origin HTTP, WebSocket and stream routes |
| `deploy/sidecars/*-supervisord.conf` | Exact sidecar process supervision and health behavior |
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

`deploy/platform.sh` creates `.runtime/agent.secret` with 32 bytes and mode `0600`. Compose mounts the same file read-only at `/run/masar-platform/agent.secret` in the API and both Runtime Agents. Each run receives a distinct derived HMAC key using `HMAC(master, run_id)`. If the file is missing or has invalid permissions, startup fails before any cleanup signal is sent.

### 2.4 Sidecar commands

The ROS1 sidecar runs `roslaunch rosbridge_server rosbridge_websocket.launch port:=18111`, `rosrun web_video_server web_video_server _port:=18121`, and the Noetic `tf2_web_republisher` executable on the configured ROS master. The ROS2 sidecar runs `ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=18112`, `ros2 run web_video_server web_video_server --ros-args -p port:=18122`, and the Humble `tf2_web_republisher` executable. Each sidecar has a supervisor health endpoint and exits non-zero if its required launch command is unavailable.

### 2.5 Topic profiles

Profiles are explicit, not inferred from arbitrary topic names:

| Profile | Allowed sources and type checks | UI rate cap |
|---|---|---:|
| `swarm_sensor` | `/<robot>/scan` `sensor_msgs/msg/LaserScan`; `/<robot>/odom` `nav_msgs/msg/Odometry`; `/<robot>/cmd_vel` `geometry_msgs/msg/Twist`; optional camera `sensor_msgs/msg/Image`, battery and safety, `/global_info` `std_msgs/msg/String` | 10 Hz |
| `response_demo` | `/gazebo/model_states` `gazebo_msgs/ModelStates`, `/tb_i/odom`, `/tb_i/cmd_vel`, `/response_group/status` `robosar_messages/ResponseGroupStatus`, `/response_group/behavior_mode` `std_msgs/String`, `/response_group/task_result` `EmergencyTask` | 10 Hz |
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

- [x] **Step 1: Write Agent protocol tests.** Assert HMAC rejection for missing, stale and reordered sequence headers; accepted response schema; invalid command ref rejection; log cursor behavior; and no response body containing the master secret.

- [x] **Step 2: Write process ownership tests using a fake command.** Spawn a Python `sleep` child with `start_new_session=True`; assert the Agent records `pid`, `pgid`, `run_id`, command ref and argv; stop it through SIGINT/SIGTERM/SIGKILL; assert a separate same-name process remains alive.

- [x] **Step 3: Run the tests before implementation.**

  Run: `python3 -m pytest tests/unit/test_agent_protocol.py tests/unit/test_process_ownership.py tests/contract/test_agent_http.py -q`  
  Expected: FAIL because the Agent server and process owner do not exist.

- [x] **Step 4: Implement the standard-library HTTP Agent.** Expose `GET /healthz`, `POST /v1/runs/{run_id}/start`, `POST /v1/runs/{run_id}/stop`, `GET /v1/runs/{run_id}/probe`, and `GET /v1/runs/{run_id}/logs?after=<cursor>`. The Agent accepts only a signed `CommandRef` ID and validated argv from the control plane, calls `os.setsid`, writes stdout/stderr to the run directory, and never invokes `shell=True`.

- [x] **Step 5: Implement the host Agent client.** Use `http.client` or `httpx` with a 3 s connect timeout and 10 s read timeout. Send `X-Platform-Run`, `X-Platform-Signature`, and an increasing sequence. Retry only idempotent health/probe calls; never retry start automatically.

- [x] **Step 6: Test in both existing containers without changing images.**

  ```bash
  docker exec masar_ros1_srv-g python3 /root/workspace/masar-simulation-platform/agents/runtime_agent/main.py --help
  docker exec masar_ros2_bridge-w python3 /root/workspace/masar-simulation-platform/agents/runtime_agent/main.py --help
  ```

  Expected: both commands exit 0 and do not import code from `masar-unified-platform`.

- [x] **Step 7: Commit the Agent.**

  ```bash
  git add masar-simulation-platform/agents masar-simulation-platform/backend/masar_platform/agents.py masar-simulation-platform/tests/unit/test_agent_protocol.py masar-simulation-platform/tests/unit/test_process_ownership.py masar-simulation-platform/tests/contract/test_agent_http.py masar-simulation-platform/deploy/agent-bootstrap.sh
  git commit -m "feat(platform): add signed runtime agents"
  ```

## 6.5. Task 3.5: Integration Contracts

**Files:**
- Create: `agents/runtime_agent/probes.py`, `tests/unit/test_agent_probes.py`
- Create: `deploy/compose.yaml`, `deploy/api/Dockerfile`, `.dockerignore`
- Modify: scenario schema/manifests, command environment, Runtime Agent, ROS1 launch scripts
- Create: `tests/unit/test_deploy_contract.py`, `tests/unit/test_platform_script_contract.py`

- [x] **Step 1: Freeze per-scenario ROS graph identity.** Require `runtime.ros1_master_uri` and `runtime.ros2_domain_id`; inject them into every platform invocation and readiness request. All platform scenarios, the ROS2 Web sidecar and standalone ROS2swarm use domain `42`.

- [x] **Step 2: Make readiness exact and executable.** Require `message_type` for every ROS probe, allow an explicit ROS2 QoS profile, and implement signed bounded Agent probes for loopback TCP/HTTP, workspace files, ROS1 topics and ROS2 topics. Topic readiness requires exact type and a newly observed message.

- [x] **Step 3: Restore real process-group ownership.** Set `MASAR_PLATFORM_MODE=1` for every Agent child. In that mode existing ROS1 scripts do not create nested sessions, launch the original Qt GUI, or execute native global cleanup; native direct-run behavior remains unchanged.

- [x] **Step 4: Freeze localhost proxy bypass in every runtime layer.** Merge `127.0.0.1,localhost,::1` into Runtime Agent and child `NO_PROXY/no_proxy`. Create the API Compose service with the same environment anchor; every future Compose service is required by test to inherit both variables.

- [x] **Step 5: Run host and real-container acceptance.** The full suite returns `103 passed`; Ruff, compileall, Compose config/build/health and diff checks pass. Signed live ROS1 and ROS2 domain-42 topic probes return `READY`; all platform-managed ROS1 script dry-runs pass and temporary resources are removed.

## 7. Task 4: Orchestrator and REST/WebSocket API

**Files:**
- Create: `backend/masar_platform/orchestrator.py`, `artifacts.py`, `api.py`, `main.py`
- Create: `tests/unit/test_orchestrator.py`
- Create: `tests/contract/test_api.py`, `tests/contract/test_event_ws.py`

- [x] **Step 1: Write API contract tests.** Cover `GET /api/v1/capabilities`, `GET /scenarios`, `POST /scenarios/{id}/validate`, `POST /runs`, `GET /runs/current`, stop, mission command endpoints, artifacts, 409 active-run, 422 invalid parameters, 503 Agent unavailable, and the exact error body from `SPEC.md:19.5`.

- [x] **Step 2: Write orchestrator tests with fake Agents.** Assert PREFLIGHT checks in order, dependency order for Webots ROS2 then ROS1, required readiness timeout, reverse stop order, normal/failed exit reasons, and recovery to `FAILED/NEEDS_INSPECTION` when lease ownership cannot be proven.

- [x] **Step 3: Run tests to verify failure.**

  Run: `python3 -m pytest tests/unit/test_orchestrator.py tests/contract/test_api.py tests/contract/test_event_ws.py -q`  
  Expected: FAIL because routes and orchestrator are absent.

- [x] **Step 4: Implement preflight and run creation.** Validate catalog schema, parameter allow-list, container health, ports, secret permissions, disk free space (`minimum_free_bytes`), Agent health, and display prerequisites. Acquire the SQLite lease before any Agent start call. Return HTTP 202 with `run_id` only after the run row and resolved manifest are durable.

- [x] **Step 5: Implement startup, commands and stop.** Start steps by manifest dependency, poll exact topic/type readiness, publish mission commands only through fixed adapter methods, and stop by run-owned PGID. START/STOP/HOME are separate API routes; DEFORM mission HOME is rejected with `COMMAND_NOT_SUPPORTED`, while `/response_group/return_home` remains available.

- [x] **Step 6: Implement event WebSocket and cursor recovery.** Send status/event/log envelopes, ping every 15 s, close idle clients, and let the client replay missing events through REST before subscribing live.

- [x] **Step 7: Run contract tests.** The narrow suite returns `19 passed`; the full suite returns `125 passed`; host and Compose API smoke checks plus real ROS1/ROS2 fixed-command publication pass. Commit remains a user-controlled action.

  Run: `python3 -m pytest tests/unit/test_orchestrator.py tests/contract/test_api.py tests/contract/test_event_ws.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/backend/masar_platform/orchestrator.py masar-simulation-platform/backend/masar_platform/artifacts.py masar-simulation-platform/backend/masar_platform/api.py masar-simulation-platform/backend/masar_platform/main.py masar-simulation-platform/tests/unit/test_orchestrator.py masar-simulation-platform/tests/contract/test_api.py masar-simulation-platform/tests/contract/test_event_ws.py
  git commit -m "feat(platform): add run orchestration API"
  ```

## 8. Task 5: ROS1/ROS2 Web Sidecars and Reverse Proxy

**Files:**
- Create: `deploy/ros1-web/Dockerfile`, `deploy/ros2-web/Dockerfile`
- Create: `deploy/sidecars/ros1-supervisord.conf`, `deploy/sidecars/ros2-supervisord.conf`
- Create: `deploy/compose.yaml`, `deploy/caddy/Caddyfile`
- Create: `tests/integration/test_sidecar_configs.py`

- [x] **Step 1: Write configuration tests.** Parse Compose and Caddy files; assert exact loopback port table, ROS1 Noetic package names, ROS2 Humble package names, WebSocket upgrade headers, noVNC/Webots routes, and absence of `18088`.

- [x] **Step 2: Run configuration tests before implementation.**

  Run: `python3 -m pytest tests/integration/test_sidecar_configs.py -q`  
  Expected: FAIL because deployment files do not exist.

- [x] **Step 3: Build sidecar images.** Install the apt packages verified in the containers: `ros-noetic-rosbridge-suite`, `ros-noetic-tf2-web-republisher`, `ros-noetic-web-video-server` for ROS1; corresponding `ros-humble-*` packages for ROS2. Pin base image digests in the generated lock record. Do not `apt-get install` into `masar_ros1_srv-g` or `masar_ros2_bridge-w`.

- [x] **Step 4: Add supervisor commands and health checks.** Launch rosbridge, TF republisher and video server with the ports in `SPEC.md:19.2`. Health checks query the process supervisor and a local TCP socket; readiness remains the Platform API's ROS topic/type probe.

- [x] **Step 5: Add Caddy routes.** Preserve `/api/`, `/ws/`, `/ros1/`, `/ros2/`, `/video/ros1/`, `/video/ros2/`, `/display/gazebo/`, `/display/rviz/`, and `/webots/` WebSocket upgrades. Reject requests to Agent ports and any path containing `..`.

- [x] **Step 6: Build and smoke test sidecars.**

  ```bash
  docker compose -f deploy/compose.yaml config
  docker compose -f deploy/compose.yaml build ros1-web ros2-web
  docker compose -f deploy/compose.yaml up -d ros1-web ros2-web caddy
  curl -fsS http://127.0.0.1:18090/api/v1/capabilities
  ```

  Expected: Compose config succeeds, both sidecars report healthy, capabilities returns JSON, and `ss -ltnp` shows no public bind.

- [x] **Step 7: Commit deployment support.** Historical integration is recorded in `ACCEPTANCE.md`; the current user-owned dirty worktree is not auto-committed.

  ```bash
  git add masar-simulation-platform/deploy masar-simulation-platform/tests/integration/test_sidecar_configs.py
  git commit -m "feat(platform): add ROS web sidecars and proxy"
  ```

## 9. Task 6: Webots Streaming Compatibility and Existing-Project Guardrails

**Files:**
- Modify: `../ROS2swarm_w/start_robosar_webots.sh`
- Modify: `../ROS2swarm_w/src/launch_webots/launch/start_webots.launch.py`
- Modify: `../ROS2swarm_w/src/launch_webots/launch/create_enviroment.launch.py` only if argument forwarding requires it
- Modify: `../ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py`
- Modify: `../ROS2swarm_w/tests/test_webots_start_scripts.py`
- Create: `backend/masar_platform/webots.py`, `tests/integration/test_webots_proxy.py`
- Create: `deploy/webots-guard/Dockerfile`, `deploy/webots-guard/webots-port-guard`

- [x] **Step 1: Capture the pre-change command baseline.** Run `--dry-run` for `start_simulation.sh` and `start_robosar_webots.sh`; save stdout under `tests/fixtures/baseline/` with the current commit SHA and do not edit the fixture manually.

- [x] **Step 2: Write failing compatibility tests.** Assert the current default launch still has `stream=false` and the existing port, while `WEBOTS_STREAM=true WEBOTS_PORT=18141 ./start_robosar_webots.sh --dry-run --no-vglrun` forwards explicit `webots_stream:=true webots_port:=18141` launch arguments and both the swarm and RoboSAR launch paths pass `stream=True`, `port='18141'` to `WebotsLauncher`.

- [x] **Step 3: Implement the smallest default-off argument change.** Make `start_robosar_webots.sh` consume the platform-only `WEBOTS_STREAM`/`WEBOTS_PORT` environment metadata, add declared launch arguments to both Webots launch paths, and forward them. Preserve the current defaults and all existing world, mode, GUI and supervisor behavior when those variables are absent.

- [x] **Step 4: Run existing ROS2 tests and dry-run comparisons.**

  ```bash
  python3 -m pytest ../ROS2swarm_w/tests/test_webots_start_scripts.py -q
  cd ../ROS2swarm_w
  ./start_simulation.sh --dry-run --no-vglrun
  ./start_robosar_webots.sh --dry-run --no-vglrun
  ```

  Expected: existing tests pass; default business argv remains unchanged except for explicit platform arguments.

- [x] **Step 5: Verify the Webots path.** Start a stream-only world on port 18141, request `/webots/` through Caddy, assert WebSocket upgrade, non-empty canvas pixels in Chromium, and drag/zoom changes the camera state. If path-prefix handling fails, configure Caddy path stripping and repeat; do not expose a second public port. Webots R2025a has no bind-host option, so the accepted deployment adds an exact IPv4/IPv6 firewall guard that rejects non-loopback ingress without altering the native process.

- [x] **Step 6: Commit the compatibility change separately from new platform code.** Integrated implementation commit: `65ab31c`.

  ```bash
  git add ../ROS2swarm_w/start_robosar_webots.sh ../ROS2swarm_w/src/launch_webots/launch/start_webots.launch.py ../ROS2swarm_w/src/launch_webots/launch/create_enviroment.launch.py ../ROS2swarm_w/src/launch_webots/launch/robosar_webots_sim.launch.py ../ROS2swarm_w/tests/test_webots_start_scripts.py backend/masar_platform/webots.py tests/integration/test_webots_proxy.py
  git commit -m "feat(ros2swarm): expose default-off Webots stream"
  ```

## 10. Task 7: Gazebo/RViz Displays and noVNC

**Files:**
- Create: `deploy/display/display_manager.py`, `deploy/display/xstartup-gazebo`, `deploy/display/xstartup-rviz`
- Create: `deploy/novnc/Dockerfile`, `deploy/novnc/supervisord.conf`
- Create: `backend/masar_platform/displays.py`
- Create: `tests/integration/test_displays.py`

- [x] **Step 1: Write display lifecycle tests.** Assert allocation of `:101` for Gazebo and `:102` for RViz, separate PID/PGID records, bounded shutdown, and no reuse of a display owned by an unrelated process.

- [x] **Step 2: Run tests and verify failure.**

  Run: `python3 -m pytest tests/integration/test_displays.py -q`  
  Expected: FAIL because the display manager does not exist.

- [x] **Step 3: Implement display manager.** Start Xvfb and openbox in the GUI sidecar; expose each display through an x11vnc/websockify/noVNC process. Record all PIDs under the current `run_id`. Preserve the existing `run_vnc_gpu.sh` environment path for direct native runs; platform displays are only used by platform-owned processes.

- [x] **Step 4: Implement platform GUI wrappers.** Start Gazebo-bearing scripts with `--no-rviz`, set `DISPLAY=:101`, and start the repository's RViz config separately on `DISPLAY=:102`. Record the exact RViz command and config file in the resolved manifest.

- [x] **Step 5: Test both panels through Caddy.** Assert independent noVNC WebSocket connections, non-empty canvas screenshots, and mouse focus isolation. A noVNC panel failure must set a panel error and not stop the ROS run unless its display is a required readiness step.

- [x] **Step 6: Run tests and commit.** Integrated implementation commit: `8b202f8`.

  Run: `python3 -m pytest tests/integration/test_displays.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/deploy/display masar-simulation-platform/deploy/novnc masar-simulation-platform/backend/masar_platform/displays.py masar-simulation-platform/tests/integration/test_displays.py
  git commit -m "feat(platform): add independent Gazebo and RViz displays"
  ```

  Accepted on 2026-08-01: the narrow suite returned `10 passed`, the full platform suite returned `144 passed`, the existing ROS2 Webots suite returned `15 passed`, and all seven native container dry-runs returned exit code 0 with `platform_mode=0`. The rebuilt display sidecar and Caddy remained healthy; real Gazebo and RViz canvases were non-empty and simultaneously connected through the two same-origin noVNC routes, with independent iframe focus.

## 11. Task 8: React Workbench and Responsive Layout

**Files:**
- Create: `frontend/package.json`, `frontend/package-lock.json`, `frontend/tsconfig.json`, `frontend/vite.config.ts`
- Create: `frontend/src/app/App.tsx`, `frontend/src/app/runStore.ts`, `frontend/src/app/api.ts`
- Create: `frontend/src/panels/SimulationPanel.tsx`, `RvizPanel.tsx`, `MapPanel.tsx`, `SensorPanel.tsx`, `HealthPanel.tsx`, `MissionPanel.tsx`, `EventLogPanel.tsx`, `ProcessLogPanel.tsx`
- Create: `frontend/src/styles/layout.css`, `frontend/src/styles/tokens.css`
- Create: `frontend/tests/layout.spec.ts`, `frontend/tests/panel-state.spec.ts`

- [x] **Step 1: Write Playwright layout tests.** At 1440x900 assert the four required panels are simultaneously visible for exploration/rescue; at 390x844 assert single-column order, toolbar height at most 56 px, no horizontal overflow and no overlapping bounding boxes. At swarm scenarios assert RViz and generic behavior metric labels are absent.

- [x] **Step 2: Run the browser tests before implementation.**

  Run: `cd frontend && npm ci && npx playwright test tests/layout.spec.ts --project=chromium`  
  Expected: FAIL because the Vite app and panels are absent.

- [x] **Step 3: Implement the shell and state store.** Use a single `run_id` store, explicit panel registry, CSS grid areas and panel actions `resize`, `maximize`, `restore`, `popout`. Popout uses the existing run store and never calls `POST /runs`.

- [x] **Step 4: Implement the three module layouts.** Swarm uses simulation, trajectory, sensor, health and logs; exploration adds RViz, map and progress; rescue adds mission timeline and response status. Render no behavior score fields anywhere in the swarm layouts.

- [x] **Step 5: Implement missing-data states.** `UNAVAILABLE`, `WAITING`, `ONLINE`, `STALE`, `TYPE_MISMATCH` and `DISCONNECTED` must be rendered from API data. The SensorPanel renders `未提供` for an absent camera and never replaces it with zero or sample values.

- [x] **Step 6: Run Playwright tests and commit.** Integrated implementation commit: `5a54a9b`.

  Run: `cd frontend && npm ci && npx playwright test tests/layout.spec.ts tests/panel-state.spec.ts --project=chromium`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/frontend
  git commit -m "feat(frontend): add synchronized simulation workbench"
  ```

  Accepted on 2026-08-01: seven Playwright tests passed across desktop and mobile layouts; TypeScript and the production Vite build passed; `npm audit` reported zero vulnerabilities; the platform suite remained `144 passed`. Caddy served the production bundle at `/` while preserving API, ROS, Webots and both noVNC routes. Real Chromium screenshots covered all three modules and 390x844 mobile layout with no console errors or horizontal overflow.

## 12. Task 9: Robot Web Tools Adapters

**Files:**
- Create: `frontend/src/ros/RosConnectionManager.ts`, `TopicRegistry.ts`, `TopicHealth.ts`, `TfClient.ts`, `VideoClient.ts`
- Modify: `frontend/src/panels/MapPanel.tsx`, `SensorPanel.tsx`, `HealthPanel.tsx`
- Create: `frontend/tests/ros-adapters.unit.spec.ts`, `frontend/tests/ros-live.spec.ts`

- [x] **Step 1: Write adapter tests with controlled ROS doubles and live sidecars.** Cover one shared ROS1 connection, one shared ROS2 connection, reconnect without losing subscriptions, topic type rejection, queue length one, 10 Hz render cap, stale transition at the profile threshold, TF frame selection, and synchronously releasing the previous video stream.

- [x] **Step 2: Run tests and capture failure.** The final review added five regression cases that first failed for reconnect, registry replacement, timer leakage, video/topic health interference and large LaserScan stack overflow; a sixth case then exposed unchanged health snapshots emitting every 500 ms.

  Run: `cd frontend && npm run test:unit`
  Expected: FAIL because adapters do not exist.

- [x] **Step 3: Implement the singleton connection manager.** Use `roslib` 2.1.0 and route all topic subscriptions through `TopicRegistry`. Components cannot create raw WebSockets. `TopicHealth` records observed frequency and last age as transport health only. ROS1 and ROS2 reconnect with bounded backoff while an unchanged contract preserves the existing subscription registry.

- [x] **Step 4: Implement map and TF adapters.** Keep the task-allocation image on the existing video snapshot path, render world-coordinate traces with native SVG, and expose an explicit-frame-only `TfClient` for panels that require transforms. PointCloud2 remains disabled. The unused ROS2D/EaselJS and ros3djs chains were removed after the dependency audit.

- [x] **Step 5: Implement video and sensor adapters.** Build web-video URLs from the same-origin API capability response; synchronously clear the old stream before selecting a new one. Decode only profile-approved LaserScan, Odometry and Twist messages. Native MJPEG `<img>` load/error is reported as connection health without inventing per-frame timestamps.

- [x] **Step 6: Run adapter, browser and TypeScript checks.**

  ```bash
  cd frontend
  npm run test:unit
  npm run typecheck
  npm run build
  npm run test:e2e
  MASAR_LIVE_ROS=1 npx playwright test tests/ros-live.spec.ts
  npm audit --audit-level=moderate
  ```

  Expected: all commands PASS.

- [x] **Step 7: Commit the adapters.** Implementation commit: `9587ac2`.

  ```bash
  git add masar-simulation-platform/frontend/src/ros masar-simulation-platform/frontend/src/panels masar-simulation-platform/frontend/tests
  git commit -m "feat(platform): integrate Robot Web Tools data plane"
  ```

  Accepted on 2026-08-01: 17 adapter unit tests, seven standard Chromium tests and two live ROS browser tests passed. The live checks rendered a real ROS1 `nav_msgs/OccupancyGrid`, ROS2 `nav_msgs/msg/Odometry` trajectory and bounded `sensor_msgs/msg/LaserScan` values through the production same-origin gateway. TypeScript, Vite build, npm audit, the 144-test Python suite, Ruff, compileall, Compose validation, shell syntax and diff checks passed. All temporary test publishers were removed by exact container PID.

## 13. Task 10: Artifact Recording and Download

**Files:**
- Modify: `agents/runtime_agent/main.py`, `probes.py`, `processes.py`; `backend/masar_platform/agents.py`, `artifacts.py`, `db.py`, `events.py`, `orchestrator.py`, `api.py`
- Create: `backend/masar_platform/recording.py`
- Create: `tests/unit/test_artifacts.py`, `tests/contract/test_download.py`

- [x] **Step 1: Write artifact tests.** Assert fixed manifest/summary keys, SHA-256 artifact IDs, GNU checksum format, complete events copied before retention, path traversal rejection, and Range responses (`206`, `Content-Range`, `Accept-Ranges`).

- [x] **Step 2: Run tests to verify failure.** Artifact, download, recorder ownership, finalization retry and display-recovery regressions were each observed failing before implementation or correction.

  Run: `python3 -m pytest tests/unit/test_artifacts.py tests/contract/test_download.py -q`  
  Initial expected result: FAIL because artifact and recording implementations are absent.

- [x] **Step 3: Implement finalization.** Write resolved manifest, complete events, process logs, ROS graph snapshot, summary and checksums atomically through `.partial` files and `os.replace`. Keep the original failed-start reason and durable missing-data list across finalization retries and API recovery.

- [x] **Step 4: Implement recording profiles.** Start only exact manifest-derived ROS1/ROS2 recorder command refs and topic regexes; reject `-a`, `/.*`, mismatched profiles and caller-selected topics; stop and prove recorder inactivity before process groups and bag indexing.

- [x] **Step 5: Implement secure streaming download.** Resolve an `artifact_id` from SQLite, open each path component with `O_NOFOLLOW`, verify size and full SHA-256 on the opened descriptor outside the event loop, reject traversal/symlinks/tampering, and support single-range streaming with checksum headers.

- [x] **Step 6: Run tests and commit.** Implementation commit: `fccb40e`.

  Run: `python3 -m pytest tests/unit/test_artifacts.py tests/contract/test_download.py -q`  
  Expected: PASS.

  ```bash
  git add masar-simulation-platform/agents/runtime_agent masar-simulation-platform/backend/masar_platform masar-simulation-platform/tests
  git commit -m "feat(platform): archive runs and stream artifacts"
  ```

  Accepted on 2026-08-02: the narrow artifact/download suite returned `18 passed` and the full Python suite returned `176 passed`. Ruff, compileall, Compose validation, shell syntax and diff checks passed. Real Noetic and Humble containers produced valid strict-profile ROS1 and ROS2 bags and graph snapshots; all isolated publishers, Agents, secrets and run data were removed. The final API image and existing Compose volume passed health and schema migration checks. Frontend regression returned 17 unit tests and seven Chromium tests with two live-only skips; typecheck, build and npm audit passed. Final independent review reported no P0/P1/P2 findings.

## 14. Task 11: Scenario Integration and Regression Gates

状态：已由 Stage A-D 与 E3 的自动化、原项目回归和真实三模块 Run 验收取代。
当前保留 `tests/integration/test_scenario_smoke.py`、故障注入测试、ROS 图 fixture 与
`ACCEPTANCE.md` 中的 Run 证据；不再维护一套重复的待办清单或第二份运维文档。

## 15. Task 12: Security, Readability and Release Gate

状态：已由 Task 10、Stage D 与 E1-E4 的安全、归档、依赖审计、干净启动和文档门禁
取代。Compose 现在拥有两个业务 runtime，因此旧的“平台停止后外部业务容器仍运行”
要求已失效；当前权威发布证据统一记录在 `ACCEPTANCE.md`，当前使用方式统一记录在
`OPERATIONS.md`。

## 16. Dependency Graph and Sequencing

```text
Task 0 scaffold
  └─> Task 1 catalog/profiles/commands
       ├─> Task 2 state/lease/events
       │    └─> Task 3.5 integration contracts
       │         └─> Task 4 orchestrator/API
       │         ├─> Task 10 artifacts
       │         └─> Task 11 scenario integration
       └─> Task 3 runtime Agent
            └─> Task 3.5 integration contracts

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

## 19. Stage 0 Closure Amendments (2026-08-02)

Real dogfooding found lifecycle, rendering and data-panel gaps after Tasks 0-10. These amendments supersede conflicting W3D/display statements above while preserving their historical acceptance evidence. Stages execute strictly in order; any failed acceptance blocks the next stage.

### 19.0 Measured diagnostic baseline

- Swarm baseline `run-911043f496e84702ac0fbfdccff4a20e` used three Burger robots, `command_world.wbt` and `aggregation_pattern`. Webots consumed about `805% CPU`; during 19.09 wall seconds `/clock` advanced 8.66 seconds (real-time factor about `0.45`). `DISPLAY=:1` reported Mesa `llvmpipe`; `vglrun -d :0 glxinfo -B` reported an RTX 6000 Ada renderer. No Webots graphics process appeared as a GPU workload in `nvidia-smi pmon`.
- The same Run published `/robot_0..2/{odom,scan,cmd_vel}`, `/clock` and `/global_info`; robot 0 moved from `(0.188,-0.035)` to `(0.848,-0.416)`. The live browser still showed waiting trajectory, sensor and graph-health panels, an instruction-only log panel and a blank W3D area. This proves the primary panel gap is discovery/decoding/UI wiring, not absence of ROS data.
- That Run stopped as `COMPLETED/USER_STOP`; port `18141` and all Run-owned Webots/ROS children disappeared, both business containers stayed running, and all 11 archived checksums passed.
- Historical swarm `run-d92eef4e4fda4f08abb6cd52b65a86b8` stopped cleanly, but the next Run `run-dfbd61a2b4564ac485f10cb4f6e5d789` ended in Webots exit code `139` before readiness. Historical exploration `run-7af1d9ba386a4523aed53bd808903f6e` timed out with `Package 'ros1_bridge' not found` and repeated controller switch timeouts.
- `ros1_bridge` is present at `/root/ros-humble-ros1-bridge/install`; platform non-interactive startup simply does not source it. After either business container restarts, its Runtime Agent is also absent until manually bootstrapped. Both are environment/lifecycle contract defects, not missing application functionality.

### 19.1 Stage A: Runtime lifecycle and performance

- [x] Add tests for Webots noVNC display plan, `:103/18133` lifecycle, command gating outside `RUNNING`, exact Run process cleanup and normal viewer disconnect handling. The live browser gate additionally rejects a blank noVNC framebuffer by sampled color/luminance evidence.
- [x] Run Webots with `DISPLAY=:103` and `vglrun -d :0`; production launch sets `webots_stream=false`. Preflight rejects `llvmpipe` and missing `ros1_bridge` before starting a child.
- [x] Source the existing `/root/ros-humble-ros1-bridge/install/setup.bash` explicitly for platform exploration, and fail preflight with the exact missing executable if that installation is unavailable.
- [x] Align each platform-generated Webots world's `basicTimeStep` and ros2_control update period without changing direct-run defaults. Three- and five-Burger real-time factors reached `0.980` and `0.940`; both used the NVIDIA VirtualGL renderer and delivered changing browser frames.
- [x] Provide one idempotent platform startup command that restores both Runtime Agents after container restart and verifies their health before enabling the launch button.
- [x] Fix exploration Webots startup and ensure stop terminates every process owned by the Run while leaving both business containers and host processes intact. Run `run-ccceea6a7b254acea82f0db22e536a17` started ROS2 `start_robosar_webots.sh` before ROS1 `launch_robosar_w.sh`, stopped them in reverse order, and passed full archive checksum verification.
- [x] Measure and record the Gazebo exploration baseline, then run the full Python, frontend, Playwright and existing ROS-project regression gates. Run `run-287b2dd9af264a0597cc9ec31dc188a1` measured RTF `0.897`; the live Gazebo/RViz/map/camera Chromium gate, archive checksum and exact cleanup checks passed. Platform Python returned `224 passed`, frontend Chromium `14 passed, 4 live-only skipped`, ROS1 `90 passed` and ROS2 `22 passed`.
- [x] Acceptance: two consecutive swarm Webots runs and one exploration Webots run reached `RUNNING`; every stop ended `COMPLETED/USER_STOP`; no Run-owned process or run-only port remained. The shared Xvfb/noVNC gateways remain healthy between Runs by design. After warm-up, 30-second `/clock` samples reached real-time factors `0.980` for three Burger robots and `0.940` for the default five-Burger case. The renderer was NVIDIA, noVNC canvases were non-empty and changing, drag/zoom changed the Webots view, and the Gazebo exploration baseline was measured. Stage A passed on 2026-08-02.

### 19.2 Stage B: Group behavior/control module

- [x] Default world is `command_world.wbt`; expose `e_puck`/`burger`; add `discussed_dispersion_pattern` and a real discussion-event/status source.
- [x] Replace the map/trajectory placeholder with a live multi-robot world-coordinate trace and pose table. Use Webots `/global_info` and Gazebo `/gazebo/model_states`; keep local Odometry in the selected-robot sensor deck only. Sensor presentation follows the selected robot's actual camera/LaserScan/infrared/IMU capabilities. On desktop, place Webots/Gazebo above the equal-width trajectory and sensor panels; retain ordered single-column mobile behavior. Fix the group-behavior world to `command_world.wbt` without a world selector, and expose `e_puck`/`burger` through the robot-type selector.
- [x] Periodically rediscover topics, render all matched robots instead of an arbitrary first six, expose real topic freshness, and stream Run-owned stdout/stderr through the existing logs API.
- [x] Return the expected Run robot inventory from `/robots`, but keep ROS graph discovery, type validation, first-frame/Hz/STALE status in the browser `RosDataRuntime`. Batch all ROS2 readiness topics through one Domain 42 rclpy Context.
- [x] Keep rosapi service calls off the ROS2 rosbridge WebSocket event loop with `call_services_in_new_thread:=true`; reproduce the blocked handshake before the fix and verify a direct `101 Switching Protocols` response after deployment.
- [x] Do not expose planned robots as observed runtime state outside an active Run. Idle and terminal pages pass empty sources and zero expected robots to trajectory, health and sensor panels.
- [x] Reaccept both simulator coordinate paths. Webots Run `run-bffc31333a064ea08e5628dc3cf87a94` rendered `3/3` Supervisor world poses. Gazebo Run `run-c25cd415a2394c28a17397544edc9620` rendered `4/4` ModelStates poses and four 81-point moving traces after START. The live Gazebo test found the ROS1 Web image lacked the `gazebo_msgs` manifest; add `ros-noetic-gazebo-msgs`, rebuild only `ros1-web`, and verify real rosbridge decoding. Both Runs stopped cleanly with valid archives and unchanged business containers.
- [x] Acceptance: both robot types are selectable and the known-working Burger/e-puck platform paths have real Runs; trajectory, selected-robot sensors, all robot rows, discussion state, noVNC and Run logs are live; no RViz is started. Platform START waits for every current `/swarm_command` subscriber. Five-e-puck Run `run-4fdfe4f599094947bd10cefb97e2e064` passed robot_2, all-sensor, trajectory, Webots, log, stop and checksum gates after API/sidecar deployment. Burger aggregation and the manually selected ROS2swarm `sensor_type=ir` layer defect remain deferred outside the default platform path. Stage B passed on 2026-08-03.

### 19.3 Stage C: Exploration module

- [x] Replace the static task-map placeholder with the existing Qt allocation image and decode the real exploration sources: `/task_allocation_image`, `/task_allocation`, `/frontier_filter/filtered_frontiers`, `/percent_completed`, robot status/odometry and emergency-object snapshots. Latch the ROS1 allocator/filter state publishers so browser and video subscribers created after mission start receive the latest snapshot.
- [x] Add one shared per-robot selector for actual camera/scan/IMU/odometry sources, and decode discovered fire/victim/push-obstacle positions, detector robot IDs and optional push goals from emergency-object snapshots.
- [x] Close the panel semantics found during real Gazebo dogfooding. `任务分配与建图` owns the allocation image plus map coverage, candidate-frontier count and latest allocation batch (`本轮分配`). `探索任务态势` owns online exploration robots plus emergency counts and discoveries only. Remove duplicate allocation/progress fields, remove `已完成任务` because the default HIGH allocator never emits `/tasks_completed`, and never fabricate zero for a discovered but silent source; emergency counts become zero only after an observed empty snapshot.
- [x] Add an allocation-image first-frame watchdog. A successful MJPEG response may close with an empty multipart body before the first ROS image, without firing an image `error`; while the Run is active and `naturalWidth === 0`, reconnect after a bounded delay until a real frame arrives. Treat `/percent_completed` as the observed `0..1` coverage ratio and render `ratio * 100` percent.
- [x] Correct the allocation-image reconnect transport. Gazebo acceptance candidate `run-d6d8007aac864d6bae4bfcfde8e14179` received one HTTP 200 empty multipart response for `/task_allocation_image`; its Playwright trace contained no second image request even though the watchdog repeatedly changed React state and ROS later held a real latched `640x480` frame. The cause was that `VideoClient` deduplicated the unchanged URL and Chromium reused the completed empty response. Only watchdog retries now append `retry=N`; the initial allocation URL and ordinary robot-camera URLs remain unchanged. The replacement Playwright gate counts actual browser requests and first failed with one request, then passed with distinct retry URLs.
- [x] Use the video server's snapshot transport for the low-frequency allocation image. Diagnostic Webots Run `run-f02a8da600e548ec9e7ae5a5ae7ebea7` proved that changing MJPEG URLs alone is insufficient when a latched source emits one JPEG and then keeps the multipart connection open: Chromium aborted 40 distinct requests at the watchdog interval without committing `naturalWidth`, although direct curl received the valid 29 KB part. The existing same-origin `/video/ros1/snapshot` route returned the complete JPEG in 7 ms. The allocation panel now retries snapshots every 1.5 seconds until its first frame and refreshes every 3 seconds afterward, with a changing cache key; robot cameras remain on MJPEG streams.
- [x] Prefer an observed per-robot source over a silent duplicate. Webots publishes ROS2 `/robot_i/odom` around 40-45 Hz while ROS1 `/robosar_agent_bringup_node/agent_i/odom_data_euler` can be discovered without a publisher; first-match selection made sensors and health wait forever. Per-robot capability selection now ranks `ONLINE` above `STALE`, waiting/discovered, disconnected, type-mismatch and unavailable sources. The online exploration count uses live per-robot odometry before falling back to the low-frequency agent-status snapshot.
- [x] Automated gate: first prove the old UI fails for map/mission ownership, `0.375 -> 37.5%`, absence of fabricated assignment/completion zero and retry after an empty successful MJPEG stream. The three RED failures were observed, then the focused panel suite returned `20 passed`, adapter units `30 passed`, TypeScript/build passed and Chromium returned `27 passed, 7 live-only skipped` on 2026-08-03.
- [x] Provide the ROS1 Web sidecar with the existing `masar_ws/devel` message overlay. Failed acceptance Run `run-37e0e2076e0a4402aca51349e0467bf6` proved the browser sent subscriptions for latched `/task_allocation` and `/frontier_filter/filtered_frontiers`, while rosbridge delivered only standard-message `/percent_completed`; `rospack` and Python imports inside ros1-web confirmed `robosar_messages` was absent. The deployed fix mounts `masar_ws` read-only at its canonical path and requires/sources `devel/setup.bash` before every sidecar program. Container imports and a temporary real-browser custom `task_allocation` smoke passed; the temporary publisher/topic disappeared and neither business container restarted.
- [x] Gazebo acceptance: two complete fresh cycles passed. In each cycle mission commands before `RUNNING` returned `409 RUN_NOT_ACTIVE`; after `RUNNING`, the browser observed a real allocation frame, coverage/frontier/latest-batch metrics, five selectable robot sensor views, online robots and emergency snapshots. Runs `run-1e6dc13688634b0bb9bae467ffdb8ceb` and `run-4ba8c33b877149a8acf8e1da26caef8b` both stopped as `COMPLETED/USER_STOP`, passed all 13 archive checksums and left no Run-owned process or Run-only port without restarting either business container.
- [x] Webots acceptance: Runs `run-919530af7fae462893dfc5ebfff3bd7a` and `run-61aa650ac5814725bb95fcc1f950b284` passed the same two-cycle gate with required startup order `ROS2swarm_w/start_robosar_webots.sh` then `masar_ws/launch_robosar_w.sh`, reverse RViz/ROS1/ROS2 stop order, nonblank Webots and RViz noVNC canvases, late-source recovery and no residual Run process. Both Runs ended `COMPLETED/USER_STOP`, each verified 16 archived files and neither business container restarted. Stage C passed on 2026-08-03.

### 19.4 Stage D: Complete rescue and documentation closure

- [x] Reuse exploration panels and decode `ResponseGroupStatus`, active task/result, behavior mode, planning state, formation error, clearance and motion-enabled values for the Run data layer.
- [x] Compact the production response summary to six operator-critical metrics: combined response/planning state, motion enabled, active task, task queue, formation error and minimum clearance. Do not render separate behavior mode, formation scale, assigned-robot count or latest-result rows; their current information is duplicate, secondary or inactive for most of the Run.
- [x] Protect the compact layout with Chromium assertions that the last response metric remains inside the response panel and that the following health panel starts after it. Use a vertical status-rail flow so expanding one panel cannot overlap another.
- [x] Close the right-rail clipping and unused-space regression. Keep mission/response/ordinary panels at `180px`/`220px`/`145px` minimum bases, distribute spare height across them, scroll the rail when the viewport is shorter, and scroll the emergency-object list inside its mission panel. RED browser evidence measured a `200px` tall-viewport gap and a task row `51px` outside the panel; the focused GREEN gate returned `2 passed`.
- [x] Remove every hardcoded `未提供`/instruction-only visible panel whose contract has an available real source. Low-frequency/latched sources are not marked offline solely from frequency.
- [x] Acceptance: real complete rescue Run `run-8adea267bc8f43f4b89d396b5096f1f6` showed exploration targets/tasks and changing response-group values, then stopped as `COMPLETED/USER_STOP`; all Agent records stopped, all 16 archive checksums passed, no Run process/port remained and neither business container restarted.
- [x] Synchronize the specification, design, plan and acceptance documents; add an exact operations guide for startup, SSH forwarding, use and shutdown; run the full Python, frontend, Playwright, ROS regression and clean-start gates. Final evidence is recorded in `docs/ACCEPTANCE.md` under Stage D.

## 20. Compose-Managed Runtime Closure (2026-08-03 to 2026-08-04)

This is the active deployment plan. It supersedes external runtime containers,
post-start Agent injection, host Node builds and Compose profiles described in
earlier historical tasks.

### 20.1 Stage 1: Automated deployment contract

- [x] Add `ros1-runtime` and `ros2-runtime` to the single Compose project with
  immutable public v2 digests, host network/IPC, GPU, X11, workspace mounts,
  Domain 42 and Runtime Agent health checks.
- [x] Keep exactly eight long-running services and make the Agent each runtime
  container's main process.
- [x] Bind-mount mutable backend, scenario, sidecar, display, guard, Caddy and
  frontend output so ordinary source changes do not rebuild dependency images.
- [x] Add `make docker-init`, `docker-start`, `docker-stop`, `docker-status`,
  `docker-logs` and `docker-doctor` through `deploy/platform.sh`.
- [x] Reuse matching local image digests, pull only missing runtime images,
  build frontend assets in a pinned Node container, and build missing catkin/
  colcon overlays with full logs under `.runtime/logs`.
- [x] Remove the obsolete startup script, Agent injection path, Caddy frontend
  build image and unused duplicate sidecar YAML manifests.
- [x] Pass the complete Python, frontend, Compose, shell, Ruff, compile and diff
  gate, then record exact counts in `ACCEPTANCE.md`. E1 passed with Python
  `250 passed, 2 skipped` and frontend unit `36 passed`.

### 20.2 Stage 2: Real migration and restart recovery

- [x] Require `current Run = null`, capture unrelated container IDs and verify
  both v2 RepoDigests before migration.
- [x] Run `make docker-start`; migrate only the two exact old runtime container
  names and require all eight services healthy.
- [x] Restart each runtime container independently and prove its Agent returns
  without a repair command or restart of the other runtime.
- [x] Run `make docker-stop` and prove unrelated containers and host processes
  are unchanged; start the stack again and record evidence.

### 20.3 Stage 3: Three-module real regression

- [x] Accept one Webots group-behavior Run, clean stop and archive.
- [x] Accept one Webots exploration Run with ROS2-before-ROS1 startup, reverse
  stop, non-empty panels and archive.
- [x] Accept one complete Gazebo rescue Run with exploration, emergency object,
  response group, Gazebo/RViz and archive evidence.

### 20.4 Stage 4: Fresh overlay and documentation closure

- [x] Exercise missing `masar_ws/devel` and `ROS2swarm_w/install` in an isolated
  workspace; require successful setup files, logs and generated-directory
  ownership without source ownership changes.
- [x] Run stale-reference scans and a final clean `make docker-stop` /
  `make docker-start` regression.
- [x] Make `docs/OPERATIONS.md` the only complete startup guide and record all final
  evidence and residual upstream risks in `docs/ACCEPTANCE.md`.

## 21. Portable Workspace, Unified SSH Entry and Documentation Closure (2026-08-04)

This section is the active documentation/deployment cleanup plan. The detailed test-first
steps are recorded in `docs/superpowers/plans/2026-08-04-workspace-path-and-ssh-entry.md`.

### 21.1 Portable workspace path

- [x] Resolve the workspace from the current checkout or `MASAR_WORKSPACE_ROOT` and normalize
  it to a real absolute directory.
- [x] Reject an invalid workspace before any Docker call and preserve the three canonical
  paths inside `/root/workspace`.
- [x] Pass 21 focused startup/deployment tests, Compose validation, Bash syntax and a real
  read-only `make docker-doctor`. Implementation commit: `b42148b`.

### 21.2 One-to-one SSH entry

- [x] Keep the only browser URL at `http://127.0.0.1:18090` for both local and remote use.
- [x] Print `LocalForward 18090 127.0.0.1:18090` from start/status without opening another
  listener or managing an SSH process.
- [x] Remove the alternate browser URL from the operator guide and pass 13 focused platform
  script tests plus a live status check. Implementation commit: `e244285`.

### 21.3 Documentation layout and stale-content removal

- [x] Keep project summary and quick start in the root `README.md`.
- [x] Move operations, specification, architecture, implementation plan and acceptance ledger
  to `docs/`; update every current link.
- [x] Remove duplicate 2026-08-03 Compose task documents and all active host-specific paths or
  alternate SSH port instructions.

### 21.4 Complete regression and closure

- [x] Run the complete Python, frontend, Compose, shell, Ruff, compile and diff gates.
- [x] Verify all eight services, the gateway and current Run state without launching a new Run.
- [x] Record exact results in `docs/ACCEPTANCE.md` and recheck README/operations consistency.
