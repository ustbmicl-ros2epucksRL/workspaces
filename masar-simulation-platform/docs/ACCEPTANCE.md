# MASAR Unified Simulation Platform Acceptance Ledger

This ledger is the authoritative implementation gate record. A task may move to
`PASS` only after its narrow tests, Spec compliance review, and code-quality
review all pass. The next task must not start before that point.

Deployment evidence in Tasks 0-D records the historical external-container
implementation used at that date. It is superseded by the active
Compose-managed closure rows and section at the end of this ledger.

| Task | Status | Narrow acceptance | Spec review | Quality review | Integrated commits |
|---|---|---|---|---|---|
| 0. Scaffold and Tooling | PASS | `7 passed`; `make compile` passed | APPROVED | APPROVED after fixes | `8f7b3b4`, `a3133b2` |
| 1. Scenario Schema, Profiles and Static Commands | PASS | `25 passed`; full unit suite `32 passed`; schema, compile, Ruff, container dry-runs passed | APPROVED | APPROVED after fixes | `cc45188`, `ea83a16`, `84427db`, `4c79639`, `cb28cd4`, `f2be4c6` |
| 2. SQLite State, Lease and Events | PASS | `35 passed`; full unit suite `67 passed`; schema, compile, Ruff passed | APPROVED after fixes | APPROVED after fixes | `bc1e0b4`, `bcade1a`, `d553693` |
| 3. Runtime Agent and Process Ownership | PASS | `23 passed`; full suite `90 passed`; Ruff, compileall, diff check and both container py_compile/--help checks passed | APPROVED | APPROVED after Python 3.8 fix | `16b56e6`, `043fc10`, `a9df88a` |
| 3.5. Integration Contracts | PASS | full suite `103 passed`; Ruff, compileall, Compose build/health, signed ROS1/ROS2 topic probes and platform-mode dry-runs passed | APPROVED | APPROVED | `8936d8a` |
| 4. Orchestrator and REST/WebSocket API | PASS | `19 passed`; full suite `125 passed`; Ruff, compileall, API/Compose smoke and live ROS1/ROS2 command publication passed | APPROVED | APPROVED | `8936d8a` |
| 5. ROS1/ROS2 Web Sidecars and Reverse Proxy | PASS | `7 passed`; full suite `130 passed`; both sidecars/Caddy healthy; live ROS1+ROS2 subscribe and publish-denial smoke passed | APPROVED | APPROVED | `8936d8a` |
| 6. Webots Stream Compatibility | PASS | ROS2 host/container `15 passed` each; platform full suite `134 passed`; real WebSocket, Chromium canvas/interaction and firewall isolation passed | APPROVED after native-bind correction | APPROVED | `65ab31c` |
| 7. Gazebo/RViz/noVNC Displays | PASS | `10 passed`; full suite `144 passed`; live dual-canvas/HTTP/listener checks passed | APPROVED | APPROVED after ownership fixes | `8b202f8` |
| 8. React Workbench | PASS | `7 passed`; typecheck/build/audit and live Caddy Chromium review passed | APPROVED | APPROVED after truthful-data and dependency fixes | `5a54a9b` |
| 9. Robot Web Tools Adapters | PASS | `17 passed`; `7 passed + 2 skipped`; live ROS `2 passed`; full suite `144 passed` | APPROVED with ros3d security deviation | APPROVED after reconnect/resource fixes | `9587ac2` |
| 10. Artifact Recording and Download | PASS | `18 passed`; full suite `176 passed`; real ROS1/ROS2 bag smoke passed | APPROVED | APPROVED after ownership/recovery fixes | `fccb40e` |
| A. Runtime Lifecycle and Performance Closure | PASS | real Webots/Gazebo performance, noVNC and lifecycle gates passed | v0.4 approved | APPROVED after lifecycle fixes | - |
| B. Group Behavior Data Closure | PASS | real aggregation/discussion Runs; `228` Python, `23` frontend unit, `20` Chromium and live gates passed | v0.4 approved | APPROVED after command-delivery fix | - |
| C. Exploration Data Closure | PASS | real Gazebo/Webots Runs, browser panels, cleanup and archives passed | v0.4 approved | APPROVED | - |
| D. Rescue and Documentation Closure | PASS | real complete rescue, layout, cleanup and 16 archive checksums passed | v0.4 approved | APPROVED | - |
| E1. Compose-managed automated gate | PASS | `250 passed, 2 skipped`; frontend `36 passed`; all static/build gates passed | v0.5 approved | APPROVED | - |
| E2. Compose migration/restart | PASS | API bind-mount recovery; legacy migration; runtime restart; stop/start | v0.5 approved | APPROVED | - |
| E3. Compose three-module regression | PASS | real swarm Webots, exploration Webots and rescue Gazebo browser/lifecycle/archive gates | v0.5 approved | APPROVED | - |
| E4. Fresh-overlay/docs closure | PASS | isolated first overlay build; clean restart; final full regression | v0.5 approved | APPROVED | - |
| 11. Scenario Integration and Regression | SUPERSEDED | Covered by A-D and E3 real/automated gates | superseded | superseded | - |
| 12. Security and Release Gate | SUPERSEDED | Covered by Task 10, D and E1-E4 gates | superseded | superseded | - |

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

## Task 3.5: Integration Contracts

Date: 2026-08-01

- Scenario acceptance: every manifest declares the loopback ROS1 master and ROS2 domain. The initial split-domain implementation was superseded in Stage A by the unified domain `42` contract. Every ROS readiness probe declares the exact ROS1/ROS2 message type, maximum silence window and optional ROS2 QoS profile.
- Probe acceptance: signed `POST /v1/runs/{run_id}/probe` performs bounded, shell-free TCP, HTTP, workspace-file, ROS1 topic and ROS2 topic checks. ROS topic checks verify graph presence, exact type and a newly observed message, returning stable `READY`, `NOT_FOUND`, `TYPE_MISMATCH`, `STALE`, `UNREACHABLE` or `HTTP_ERROR` statuses.
- Ownership acceptance: Runtime Agent children always receive `MASAR_PLATFORM_MODE=1`; ROS1 platform scripts do not create nested sessions, do not start the original Qt GUI, and bypass native global `pgrep`/`pkill` cleanup. Native script defaults remain unchanged outside platform mode.
- Proxy and deployment acceptance: Runtime Agents, child processes and every Compose service inherit `NO_PROXY/no_proxy=127.0.0.1,localhost,::1`. `docker compose config`, API image build, container health request and in-container environment assertions passed; the temporary Compose container was removed.
- Container acceptance: a temporary ROS1 Noetic publisher and a ROS2 Humble domain-42 publisher both returned `READY` through signed Runtime Agent probes with exact observed types. Gazebo exploration, Webots exploration, Tepper response and response DEFORM dry-runs confirmed platform mode and Qt suppression. Temporary publishers, masters, Agents, secrets, ports and directories were removed.
- Automated acceptance: `python3 -m pytest tests -q` returned `103 passed`; `compileall`, Ruff `0.9.2`, Compose validation and `git diff --check` passed. ShellCheck is not installed on the host; all modified shell scripts passed `bash -n` and container dry-runs.
- Filesystem note: three root-owned ROS1 scripts in the shared mount were reassigned to host UID/GID `1006:1006` through the existing ROS1 container so scoped patches could be applied; executable mode `0755` was preserved.

## Task 4: Orchestrator and REST/WebSocket API

Date: 2026-08-01

- API acceptance: capabilities expose exactly three user workspaces (`group_behavior`, `exploration`, `rescue`) and six scenarios. Exploration and rescue profiles always include simulation, RViz, map and sensors; swarm/control do not expose RViz. Validation is static and returns resolved parameters, runtime identity and shell-free invocation data.
- Run acceptance: creation persists request and resolved scenario before asynchronous startup; the SQLite partial unique index and lease enforce one active run. Webots starts ROS2 before ROS1, required readiness gates each dependency, timeouts and startup exits roll back in manifest `stop_order`, and user stop is idempotent at the public state boundary.
- Recovery acceptance: a fully proven RUNNING Agent tree is adopted by a new API owner and monitored; missing or incomplete ownership becomes `FAILED/NEEDS_INSPECTION` without speculative process cleanup.
- Command acceptance: browser routes map only to six fixed Agent actions. Exploration START/STOP/HOME, ROS2swarm start/stop and response return-home use typed fixed topics; DEFORM exploration HOME is rejected separately from response return-home. No request accepts topic, type, value or shell text.
- Event and artifact acceptance: REST cursor replay and WebSocket envelopes share the same durable sequence. Every created run registers downloadable `request.json` and `resolved-scenario.json`; path containment and checksums are enforced.
- Safety gate: production `build_orchestrator()` requires `MASAR_PLATFORM_VISUAL_STACK_READY=true` before any business Run can start. Until Tasks 5-7 pass, the platform API can expose health/catalog/capabilities but cannot accidentally launch a headless run that the user cannot observe.
- Historical automated acceptance (superseded deployment): the Task 4 narrow suite returned `19 passed`; full suite returned `125 passed`; Ruff, compileall and `git diff --check` passed. The then-current API image served health, three-workspace capabilities and the six-scenario catalog.
- Physical command smoke: signed ROS1 `ros1.mission.start` produced `robosar_messages/mission_command {data: 1}` on `/system_mission_command`; signed ROS2 domain-42 `ros2.swarm.start` produced `communication_interfaces/msg/Int8Message {data: 1}` on `/swarm_command`. Temporary Agents, ROS master, subscribers, ports and secrets were removed.
- Runtime directory note: the empty `runs/` directory was reassigned to host UID/GID `1006:1006` so the production host API can create SQLite; runtime contents are ignored and `.gitkeep` preserves only the directory boundary.

## Task 5: ROS1/ROS2 Web Sidecars and Reverse Proxy

Date: 2026-08-01

- TDD evidence: the new deployment configuration suite initially returned four expected failures and one pass because the sidecar images, supervisor definitions, lock record and Caddy routes did not yet exist.
- Image acceptance: dedicated Noetic and Humble web images build from digest-pinned ROS base images and contain pinned `rosbridge-suite`, `tf2-web-republisher` and `web-video-server` package versions. No package was installed into or image rebuilt for `masar_ros1_srv-g` or `masar_ros2_bridge-w`.
- ROS graph acceptance: the Noetic sidecar removes advertise, publish and service-call capabilities and explicitly binds Twisted to loopback. The Humble sidecar uses separate empty publish globs, subscription allowlists, the selected `ROS_DOMAIN_ID`, host networking and host IPC so Fast DDS matches the existing ROS2 business container.
- Live protocol acceptance: real `std_msgs/String` messages traversed both `ws://127.0.0.1:18090/ros1/` and `/ros2/`. Browser advertise/publish attempts did not create either test topic in the ROS1 or ROS2 graph.
- Proxy acceptance: Caddy validates with HTTP/1.1 and HTTP/2 only, preserves every reserved API/ROS/video/noVNC/Webots path, forwards WebSocket upgrade headers, rejects encoded traversal, and exposes no Agent route. `ss -lntp` showed the gateway, both rosbridges and both video servers only on `127.0.0.1`.
- Environment acceptance: every Compose service receives exact `NO_PROXY/no_proxy=127.0.0.1,localhost,::1`; sidecars use host networking without Docker-published ports; the only browser entry remains loopback port `18090`.
- Automated acceptance: `python -m pytest tests/integration/test_sidecar_configs.py tests/unit/test_deploy_contract.py -q` returned `7 passed`; the full suite returned `130 passed`; Ruff, Compose validation and Caddy validation passed.
- Cleanup acceptance: the temporary API process, ROS1 master and project Compose containers were stopped after the live smoke. Existing business containers were left running and their original direct-run behavior remains unchanged.

## Task 6: Webots Stream Compatibility

Date: 2026-08-01

- Compatibility baseline: `start_simulation.sh` and `start_robosar_webots.sh` dry-run stdout was captured automatically against pre-change commit `99c516ed6f05c1775d51d5332d6a919e948ea1ee`. With no stream environment, both commands still match the captured output byte-for-byte.
- Default-off integration: `start_webots.launch.py` declares `webots_stream=false` and `webots_port=1234`; swarm and RoboSAR launch paths forward both values. Only platform invocations add `webots_stream:=true webots_port:=18141`, and `start_robosar_webots.sh` rejects partial, non-boolean or out-of-range input.
- Existing-project regression: `ROS2swarm_w/tests/test_webots_start_scripts.py` returned `15 passed` on the host and `15 passed` in `masar_ros2_bridge-w`. The launch package rebuilt successfully in the Humble container and `ros2 launch ... --show-args` exposed both new arguments. Shell syntax and Python compilation passed.
- Native server correction: Webots R2025a source fixes its TCP server to `QHostAddress::Any` and exposes no bind-host option. An attempted process-level bind rewrite was rejected because repeat launches could exit `139`; that implementation and its artifact were removed. The accepted design leaves Webots native behavior intact and runs a dedicated `NET_ADMIN` guard that installs exact IPv4 and IPv6 INPUT rules rejecting port `18141` from non-loopback interfaces.
- Network acceptance: `webots-guard` and Caddy were healthy; both firewall rules were present. Direct loopback TCP worked, Caddy `/webots/` returned WebSocket `101`, and a connection from an independent Docker bridge namespace to `202.204.62.208:18141` was rejected. A stop/start lifecycle check proved the exact rules are removed on graceful stop and restored before health becomes ready. Caddy `127.0.0.1:18090` remains the only browser entry.
- Browser acceptance: the R2025a Webots viewer reached `ready` with a `1280x720` canvas. The pre-interaction screenshot contained `1755` colors; drag changed viewpoint position/orientation, wheel changed position again, and the before/after SHA-256 values differed. Chromium reported no JavaScript/network errors; GPU readback performance warnings are non-functional diagnostics.
- Dependency boundary: the acceptance-only HTML loads the exact R2025a viewer from Cyberbotics. The production React task must vendor or lock those assets and may not depend on an unpinned runtime CDN.
- Automated acceptance: the platform suite returned `134 passed`; Ruff `0.9.2`, `compileall`, Compose validation, Caddy/guard health, Bash syntax, Python compilation and `git diff --check` passed.
- Cleanup acceptance: all temporary Webots/ROS launch process groups and the rejected bind library were removed. Caddy and the Webots firewall guard remain as healthy platform services; both original business containers remain running.

## Task 7: Gazebo/RViz/noVNC Displays

Date: 2026-08-01

- Display isolation: Gazebo uses X display `:101`, VNC `19131` and noVNC `18131`; RViz uses X display `:102`, VNC `19132` and noVNC `18132`. Caddy exposes only `/display/gazebo/` and `/display/rviz/` from the existing loopback gateway.
- Lifecycle ownership: Xvfb, openbox, x11vnc and websockify run in separate owned process groups. Every component PID, PGID and `/proc` start tick is atomically persisted as soon as it starts; shutdown is bounded and refuses a reused or otherwise unverifiable PID.
- Network acceptance: the rebuilt sidecar remained healthy; both Caddy pages returned HTTP 200. `ss` showed no temporary ROS/Gazebo ports and showed no wildcard display listener: noVNC was IPv4 loopback-only and VNC was restricted to `127.0.0.1` and `::1`.
- Browser acceptance: real Gazebo and RViz applications connected simultaneously through the committed dual-panel fixture. Both `1280x720` canvases contained non-empty rendered pixels; focusing either iframe did not focus the other. A simulated panel exit changed only that panel to `ERROR`.
- Scenario acceptance: group-behavior and response-controller profiles omit RViz. Exploration and rescue profiles start the repository's exact `multi_robot.rviz` command as an optional Agent-owned process on `:102`; an RViz start/exit error emits `DISPLAY_ERROR` without failing the ROS run.
- Native compatibility: all five ROS1 scripts and both ROS2 scripts passed their original container dry-run entry points with platform-only environment removed. Output reported `platform_mode=0`, preserving Qt, RViz, VGL and native defaults outside platform mode.
- Automated acceptance: the Task 7 suite returned `10 passed`; the platform suite returned `144 passed`; the existing ROS2 Webots suite returned `15 passed`. Ruff `0.9.2`, `compileall`, Compose validation, Docker image rebuild, `git diff --check` and the two Caddy HTTP probes passed.
- Dependency lock: the display image uses digest-pinned Ubuntu 22.04, noVNC `v1.7.0` and websockify `v0.13.0` from commit-addressed tarballs with verified SHA-256 values.
- Integrated implementation commit: `8b202f8`.

## Task 8: React Workbench and Responsive Layout

Date: 2026-08-01

- TDD evidence: the layout suite first failed because `/src/main.tsx` and all panels were absent. The server environment also exposed two infrastructure constraints that are now explicit in test config: loopback must bypass the host proxy, and Vite uses low-frequency polling to avoid the shared server's file-watcher limit.
- Module acceptance: the workbench exposes exactly three primary tracks: group behavior/control validation, exploration-only operation, and the complete exploration-plus-response rescue system. Group behavior has simulation, trajectory, sensors, health and logs without RViz or invented behavior scores; exploration and rescue show simulation, RViz, map and sensors simultaneously.
- Layout acceptance: at `1440x900`, all four exploration/rescue core panels were visible inside the viewport. At `390x844`, the toolbar was exactly 56 px, document width was exactly 390 px, and the four panels were ordered without overlap or horizontal overflow.
- State acceptance: one external store owns the active `run_id`, scenario, parameters, event cursor and panel state. Resize, maximize, restore and popout caused no `POST /runs`; popout and refresh restore the same run through URL/current/localStorage lookup and cursor persistence.
- Truthful-data acceptance: `UNAVAILABLE`, `WAITING`, `ONLINE`, `STALE`, `TYPE_MISMATCH` and `DISCONNECTED` render explicitly. An absent camera shows `未提供`; empty maps contain no decorative robot positions, and lifecycle progress is zero until backed by a real run state.
- Browser acceptance: controlled Chromium reviewed group behavior, exploration, rescue and mobile layouts through the production Caddy origin. The three desktop screenshots had no horizontal overflow; exploration/rescue core panels ended within the 900 px viewport; the console contained no errors.
- Gateway acceptance: Caddy serves the built React bundle at `/` with SPA fallback after all reserved proxy handlers. The root, scenario API, Gazebo noVNC and RViz noVNC returned HTTP 200; Compose API, Caddy and display services remained healthy.
- Dependency acceptance: React `19.0.0`, TypeScript `5.7.2`, Vite `6.4.3` and Playwright `1.62.1` are exact-lock dependencies. The originally planned Vite `6.0.7` and Playwright `1.49.1` were rejected after npm reported high-severity advisories; `npm audit --audit-level=moderate` now reports zero vulnerabilities.
- Automated acceptance: frontend `7 passed`; `npm run typecheck`, `npm run build`, Compose config and Caddy validation passed. The Python suite remained `144 passed`; Ruff, compileall and `git diff --check` passed.
- Integrated implementation commit: `5a54a9b`.

## Task 9: Robot Web Tools Adapters

Date: 2026-08-01

- TDD evidence: the final review introduced six regressions before their fixes. The suite first reproduced missing reconnect, registry replacement after rediscovery, a leaked health interval, camera/topic health interference, a large LaserScan call-stack overflow and unchanged snapshots causing 500 ms global store emissions.
- Connection acceptance: the browser owns at most one read-only roslib connection per ROS graph, reconnects with bounded backoff and preserves active subscriptions when rosapi resolves the same exact graph/topic/type contracts. Panels cannot publish; browser control remains on the authenticated REST command plane.
- Discovery and sidecar acceptance: ROS1 and ROS2 expose only exact `rosapi/topics` service discovery while publish globs/capabilities remain disabled. The prior Compose quoting error was removed, all services retain exact `NO_PROXY/no_proxy=127.0.0.1,localhost,::1`, and both rebuilt sidecars remained healthy.
- Rendering acceptance: the current browser adapters rendered real ROS1/ROS2 data in Chromium with the task-allocation image, world-coordinate SVG trajectory and selected-robot sensors. `TfClient` accepts only explicit safe frames at 10 Hz; no panel requests a full TF tree.
- Sensor acceptance: one same-origin `web_video_server` request is active at a time and the previous URL is cleared synchronously. LaserScan, Odometry and Twist payloads are shape/finite-value checked; a 200,000-beam LaserScan regression passes without spreading values onto the JavaScript call stack.
- Truthful-health acceptance: WAITING/ONLINE/STALE/TYPE_MISMATCH/DISCONNECTED, observed Hz and message age come from actual transport observations. MJPEG reports only browser-observable load/error connection state; it does not fabricate per-frame timestamps. Releasing the final topic subscription stops health polling.
- Security deviation: `ros3d` 1.1.0 and the unused ROS2D/EaselJS adapter were removed; `npm audit --audit-level=moderate` reports zero vulnerabilities.
- Automated acceptance: adapter unit tests returned `17 passed`; standard Playwright returned `7 passed, 2 skipped`; environment-gated live ROS Playwright returned `2 passed`; the Python suite returned `144 passed`. TypeScript, Vite build, Ruff, compileall, Compose validation, shell syntax, Caddy health, diff checks and npm audit passed. The Vite build retains a non-blocking 506 kB chunk-splitting advisory.
- Cleanup acceptance: live ROS1/ROS2 publishers were stopped by exact container PID and verified absent. Existing business containers and native ROS project commands were not modified.
- Integrated implementation commit: `9587ac2`.

## Task 10: Artifact Recording and Download

Date: 2026-08-03

- TDD evidence: artifact/download modules first failed because their contracts were absent. Later review regressions separately reproduced non-transactional legacy migration, live-bag indexing after an unconfirmed stop, blocking file hashing, incomplete checksum coverage, missing recorder/display ownership and loss of display state across API restart before each correction.
- Archive acceptance: terminal and failed-start runs atomically contain the original request, resolved YAML, complete events, process logs, per-Agent and aggregate ROS graph snapshots, fixed manifest/summary documents and GNU-format checksums. Archive conditions persist in SQLite, retries verify exact index/checksum coverage, and recovery cannot erase `ownership_unproven`, `shutdown_unconfirmed` or unsettled-bag evidence.
- Recording acceptance: the browser supplies only `{enabled, profile}`. Recorder commands and anchored topic regexes are derived from the scenario data/artifact profiles; the Agent rejects `-a`, `/.*`, arbitrary topics and cross-profile command refs. Recorders stop before RViz and business steps, and bags are indexed only after ownership records prove a terminal process state.
- Download acceptance: duplicate content may share a content-derived `artifact_id` while `(run_id, path)` remains the database identity. Downloads open every directory component and the final file with `O_NOFOLLOW`, verify regular-file type, size and full SHA-256 on the same descriptor in a worker thread, then stream full or single-range responses with checksum and range headers. Traversal, symlink, multi-range and post-index tamper tests pass.
- Recovery acceptance: legacy artifact migration uses a verified `BEGIN IMMEDIATE` transaction and safely resumes an interrupted `artifacts_legacy` migration. Optional RViz start state is persisted in `run_steps`; successful or uncertain starts require shutdown proof after restart, explicit pre-start 4xx rejection remains optional, and pre-upgrade active runs are conservatively backfilled from Agent records.
- Container acceptance: isolated Noetic master `/percent_completed` recording created a valid ROS1 bag; isolated Humble domain 98 `/global_info` recording created `metadata.yaml` and a non-empty `.db3`. Both signed graph routes reported the real topics, both recorders stopped with SIGINT, and exact temporary PIDs, secrets and run directories were verified absent afterward.
- Integrated acceptance: the narrow suite returned `18 passed`; the full suite returned `176 passed`; Ruff, compileall, Compose validation, Bash syntax and diff checks passed. The final API image was rebuilt, remained healthy and migrated the existing Compose SQLite volume. Frontend regression returned `17 passed` unit tests and `7 passed, 2 skipped` Chromium tests; TypeScript, Vite production build and npm audit passed.
- Review and scope: final independent review reported no P0/P1/P2 findings. No file in `masar_ws`, `ROS2swarm_w`, `multi-agent-neat` or the abandoned platform was changed.
- Integrated implementation commit: `fccb40e`.

## Stage A Baseline: Runtime Lifecycle and Performance Closure

Date: 2026-08-02

Status: `PASSED`. Runtime lifecycle, Webots performance, Gazebo baseline and full regression gates are closed; Stage B may start.

Closure thresholds: after warm-up, 30-second real-time factor `>=0.90` for three Burger robots and `>=0.80` for the default five-Burger case; NVIDIA renderer; changing non-empty noVNC canvas with working drag/zoom; two consecutive clean swarm restarts; one clean exploration Webots Run; zero owned residuals. Missing any threshold keeps this stage open.

Current platform graph identity: all six platform manifests, the ROS2 Web sidecar and standalone `ROS2swarm_w/start_simulation.sh` use ROS2 domain `42`.

- Fresh real Run `run-911043f496e84702ac0fbfdccff4a20e` used three Burger robots, `command_world.wbt` and `aggregation_pattern`. Webots reported about `805% CPU`; `/clock` advanced 8.66 seconds over 19.09 wall seconds, giving an observed real-time factor near `0.45`.
- The active Webots container display rendered through Mesa `llvmpipe`; `vglrun -d :0 glxinfo -B` independently proved the RTX 6000 Ada OpenGL path is available. The current Run did not appear as a Webots GPU graphics workload in `nvidia-smi pmon`.
- ROS2 published `/clock`, `/global_info` and `/robot_0..2/{odom,scan,cmd_vel}`. Robot 0 odometry changed from `(0.188,-0.035)` to `(0.848,-0.416)`, while the live browser still rendered waiting trajectory, sensors and graph health, an instruction-only log panel and a blank W3D area.
- Stop returned `COMPLETED/USER_STOP`. The Run left no Webots/launch/behavior child and no `18141` listener; both business containers remained running; all 11 archive checksum entries passed.
- Runtime Agent health was absent after the business containers had restarted and required manual bootstrap. This is now a Stage A startup-closure requirement.
- `ros1_bridge` is installed under `/root/ros-humble-ros1-bridge/install`, but platform non-interactive startup does not source it. The historical `Package 'ros1_bridge' not found` is therefore an environment propagation defect. Historical controller switch failures and Webots exit `139` remain open until the Stage A fixes pass consecutive real runs.
- The superseding swarm Runs `run-3d72df0252b04936ad5522b3e72ca22d` (three Burger robots) and `run-e0e871a6d0f0471e81ccfa247d2d30db` (default five Burger robots) reached `RUNNING` with measured real-time factors `0.980` and `0.940`. Both rendered changing trajectory, LaserScan/health data and an interactive NVIDIA-backed Webots noVNC framebuffer, then stopped as `COMPLETED/USER_STOP` with no Run-owned residual process or Webots port.
- Webots exploration Run `run-ccceea6a7b254acea82f0db22e536a17` proved the required two-step order: ROS2 `start_robosar_webots.sh` reached readiness before ROS1 `launch_robosar_w.sh` started. The browser rendered the real Webots scene, RViz framebuffer, `/sim_map`, changing ROS2 LaserScan and online graph health. The live Playwright gate now samples framebuffer colors and luminance instead of accepting a non-empty PNG byte count, so an empty noVNC desktop cannot pass.
- Exploration stop events proved the reverse order `display_rviz`, `ros1_robosar`, `ros2_webots_bridge`. PGIDs `6231`, `2783` and `30287`, Webots, `dynamic_bridge`, all Run-tagged processes and port `1234` were absent afterward. Both business container IDs and start timestamps were unchanged. The archive indexed 16 artifacts and `sha256sum -c checksums.sha256` verified every covered file.
- Domain acceptance is both static and live: all six manifests, the ROS2 Web sidecar and native `ROS2swarm_w/start_simulation.sh` use domain `42`; the sidecar plus both ROS1/ROS2 process groups in the exploration Run carried `ROS_DOMAIN_ID=42`. The focused configuration/Agent/sidecar contract suite returned `55 passed`.
- Gazebo exploration Run `run-287b2dd9af264a0597cc9ec31dc188a1` reached `RUNNING`, accepted the existing START mission command and exposed real `/sim_map`, `/task_allocation_image`, `/percent_completed`, emergency-object/event topics plus five robots' camera, LaserScan and odometry topics. A 30.03 s wall-clock sample advanced simulation time by 26.94 s (RTF `0.897`). The ROS1 business container used about `4405%` CPU during the sample, so the remaining Gazebo slowdown is primarily simulator/ROS workload rather than browser transport.
- The Gazebo live Chromium gate rendered non-blank Gazebo and RViz framebuffers, the OccupancyGrid and a real camera stream (`1 passed`). Stop completed as `COMPLETED/USER_STOP`; both Agent records were `STOPPED`, no process carried the Run id, `current` became null, both business container IDs/start times stayed unchanged and all 13 archived checksum entries passed. The platform display sidecar's Xvfb/noVNC gateways remain running by design between Runs; no Run-owned Gazebo or RViz application remains.
- A separate approximately 20-minute historical Gazebo Run `run-39c3cc17403848b79aadd2239f5d3fbc` ended after an existing Gazebo/task-process `free(): invalid pointer`. The control plane correctly classified and cleaned that exit. The fresh Stage A Run did not reproduce it in the acceptance window; long-duration Gazebo stability remains an upstream `masar_ws` risk to isolate after the platform data panels close.
- Final regression used the project `.venv`: platform Python `224 passed`, focused lifecycle/contracts `62 passed`, Ruff and compileall passed; frontend unit `17 passed`, production TypeScript/Vite build passed and Chromium returned `14 passed, 4 live-only skipped`. Original-project regression returned ROS1 `90 passed`, ROS2 `22 passed`, and all seven direct-entry dry-runs passed with `platform_mode=0`; the native swarm script retained `roundflocking_pattern` and Domain `42` defaults.
- The final API image `sha256:567c28d6cef9040044a5dadddf4004a013323be0bf4c314383e0eaddefbd6786` was built from source and the recreated container is healthy. Both Runtime Agents were restarted by exact PID and load the canonical `runs/agent-ros1|agent-ros2/<run_id>` layout. Exploration recorder allow-lists now include the same four per-agent scan/camera/odometry/cmd topics declared by the data profile; the exact-regex contract passed.

## Stage B Acceptance: Group Behavior Panels

Date: 2026-08-03

Status: `PASSED`. Group-behavior layout, real data, command delivery, lifecycle and full regression gates are closed; Stage C may start.

- Real Webots Run `run-92f9915344244ac492a0ff06d922dc5e` used three e-puck robots, `command_world.wbt` and `aggregation_pattern`. It remained `RUNNING` beyond readiness, showed changing odometry, a non-empty interactive noVNC framebuffer, real LaserScan/camera data, all robot health rows and Run-owned stdout/stderr.
- The group-behavior desktop workspace places Webots above the equal-width trajectory and sensor panels; at 390x844 the same panels remain ordered in a non-overlapping single column. The full Chromium suite returned `21 passed, 5 live-only skipped`; its layout regression includes the desktop geometry and mobile ordering checks.
- The group-behavior parameter contract now fixes the only supported world to `command_world.wbt`; the world is not rendered as a selector. The real `18090` page showed a visible robot selector with exactly `e_puck` and `burger`, desktop panel geometry matched the upper-simulation/lower-two-panel contract, and the browser console had no errors. Screenshot: `.superpowers/visual-review/stage-b-live/stage-b-fixed-world-robot-layout.png`.
- The Run stopped as `COMPLETED/USER_STOP`; `/runs/current` became null, port `18141` and Run-owned Webots/ROS2 children disappeared, both business containers retained their original start timestamps and restart count `0`, and all 11 files covered by `checksums.sha256` verified successfully.
- Real discussed-dispersion Run `run-2d6a2d6a61484aafa1c3c31ca156f524` proved that one platform `mission.start` reaches all 12 current `/swarm_command` subscribers. All three e-puck opinion topics then published at `0.5 Hz`; the real browser gate passed selected-robot camera, eight infrared channels, discussion state, three trajectories, three health rows, Run logs and a non-empty Webots framebuffer.
- The command-delivery fix first reads the current ROS2 subscriber count, then publishes with `--wait-matching-subscriptions`, explicit volatile/reliable QoS and bounded keep-alive. Its regression test failed against the former direct `--once` implementation and passed after the fix. A missing or unparseable subscriber graph now fails the command instead of reporting false success.
- The discussed Run stopped as `COMPLETED/USER_STOP`; current became null, no Run-owned Webots/ROS2 process or `18141` listener remained, container start timestamps and restart counts were unchanged, and all 11 archived files passed checksum verification.
- Final gates returned platform Python `229 passed`, Ruff and compileall success, frontend unit `22 passed`, production TypeScript/Vite build success, ordinary Chromium `21 passed, 5 live-only skipped`, Stage B live Chromium `1 passed` from the real discussed-dispersion Run, ROS2 project `22 passed`, Compose validation success and a clean platform-directory diff check. The API image was rebuilt with the fixed-world manifest; only the platform API container was recreated, while both ROS business containers retained their start timestamps and restart count `0`.
- Both `e_puck` and `burger` remain catalogued platform choices. Burger Webots lifecycle/performance was proven in Stage A, while the user-confirmed Burger `aggregation_pattern` behavior defect also reproduces outside the platform and remains deferred to ROS2swarm algorithm work; the platform does not hide or reinterpret that upstream behavior.
- The deployed `/runs/{run_id}/robots` route now returns the resolved Run inventory instead of `[]`. Historical five-Burger Run `run-d1c33177dcc04214bf2be03f05a257e7` returned five `robot_N` entries with model and topic prefix; this inventory is explicitly expected state, not fabricated ROS health.
- Five-e-puck deployment Run `run-4fdfe4f599094947bd10cefb97e2e064` reached `RUNNING` in 70 seconds with a single Domain 42 rclpy readiness batch observing `/clock` and all five odometry topics. The real Chromium gate selected robot_2 and passed its camera, LaserScan, eight infrared channels, all five health rows, multi-robot SVG trajectory, Run logs and the Webots framebuffer. Screenshot set: `.superpowers/visual-review/stage-b-final/`.
- A first manual acceptance request used ROS2swarm `sensor_type=ir`, unlike the platform UI default `lidar`. All five upstream `ir_layer` processes rejected a trailing-slash topic and two controllers failed to activate, so the Run correctly ended `FAILED/READINESS_TIMEOUT`. The default `lidar` path still publishes the physical e-puck `ps0..ps7` infrared topics consumed by the panel; the broken optional ROS2swarm layer is deferred with the other upstream behavior work.
- The first browser attempt against the valid Run reproduced a separate sidecar failure: the rosbridge listener had 36 queued connections and a direct WebSocket handshake timed out while direct `/rosapi/topics` remained healthy. Enabling `call_services_in_new_thread:=true` isolated rosapi calls from the WebSocket loop; after rebuilding only `ros2-web`, the handshake returned `101` immediately and the same live Chromium test passed. Neither business container restarted.
- The final stop produced `COMPLETED/USER_STOP`, `current=null`, Agent status `STOPPED`, no Run-owned process and no `18141`/`1234` listener. All 11 archived checksum entries passed. Final regression returned platform Python `234 passed, 2 environment-skipped` plus ROS2-container smoke `2 passed`; frontend unit `28 passed`; ordinary Chromium `24 passed, 6 live-only skipped`; real Stage B Chromium `1 passed`; ROS2swarm `23 passed`; production build, Ruff, compileall, shell syntax, Compose validation and platform diff checks passed.

## Stage C Acceptance: Exploration Panel Semantics

Date: 2026-08-03

Status: `PASSED`. Exploration panel semantics, Gazebo/Webots two-cycle lifecycle, real browser data, cleanup, archive and full regression gates are closed; Stage D may start.

- Real Gazebo Run `run-1ba9ef43618447c2b3653e526119ee66` established the data semantics before implementation: `/percent_completed=0.4578248858451843` means about `45.8%`; the latest `/task_allocation` contained two newly assigned robots; candidate frontiers were `10`; five exploration robots were online; the emergency snapshot contained three victims, one fire and two push-obstacles.
- A late subscriber received the latched task-allocation image, but the browser's first MJPEG request could receive HTTP 200 with an empty multipart body before the allocator's first image. Chromium then emitted no useful `load` or `error`. The panel now reconnects every 1.5 seconds while the Run is active and `naturalWidth` remains zero.
- `任务分配与建图` now exclusively owns the Qt allocation image, map coverage, candidate-frontier count and latest allocation batch labeled `本轮分配`. `探索任务态势` exclusively owns online exploration robots and observed emergency-object counts, positions and detector IDs. It no longer duplicates progress/allocation or displays `已完成任务`.
- The default HIGH allocator creates a `/tasks_completed` publisher but emitted no messages in a bounded late-subscriber probe. That capability was removed from exploration, exploration_webots and rescue profiles so it no longer appears as a meaningless waiting row. Emergency counts remain `—` until a real snapshot; only a real empty snapshot renders `0`.
- TDD evidence: three focused browser tests first failed for the missing coverage element, fabricated silent allocation zero and absent no-event first-frame retry. A profile test separately failed while `/tasks_completed` remained catalogued. After the minimal changes, focused panel Chromium returned `20 passed`, adapter units `30 passed`, profile tests `8 passed`, TypeScript and production build passed, and ordinary Chromium returned `27 passed, 7 live-only skipped`.
- Failed live gate `run-37e0e2076e0a4402aca51349e0467bf6` exposed a sidecar boundary defect: the browser sent exact subscriptions for latched `/task_allocation` and `/frontier_filter/filtered_frontiers`, but rosbridge delivered only standard-message `/percent_completed`. The ros1-web container could not find or import `robosar_messages`, so task/frontier/agent/emergency custom messages remained discovered but unsampled. The Run stopped as `COMPLETED/USER_STOP`, left no Run-owned process, preserved both business container timestamps/restart counts and passed all 13 archive checksum entries; it does not count toward the two required successful Gazebo cycles.
- The ROS1 sidecar now mirrors the existing ROS2 overlay contract: `<workspace>/masar_ws` is mounted read-only at `/root/workspace/masar_ws`, and every sidecar process requires/sources `devel/setup.bash`. The deployment test first failed on both missing contracts and then returned `5 passed`; Compose validation and Bash syntax passed. The rebuilt sidecar imported `task_allocation`, `PointArray`, `EmergencyObjectArray` and `agents_status`, and a real-browser temporary custom-message smoke received the full payload. Its bounded publisher and topic were absent afterward; neither business container restarted.
- Failed Gazebo candidate `run-d6d8007aac864d6bae4bfcfde8e14179` isolated a second first-frame defect. Its trace contained exactly one HTTP 200 empty multipart request even though React advanced the watchdog state for 60 seconds; later ROS inspection showed a latched `640x480` image and an active `web_video_server` subscriber. Reassigning the same `src` did not force Chromium to request again. The Run was stopped as `COMPLETED/USER_STOP`, both Agent records were `STOPPED`, no Run process remained, all 13 checksums passed and both business containers were unchanged; it does not count toward Gazebo acceptance.
- The replacement regression intercepts the real browser route rather than the `HTMLImageElement.src` setter. It first failed with one request, then passed after watchdog retries gained a changing `retry=N` query while initial allocation and ordinary camera URLs stayed unchanged. Adapter units returned `31 passed`, focused panels `20 passed`, TypeScript and production build passed, and ordinary Chromium returned `27 passed, 7 live-only skipped`.
- Gazebo valid cycle 1 `run-1e6dc13688634b0bb9bae467ffdb8ceb` passed the real Chromium panel gate and stopped as `COMPLETED/USER_STOP`; both Agent records were `STOPPED`, no Run/Gazebo/RViz/task process or Run-only port remained, all 13 checksums passed and the business containers retained their original timestamps and restart count `0`.
- Gazebo valid cycle 2 `run-4ba8c33b877149a8acf8e1da26caef8b` rejected the pre-`RUNNING` START with `409 RUN_NOT_ACTIVE`, then passed the live Chromium gate in 1.7 minutes. The captured page showed `36.3%` coverage, five candidate frontiers, one current allocation, `5/5` online exploration robots and one discovered push-obstacle with its position, together with nonblank Gazebo, RViz, allocation and selected-robot camera views. Stop produced `COMPLETED/USER_STOP`, both Agent records were `STOPPED`, no owned process or `18141`/`1234` listener remained, all 13 checksums passed and neither business container restarted. Screenshot set: `.superpowers/visual-review/stage-c-gazebo-cycle2/`.
- Diagnostic Webots Run `run-f02a8da600e548ec9e7ae5a5ae7ebea7` confirmed the required launch order from durable events and process timestamps: ROS2 `start_robosar_webots.sh` reached readiness before ROS1 `launch_robosar_w.sh` started. Real Webots sensors exposed all five `/robot_i/camera/image_color`, `/scan`, `/odom` and `/cmd_vel` sources, while silent ROS1 odometry duplicates shadowed live ROS2 odometry in first-match panel selection. A RED unit test reproduced that source-selection defect; the focused fix and deterministic Webots panel regression then passed with live ROS2 pose, health and `5/5` online count.
- The same diagnostic Run isolated the remaining task-image transport defect. Its trace showed 40 distinct HTTP 200 MJPEG requests aborted at the 1.5-second watchdog interval with `naturalWidth=0`; direct curl received one valid 29 KB multipart JPEG but the low-frequency latched source emitted no next boundary for Chromium to commit. The existing same-origin snapshot endpoint returned the complete JPEG in about 7 ms. This Run does not count toward Webots acceptance. It stopped in reverse order with all three Agent records `STOPPED`, no Run process or `18141`/`1234` listener, all 16 archived checksums valid and unchanged business containers.
- The allocation image now uses bounded `/video/ros1/snapshot` requests: it retries every 1.5 seconds until the first complete JPEG and refreshes every 3 seconds afterward. Robot cameras retain MJPEG streams. Source selection ranks real availability so an online ROS2 Webots odometry source cannot be hidden by a silent ROS1 duplicate, and the online robot count prefers live per-robot odometry over stale agent-status snapshots. Network-level snapshot retry/refresh, duplicate-source priority, real-time online-count and simulator-specific camera regressions passed; adapter units returned `33 passed`, focused panels returned `22 passed`, TypeScript and the production build passed.
- Webots valid cycle 1 `run-919530af7fae462893dfc5ebfff3bd7a` passed the live Chromium gate. The page rendered nonblank Webots and RViz canvases, a complete task-allocation snapshot, coverage `56.2%`, six candidate frontiers, one current allocation, `5/5` online robots, one fire and two push-obstacles, along with all selected-robot sensor views. Durable events prove ROS2 readiness preceded ROS1 startup and stop used the reverse RViz, ROS1, ROS2 order. The Run ended `COMPLETED/USER_STOP`; all three process records were `STOPPED`, no Run process or `18141`/`1234` listener remained, all 16 archived checksums passed and neither business container restarted. Screenshot set: `.superpowers/visual-review/stage-c-webots-cycle1/`.
- Webots valid cycle 2 `run-61aa650ac5814725bb95fcc1f950b284` rejected START during `PREFLIGHT` with `409 RUN_NOT_ACTIVE`, reached `RUNNING` after ROS2 readiness and then ROS1 readiness, and passed the live Chromium gate in 41.6 seconds. The captured page showed coverage `23.4%`, eight candidate frontiers, one current allocation, `5/5` online robots and one fire, with all five selectable sensor views plus nonblank Webots and RViz canvases. Stop used the reverse RViz, ROS1, ROS2 order and produced `COMPLETED/USER_STOP`; all three process records were `STOPPED`, no Run process or `18141`/`1234` listener remained, all 16 archived checksums passed and both business container IDs, start times and restart counts were unchanged. Screenshot set: `.superpowers/visual-review/stage-c-webots-cycle2/`.
- Final regression returned platform Python `235 passed, 2 environment-skipped` plus ROS2-container smoke `2 passed`; frontend adapters `33 passed`; ordinary Chromium `29 passed, 7 live-only skipped`; ROS1 project `90 passed`; ROS2 project `23 passed`. TypeScript, production build, npm audit, Ruff, compileall, Compose validation, Bash syntax and platform diff checks passed. All seven existing ROS entry-point dry-runs returned success with `platform_mode=0`. The regression exposed and removed the stale `/tasks_completed` Runtime Agent recorder allow-list entry so Agent and control-plane exact regex contracts remain identical.

## Stage D Acceptance: Complete Rescue and Documentation Closure

Date: 2026-08-03

Status: `PASSED`. Complete-rescue data, compact response state, lifecycle ownership, responsive right-rail layout, documentation and full regression gates are closed.

- Real Gazebo/Tepper Run `run-8adea267bc8f43f4b89d396b5096f1f6` ran the five-robot exploration group and four-Burger response group together. Real Chromium showed nonblank Gazebo and RViz desktops, the Qt allocation image, map coverage, candidate frontiers, latest allocation batch, selected-robot sensors, online exploration robots, emergency objects and changing response-group values.
- The response summary contains exactly six operator-critical items: combined response/planning state, motion enabled, active task, task queue, formation error and minimum clearance. Behavior mode, formation scale, response robot count and latest result are not repeated in the default right rail. Browser geometry checks proved the last response value remains inside its panel and the following health panel starts after it.
- TDD reproduced the final layout defect before the fix: at `1440x1100` the last status panel ended `200px` above the rail bottom, and after scrolling a six-row object feed its last row ended `51px` below the mission panel. After the minimal flex/minimum-height change, both focused Chromium regressions passed.
- Production Chromium at `1440x1100` measured right-rail `clientHeight=scrollHeight=964`, a `0px` bottom gap and mission/response/ordinary panel growth without overlap. At `1440x900` the rail measured `clientHeight=764`, `scrollHeight=875` and `overflow-y:auto`; at `1440x760`, scrolling to `scrollTop=251` aligned the last panel bottom exactly with the rail bottom. No browser console warning or error was observed. Screenshots: `.superpowers/visual-review/stage-d-rescue/layout-1440x1100.png`, `layout-1440x900.png` and `layout-1440x760-scrolled.png`.
- The Run rejected START during `PREFLIGHT` with `409 RUN_NOT_ACTIVE`; after `RUNNING`, task control succeeded. Stop completed as `COMPLETED/USER_STOP` in recording, RViz, then `tepper_response` order. All three Agent records were `STOPPED`, `current=null`, no Run-owned process or `1234/18141` listener remained, all 16 archived checksum entries passed, and neither business container restarted.
- Final Stage D regression returned platform Python `236 passed, 2 environment-skipped` plus ROS2-container smoke `2 passed`; frontend adapters `34 passed`; ordinary Chromium `34 passed, 8 live-only skipped`; ROS1 project `90 passed`; ROS1 task-allocation tests `5 passed`; ROS2swarm `23 passed`. TypeScript, build, npm audit, Ruff, compileall, Compose validation, Bash syntax, clean diff checks and all seven original ROS entry-point dry-runs passed. Its deployment startup evidence is historical and superseded by Compose closure E1-E4.
- `docs/OPERATIONS.md` is the single operator manual for prerequisites, startup, SSH forwarding, module controls, right-rail scrolling, Run stop semantics, complete shutdown, artifacts and troubleshooting. `docs/SPEC.md`, `docs/PLATFORM_DESIGN.md` and `docs/IMPLEMENTATION_PLAN.md` carry the same Stage D behavior and acceptance contract.

## Stage B Reacceptance: Shared World Trajectories

Date: 2026-08-03

Status: `PASSED`. Webots and Gazebo now share one world-coordinate trajectory contract; idle and terminal pages no longer expose planned robots as live state.

- The former trajectory panel subscribed to every robot's Odometry and overlaid those values directly. Webots Odometry is local to each spawn pose, so the plot did not represent real inter-robot geometry. The replacement consumes Webots `/global_info` (`std_msgs/msg/String`) from `GlobalInfoSupervisor` and Gazebo `/gazebo/model_states` (`gazebo_msgs/ModelStates`). Local Odometry remains available in the selected-robot sensor panel and is never used as a world-trajectory fallback.
- Deterministic browser regressions first failed while idle/terminal pages rendered five planned robots and while fake local Odometry drove the plot. After the fix, Webots and Gazebo tests ignored deliberately incorrect local values and rendered only their world sources. Ordinary Chromium returned `38 passed, 8 live-only skipped`; adapters returned `36 passed`; the final platform Python suite returned `238 passed, 2 environment-skipped` after adding the ROS1 sidecar dependency contract.
- Real Webots Run `run-bffc31333a064ea08e5628dc3cf87a94` used three e-pucks, `command_world.wbt`, Domain 42 and `aggregation_pattern`. A best-effort ROS2 probe observed distinct Supervisor world positions while each robot retained its own Odometry frame. Production Chromium rendered `3/3 世界位姿`, three world-coordinate traces, real sensors and a nonblank Webots framebuffer without console errors. Screenshot: `.superpowers/visual-review/world-frame-webots/stage-b-epuck-trajectory.png`.
- The first real Gazebo DEFORM Run `run-a802d0b611c2497899e610647abeadda` exposed a deployment dependency rather than a decoder defect: rosbridge discovered `/gazebo/model_states` but logged `Unable to load the manifest for package gazebo_msgs`. A RED deployment test captured the missing package. The `ros1-web` image now installs `ros-noetic-gazebo-msgs`; after rebuilding only that platform sidecar, `rospack find gazebo_msgs` and `rosmsg show gazebo_msgs/ModelStates` succeeded.
- Fixed Gazebo Run `run-c25cd415a2394c28a17397544edc9620` rendered `4/4 世界位姿`. After START, all four robots moved and production Chromium accumulated 81 points per trace from ModelStates; the displayed coordinates tracked direct `/gazebo/model_states` samples and the browser console stayed clean. Screenshot: `.superpowers/visual-review/world-frame-gazebo-trajectory-fixed.png`.
- Both accepted Runs stopped as `COMPLETED/USER_STOP`; `current=null`, no Run-owned process or `1234`/`18141` listener remained, and all 11 checksum entries in each archive passed. A production idle-page probe then measured zero trajectory rows, zero health rows, `0/0 世界位姿`, and only the “未发现机器人” sensor option. Both business containers kept IDs `6b81315c...` and `a812923f...`, their original start timestamps, and restart count `0`.

## E1 Compose-Managed Automated Gate

Date: 2026-08-03

Status: `PASSED`. Real container migration may proceed to E2.

- Both public v2 `linux/amd64` image digests were resolved and recorded. The
  current host already has matching local images, so migration acceptance must
  prove no registry pull occurs.
- TDD first produced `10 failed` because the two runtime services,
  `deploy/platform.sh` and Make deployment targets did not exist. The focused
  controller and deployment suite now passes, including local digest reuse,
  missing-image pull, Node-container frontend build, first overlay ordering,
  exact legacy-name migration, unrelated-port refusal, idempotent start and
  active-Run stop before Compose down.
- Mutable platform code and configuration are bind-mounted into API, sidecar,
  display, guard and Caddy services. A regression test reproduced non-executable
  bind-mounted sidecar scripts before their host modes were corrected.
- Obsolete deployment artifacts removed so far: the external-container startup
  script, the Caddy frontend-build Dockerfile, and duplicate unused ROS1/ROS2
  sidecar YAML manifests. Runtime deployment code contains no old Agent
  injection or Compose profile path.
- Final gate: Python `250 passed, 2 skipped` (the two skips require the live ROS2
  Runtime Agent environment); frontend adapter unit `36 passed`; TypeScript and
  Vite production build passed. Python 3.13.5, pytest 8.3.4, Ruff 0.9.2,
  Docker Compose v5.2.0, Node v24.18.0 and npm 11.16.0 were recorded for the
  host-side test gate. Deployment itself continues to build frontend assets in
  the pinned Node 22 container.
- `compileall`, Ruff, Compose config, Bash syntax and platform-scoped
  `git diff --check -- .` all passed. The workspace-wide diff check separately
  reports pre-existing trailing whitespace in two modified ROS2 burger YAML
  files; E1 neither edits nor hides those user-owned changes.

## E2 Compose Migration and Restart Recovery

Date: 2026-08-03

Status: `PASSED`. The two public v2 runtime images are now the only business
runtime definition; the old external containers were migrated by exact name and
the full Compose lifecycle was repeated successfully.

- Before migration, `/api/v1/runs/current` was `null`, both local v2 image
  RepoDigests matched the pinned values, and the two old containers had no
  Compose project label. No unrelated container was selected.
- `make docker-start` stopped and removed only `masar_ros1_srv-g` and
  `masar_ros2_bridge-w`, created all eight Compose services and reached healthy
  state. The first attempt exposed a real bind-mount permission defect: the API
  image's `65532:65532` user could not import owner-only checkout files. The
  regression test reproduced it; the development Compose service now explicitly
  runs API as `0:0` while keeping source and scenario mounts read-only.
- After the minimal fix, the API returned `/healthz=200`, the gateway and all
  eight services were healthy, and no source copy or manual Agent injection was
  needed. The pinned ROS images were reused locally without a registry pull.
- `docker restart masar_ros1_srv-g` and `docker restart masar_ros2_bridge-w`
  each restored its Agent health endpoint automatically. The other runtime and
  the sidecars remained healthy; no `docker exec`, repair command or host-wide
  process signal was used.
- `make docker-stop` removed the eight managed containers only. The unrelated
  VitFly, FedSwarm, Deer-Flow, HN25 and other containers remained running. A
  second `make docker-start` reused the existing frontend and ROS overlays and
  again reached all eight healthy services. Fresh containers had restart count
  `0`, both Agent endpoints returned `status=ok`, and the gateway returned
  `ok`.
- E2 cleanup removed no additional active implementation. The obsolete startup
  path, duplicate supervisor YAML, Caddy build image and unused ROS2D
  `MapRenderer` are deleted; all remaining adapters and panels have live code
  references. Stage E3 may now begin.

## E3 Compose Three-Module Real Regression

Date: 2026-08-03

Status: `PASSED`. All three operator modules completed a fresh real Run using
the Compose-managed v2 runtimes and the production browser gateway.

- Group behavior Run `run-18912479403b473ea62b8007cb055d24`
  used five e-pucks, `command_world.wbt`, Domain 42 and
  `aggregation_pattern`. Live Chromium passed Webots framebuffer, selected
  robot 2 camera/LaserScan/eight infrared channels, all robot health rows,
  world-coordinate trajectories and process logs. Stop ended
  `COMPLETED/USER_STOP`; 11 files listed by `checksums.sha256` verified and no
  Run process remained.
- Webots exploration Run `run-15c024095bdc49aa965c1e33caf1115a`
  proved event order `ros2_webots_bridge READY -> ros1_robosar START/READY ->
  RViz`. Chromium passed the real task-allocation image, coverage, frontiers,
  latest allocation, five selectable robot sensor views, discoveries, Webots
  and RViz framebuffers. Stop order was RViz, ROS1, ROS2; all 16 checksum
  entries passed and ports `1234/18141` plus both Runtime-Agent process trees
  had no Run residual.
- Complete rescue Run `run-aa86de29f07543cf87533aab82ecb627`
  passed the production Chromium gate with task allocation, map coverage,
  emergency discoveries, active response task/queue, motion gate, formation
  error, minimum clearance, Gazebo and RViz. Stop ended
  `COMPLETED/USER_STOP`; all 13 checksum entries passed and no Run process or
  run-only listener remained.
- After every Run, `/api/v1/runs/current` returned `null` and all eight Compose
  services remained healthy. No runtime container restart, Agent injection or
  host-wide cleanup command occurred. The fresh-overlay and documentation closure
  evidence is recorded in E4 below.

## E4 Fresh Overlay and Documentation Closure

Date: 2026-08-04

Status: `PASSED`. A task-specific copy of the three-project workspace was built
from source with both ROS overlays absent, then removed without touching the
main workspace or unrelated containers.

- The isolated copy omitted `masar_ws/devel` and `ROS2swarm_w/install` before
  startup. `make docker-start` reused both cached v2 image digests, built the
  frontend in the pinned Node container, ran ROS1 `catkin_make` in 99 seconds
  and ROS2 `colcon build` in 14 seconds, then reached all eight healthy
  services.
- The isolated run generated `devel/setup.bash` and `install/setup.bash`.
  `build/devel/install/log` roots were `1006:1006`; representative source
  ownership and modes were unchanged. Bootstrap logs were retained as
  `.runtime/logs/e4-bootstrap-ros1.log`, `e4-bootstrap-ros2.log` and
  `e4-frontend.log` in the main checkout.
- The isolated platform was stopped through `make docker-stop`; only its eight
  Compose containers were removed. The temporary directory was moved to the
  desktop trash after an exact-path check, so it is recoverable and cannot
  affect the main workspace.
- The main checkout then passed a clean `make docker-start`, `make docker-doctor`
  and `make docker-status`: all eight services were healthy, `/api/v1/runs/current`
  was `null`, the gateway returned `ok`, and the two pinned local RepoDigests
  matched. `npm audit --audit-level=moderate` found zero vulnerabilities.
- Stale-reference scans found no active `zjc2024` image, old startup script,
  Agent injection, `dev-api`, hardcoded workspace path or removed renderer
  reference. The current guide is `docs/OPERATIONS.md`; historical evidence is
  explicitly dated and marked superseded.
- Final gates: Python `251 passed, 2 skipped`; frontend unit `36 passed`;
  ordinary Chromium `41 passed, 8 live-only skipped`; TypeScript, Vite build,
  Ruff, compileall, Compose config, shell/Python syntax and `git diff --check`
  all passed. E4 and Compose-managed closure are complete.

## Stage F Acceptance: Portable Workspace, Unified SSH Entry and Documentation Layout

Date: 2026-08-04

Status: `PASSED`. The deployment contract no longer depends on a host-specific checkout
path, the browser and SSH entry use the same port, and project process documentation is
organized under `docs/` with a root README quick start.

- Workspace acceptance: `deploy/platform.sh` derives the parent of the current platform
  checkout when no override is set, normalizes an explicit `MASAR_WORKSPACE_ROOT`, rejects
  an invalid root before Docker is called, and preserves `/root/workspace` plus the three
  canonical project paths inside both runtime containers. Focused startup/deployment tests
  returned `21 passed`; a real read-only `make docker-doctor` passed.
- SSH acceptance: start/status output now reports `http://127.0.0.1:18090` and
  `LocalForward 18090 127.0.0.1:18090`. No second platform URL, SSH process, listener or
  proxy port was added. Focused platform startup tests returned `13 passed` and a live
  `make docker-status` showed the same mapping.
- Documentation acceptance: `README.md` contains the project summary and quick start;
  `docs/OPERATIONS.md`, `docs/SPEC.md`, `docs/PLATFORM_DESIGN.md`,
  `docs/IMPLEMENTATION_PLAN.md` and this ledger are the current process documents.
  The document contract returned `4 passed`; the active-reference scan found no
  host-specific workspace path, alternate browser port or unrelated proxy port.
- Full regression: platform Python returned `258 passed, 2 skipped`; frontend unit tests
  returned `36 passed`; ordinary Chromium returned `41 passed, 8 skipped`; TypeScript,
  Vite build, Ruff, compileall, Compose config, Bash syntax, npm audit and scoped diff
  checks passed. Gateway health returned `ok`, all eight Compose services were healthy and
  `/api/v1/runs/current` returned `{"run":null}`. No new simulation Run was started.
