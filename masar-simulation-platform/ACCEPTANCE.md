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
| 3.5. Integration Contracts | PASS | full suite `103 passed`; Ruff, compileall, Compose build/health, signed ROS1/ROS2 topic probes and platform-mode dry-runs passed | APPROVED | APPROVED | `8936d8a` |
| 4. Orchestrator and REST/WebSocket API | PASS | `19 passed`; full suite `125 passed`; Ruff, compileall, API/Compose smoke and live ROS1/ROS2 command publication passed | APPROVED | APPROVED | `8936d8a` |
| 5. ROS1/ROS2 Web Sidecars and Reverse Proxy | PASS | `7 passed`; full suite `130 passed`; both sidecars/Caddy healthy; live ROS1+ROS2 subscribe and publish-denial smoke passed | APPROVED | APPROVED | `8936d8a` |
| 6. Webots Stream Compatibility | PASS | ROS2 host/container `15 passed` each; platform full suite `134 passed`; real WebSocket, Chromium canvas/interaction and firewall isolation passed | APPROVED after native-bind correction | APPROVED | `65ab31c` |
| 7. Gazebo/RViz/noVNC Displays | PASS | `10 passed`; full suite `144 passed`; live dual-canvas/HTTP/listener checks passed | APPROVED | APPROVED after ownership fixes | `8b202f8` |
| 8. React Workbench | PASS | `7 passed`; typecheck/build/audit and live Caddy Chromium review passed | APPROVED | APPROVED after truthful-data and dependency fixes | `5a54a9b` |
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

## Task 3.5: Integration Contracts

Date: 2026-08-01

- Scenario acceptance: every manifest now declares the loopback ROS1 master and ROS2 domain; swarm uses domain `42`, while RoboSAR Webots and ROS1 scenarios use domain `0`. Every ROS readiness probe declares the exact ROS1/ROS2 message type, maximum silence window and optional ROS2 QoS profile.
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
- Automated acceptance: the Task 4 narrow suite returned `19 passed`; full suite returned `125 passed`; Ruff, compileall and `git diff --check` passed. The host API and rebuilt `dev-api` Compose image served health, three-workspace capabilities and the six-scenario catalog.
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
