# Response Formation Navigation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the demo-specific four-planner response formation with one gated, clearance-aware, continuously deformable formation navigator that works in both the waypoint demo and Tepper task chain.

**Architecture:** One C++ formation navigation node plans and tracks a shared virtual-center path, computes continuous longitudinal/lateral slot deformation, filters local velocities with all TB3 scans and odometry, and publishes navigation status. Existing Python task management and task controllers remain; the final command mux adds a default-closed motion gate and hard safety invariant.

**Tech Stack:** ROS1 Noetic, C++14, Python 3, Eigen3, `nav_msgs/OccupancyGrid`, `sensor_msgs/LaserScan`, `robosar_messages`, pytest, catkin gtest, rostest, Gazebo/RViz.

**Source spec:** `docs/superpowers/specs/2026-07-07-response-formation-navigation-design.md`

**Execution status (updated 2026-07-14):** Tasks 1--12 are complete. A clean Tepper run closed `7 DONE + response HOME + exploration HOME + final STOP`; all automated suites and the full workspace build pass. The full entry now has one bridge-compatible standard-master chain and cleans the previous ROS graph before launch; the redundant isolated-master branch has been removed. No commits were created because the shared workspace already contains extensive uncommitted user changes; see `docs/RESPONSE_FORMATION_REFACTOR_PROGRESS.md` for exact evidence and rejected-run root causes.

Current execution ledger:

| Plan scope | State | Exit condition |
|---|---|---|
| Tasks 1--3: gate, resume/HOME, task geometry | Complete | Full-stack command matrix verified |
| Tasks 4--9: clearance center planner, continuous formation, velocity safety, arrival, legacy deletion | Complete | Demo/Tepper share one runtime chain; no straight-line/map fallback |
| Task 10: final task safety and controlled transport contact | Complete | Four-car physical chains, no negative linear commands, predictive cross-group limiter covered |
| Task 11: acceptance | Complete | Clean run closed both HOME chains and nine-car zero-speed invariant |
| Task 12: documentation/final checks | Complete | Exact final JSON, full tests, build and diff checks recorded |

---

## 1. Implementation boundary

This plan changes the complete response navigation chain in three independently verifiable releases:

1. **Control correctness:** default-closed START/STOP gate, resume, and manual response HOME.
2. **Navigation replacement and cutover:** single center planner, clearance-aware path, continuous affine formation, scan-based local safety, quality-based arrival, and removal of the response-specific legacy runtime in the same verified stage.
3. **Integration hardening:** final mux simplification, demo/Tepper fault injection and long-run regression, and documentation.

Do not mix checkpoint policy changes into these releases. If roundflocking or transport still fails after the navigator hands off at the correct geometry, diagnose that as a separate controller issue.

## 2. File responsibility map

### New C++ navigation core

| File | Responsibility |
|---|---|
| `src/robosar_deform_vendor/plan_manager/include/plan_manager/clearance_grid.h` | Validated OccupancyGrid view, obstacle clearance field, map/world conversion |
| `src/robosar_deform_vendor/plan_manager/src/clearance_grid.cpp` | Multi-source 8-neighbor distance propagation and footprint feasibility |
| `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_planner.h` | Clearance-weighted center A*, no-corner-cutting and line-of-sight smoothing interface |
| `src/robosar_deform_vendor/plan_manager/src/formation_planner.cpp` | Single global center plan and explicit plan failure reasons |
| `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_control.h` | Slot assignment, affine deformation, formation quality and arrival evaluation |
| `src/robosar_deform_vendor/plan_manager/src/formation_control.cpp` | Continuous scales, zero-mean slot offsets and center-path tracking references |
| `src/robosar_deform_vendor/plan_manager/include/plan_manager/velocity_safety.h` | Time-to-collision and deterministic safe velocity projection |
| `src/robosar_deform_vendor/plan_manager/src/velocity_safety.cpp` | Candidate evaluation against teammates and short-lived scan obstacles |
| `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_navigation.h` | ROS-independent navigation state and ROS node class boundary |
| `src/robosar_deform_vendor/plan_manager/src/formation_navigation.cpp` | One group-level ROS I/O/state machine and four command outputs |
| `src/robosar_deform_vendor/plan_manager/src/formation_navigation_node.cpp` | Minimal ROS executable entry point |

### Existing Python orchestration retained and simplified

| File | Responsibility after change |
|---|---|
| `scripts/emergency_task_manager_node.py` | Task queue, exact resume mode, manual response HOME, no automatic HOME |
| `scripts/response_goal_sequence_node.py` | Demo task source; START/STOP/HOME semantics; no duplicate direct goal output |
| `scripts/response_cmd_mux_node.py` | Unique final cmd_vel owner, default-closed motion gate, timeout and hard safety |
| `src/robosar_emergency_response/cmd_mux.py` | Pure command selection and hard-safety helpers |
| `src/robosar_emergency_response/task_manager.py` | Task-specific center target and terminal yaw/profile |
| `src/robosar_emergency_response/home_pose.py` | First complete runtime home snapshot, unchanged except stricter error reporting |

### Files removed after the replacement is verified

```text
src/robosar/robosar_emergency_response/scripts/deform_dispatch_bridge_node.py
src/robosar/robosar_emergency_response/scripts/response_group_coordinator_node.py
src/robosar/robosar_emergency_response/src/robosar_emergency_response/coordinator.py
src/robosar/robosar_emergency_response/src/robosar_emergency_response/dispatch.py
src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml
```

Keep `formation.py` until all Python tests and marker helpers are migrated. Delete it only if `rg "robosar_emergency_response.formation"` has no runtime consumers.

The old `plan_manager_node` executable is still referenced by vendor `one_run.xml`/`multi_run.launch` outside the response launch chain. Keep it as compatibility code in this project unless those vendor demos are migrated and tested in the same change; removing it only from the response runtime avoids breaking unrelated launch files.

## 3. Dependency order

```text
Task 1 motion gate
   |
Task 2 pause/resume/manual HOME
   |
Task 3 task goal profiles
   |
   +-------------------------------+
   |                               |
Task 4 clearance planner       Task 5 formation geometry
   |                               |
   +---------------+---------------+
                   |
             Task 6 velocity safety
                   |
             Task 7 central ROS node
                   |
             Task 8 arrival/handoff
                   |
             Task 9 launch migration/deletion
                   |
             Task 10 task-control hard safety
                   |
             Task 11 integration acceptance
                   |
             Task 12 documentation/final verification
```

Tasks 4 and 5 can be developed in parallel worktrees because they touch separate C++ modules and tests. All other tasks are sequential due to shared message/topic contracts.

---

### Task 1: Add a default-closed response motion gate

**Files:**

- Modify: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/cmd_mux.py`
- Modify: `src/robosar/robosar_emergency_response/scripts/response_cmd_mux_node.py`
- Modify: `src/robosar/robosar_emergency_response/config/response_group.yaml`
- Modify: `src/robosar/robosar_emergency_response/config/tepper_response_group.yaml`
- Test: `src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py`
- Test: `src/robosar/robosar_emergency_response/test/test_response_navigation.py`

- [x] **Step 1: Write the failing pure command-gate tests**

Add tests that prove fresh commands cannot bypass a closed gate and that opening the gate preserves existing mode/timeout behavior:

```python
def test_motion_gate_blocks_fresh_navigation_and_task_commands():
    for mode in (MODE_NAV_FORMATION, MODE_TASK_CONTROL):
        selected = select_command(
            mode=mode,
            motion_enabled=False,
            deform_cmd=Twist2D(0.2, 0.4),
            task_cmd=Twist2D(0.1, -0.3),
            now=10.0,
            last_deform_time=10.0,
            last_task_time=10.0,
            command_timeout=0.6,
        )
        assert selected == ZERO_TWIST


def test_open_motion_gate_selects_fresh_command_for_current_mode():
    assert select_command(
        mode=MODE_NAV_FORMATION,
        motion_enabled=True,
        deform_cmd=Twist2D(0.2, 0.4),
        task_cmd=Twist2D(0.1, -0.3),
        now=10.0,
        last_deform_time=10.0,
        last_task_time=10.0,
        command_timeout=0.6,
    ) == Twist2D(0.2, 0.4)
```

- [x] **Step 2: Run the focused test and confirm the signature failure**

Run:

```bash
python3 -m pytest -q \
  src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py \
  -k motion_gate
```

Expected: FAIL because `select_command()` does not accept `motion_enabled`.

- [x] **Step 3: Implement the pure gate before mode selection**

Change the signature and make the first branch explicit:

```python
def select_command(
    mode: str,
    motion_enabled: bool,
    deform_cmd: Twist2D,
    task_cmd: Twist2D,
    now: float,
    last_deform_time: float | None,
    last_task_time: float | None,
    command_timeout: float,
) -> Twist2D:
    if not motion_enabled:
        return ZERO_TWIST
    if mode in (MODE_STOP, MODE_FAILED):
        return ZERO_TWIST
    if mode == MODE_NAV_FORMATION:
        return deform_cmd if _fresh(last_deform_time, now, command_timeout) else ZERO_TWIST
    if mode == MODE_TASK_CONTROL:
        return task_cmd if _fresh(last_task_time, now, command_timeout) else ZERO_TWIST
    return ZERO_TWIST
```

- [x] **Step 4: Consume one latched motion-permission topic**

In `ResponseCmdMuxNode`:

```python
self.mode = MODE_STOP
self.motion_enabled = False
self.motion_enabled_sub = rospy.Subscriber(
    "/response_group/motion_enabled", Bool, self._motion_enabled_callback, queue_size=1
)
```

The callback has no task semantics:

```python
def _motion_enabled_callback(self, msg):
    self.motion_enabled = bool(msg.data)
```

The active task source publishes this topic with `latch=True`. This avoids missed one-shot GUI commands when nodes start in a different order and prevents the task manager, navigator and mux from implementing three subtly different command state machines.

- [x] **Step 5: Remove unsafe configuration defaults**

Set `default_mode: STOP` in both response configs. Add a config test:

```python
assert response_demo_config["default_mode"] == "STOP"
assert tepper_config["default_mode"] == "STOP"
```

- [x] **Step 6: Run unit tests**

Run:

```bash
python3 -m pytest -q src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py
```

Expected: PASS.

- [x] **Step 7: Preserve the gate changes without committing the dirty shared tree**

```bash
git add \
  src/robosar/robosar_emergency_response/src/robosar_emergency_response/cmd_mux.py \
  src/robosar/robosar_emergency_response/scripts/response_cmd_mux_node.py \
  src/robosar/robosar_emergency_response/config/response_group.yaml \
  src/robosar/robosar_emergency_response/config/tepper_response_group.yaml \
  src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py \
  src/robosar/robosar_emergency_response/test/test_response_navigation.py
git commit -m "fix: gate response motion until mission start"
```

Before committing, inspect `git diff --cached`; these files already have local edits in the current workspace, so do not accidentally include unrelated hunks.

---

### Task 2: Make pause/resume and manual response HOME state-correct

**Files:**

- Modify: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/task_manager.py`
- Modify: `src/robosar/robosar_emergency_response/scripts/emergency_task_manager_node.py`
- Modify: `src/robosar/robosar_emergency_response/scripts/response_goal_sequence_node.py`
- Modify: `src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh`
- Modify: `src/robosar/robosar_emergency_response/config/tepper_response_group.yaml`
- Test: `src/robosar/robosar_emergency_response/test/test_response_navigation.py`
- Test: `src/robosar/robosar_emergency_response/test/test_response_task_logic.py`

- [x] **Step 1: Add failing tests for resume-mode selection**

Introduce a pure helper and tests:

```python
def test_resume_mode_matches_preserved_task_phase():
    assert resume_mode_for_status(STATUS_EN_ROUTE, return_home_active=False) == "NAV_FORMATION"
    assert resume_mode_for_status(STATUS_ACTING, return_home_active=False) == "TASK_CONTROL"
    assert resume_mode_for_status(STATUS_EN_ROUTE, return_home_active=True) == "NAV_FORMATION"
    assert resume_mode_for_status(STATUS_DONE, return_home_active=False) == "STOP"
```

Add a queue cancellation test for manual HOME:

```python
def test_manual_home_terminates_active_task_without_dispatching_next():
    queue = _queue()
    queue.upsert_object(_spec(TASK_BOX, 1, 8.0, 0.0), MatrixPose())
    active = queue.dispatch_next()
    cancelled = queue.cancel_active_for_return_home()
    assert cancelled.task_id == active.task_id
    assert cancelled.status == STATUS_FAILED
    assert queue.current_task() is None
```

- [x] **Step 2: Run the tests and confirm missing helpers**

```bash
python3 -m pytest -q \
  src/robosar/robosar_emergency_response/test/test_response_navigation.py \
  src/robosar/robosar_emergency_response/test/test_response_task_logic.py \
  -k "resume_mode or manual_home"
```

Expected: FAIL because both helpers are absent.

- [x] **Step 3: Add explicit resume and cancellation helpers**

In `task_manager.py`:

```python
def resume_mode_for_status(status: int, return_home_active: bool) -> str:
    if return_home_active or status == STATUS_EN_ROUTE:
        return "NAV_FORMATION"
    if status == STATUS_ACTING:
        return "TASK_CONTROL"
    return "STOP"


def cancel_active_for_return_home(self) -> ManagedTask | None:
    current = self.current_task()
    if current is None:
        return None
    cancelled = replace(current, status=STATUS_FAILED)
    self._tasks[cancelled.key] = cancelled
    self._active_task_id = None
    return cancelled
```

- [x] **Step 4: Republish the preserved phase on START**

`_start_dispatch()` must inspect the active task before dispatching a new one:

```python
current = self.queue.current_task()
if self.return_home_active:
    self.motion_enabled_pub.publish(Bool(True))
    self.mode_pub.publish(String("NAV_FORMATION"))
    self.response_task_pub.publish(self._home_task_msg(EmergencyTask.EN_ROUTE))
elif current is not None:
    self.motion_enabled_pub.publish(Bool(True))
    mode = resume_mode_for_status(current.status, False)
    self.mode_pub.publish(String(mode))
    self.response_task_pub.publish(self._to_msg(current))
else:
    self.motion_enabled_pub.publish(Bool(True))
    self._dispatch_if_idle()
```

Create `motion_enabled_pub` as a latched `std_msgs/Bool` publisher. Publish false during construction before accepting tasks. STOP publishes false without clearing current task. The demo goal-sequence node owns the same topic in the standalone demo; only one of these task-source nodes is launched in an environment.

- [x] **Step 5: Add the Tepper response HOME subscriber**

Subscribe to `/response_group/return_home`. On true:

1. set `started=True` only for the response return flow;
2. cancel the current response task as FAILED with detail logged;
3. set a terminal `manual_return_requested` flag so no queued task dispatches after home;
4. refresh the complete runtime home snapshot;
5. publish the reserved home task and `NAV_FORMATION`.

The callback must ignore duplicate true messages while already returning or after home completion.

If the complete runtime home snapshot is unavailable, reject the HOME request, keep motion disabled, and publish/log `HOME_POSE_UNAVAILABLE`; do not silently drive toward configured fallback coordinates.

- [x] **Step 6: Disable automatic response HOME**

Change:

```yaml
auto_return_home: false
```

When all tasks are DONE, publish STOP and the state `COMPLETE_WAITING_FOR_HOME`; do not call `_maybe_dispatch_return_home()` unless the manual flag is set.

- [x] **Step 7: Align the demo sequencer with system START/STOP**

Make `response_goal_sequence_node.py` also subscribe to `/system_mission_command`. START calls its existing delayed start/resume path; STOP publishes `STOP` without clearing sequence index; exploration HOME is ignored; `/response_group/return_home` keeps its existing response HOME behavior.

In `emergency_task_manager_node.py`, remove the current `HOME -> _stop_dispatch()` branch. Exploration HOME must not change response mode or response motion permission.

- [x] **Step 8: Start the response task source before exposing the GUI**

In `run_tepper_response_demo.sh`, launch `response_overlay` before `gui`. Wait for `/response_group/motion_enabled` once and require its initial value to be false before starting the GUI process. This closes the only interval in which a user could click START before the authoritative response task source subscribed to `/system_mission_command`.

- [x] **Step 9: Run task-state tests**

```bash
python3 -m pytest -q \
  src/robosar/robosar_emergency_response/test/test_response_navigation.py \
  src/robosar/robosar_emergency_response/test/test_response_task_logic.py
```

Expected: PASS.

- [x] **Step 10: Preserve the state-machine change without committing the dirty shared tree**

```bash
git add \
  src/robosar/robosar_emergency_response/src/robosar_emergency_response/task_manager.py \
  src/robosar/robosar_emergency_response/scripts/emergency_task_manager_node.py \
  src/robosar/robosar_emergency_response/scripts/response_goal_sequence_node.py \
  src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh \
  src/robosar/robosar_emergency_response/config/tepper_response_group.yaml \
  src/robosar/robosar_emergency_response/test/test_response_navigation.py \
  src/robosar/robosar_emergency_response/test/test_response_task_logic.py
git commit -m "fix: resume response phases and wire manual home"
```

---

### Task 3: Encode task-specific center targets and terminal orientation

**Files:**

- Modify: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/task_manager.py`
- Modify: `src/robosar/robosar_emergency_response/config/tepper_response_group.yaml`
- Test: `src/robosar/robosar_emergency_response/test/test_response_task_logic.py`

- [x] **Step 1: Write failing SURROUND and TRANSPORT target tests**

```python
def test_surround_navigation_centers_formation_on_object():
    target = compute_navigation_target(
        TASK_FIRE,
        object_pose=MatrixPose(5.0, 4.0, 0.2),
        group_center=MatrixPose(1.0, 4.0),
        goal_pose=None,
        transport_rear_distance=1.0,
    )
    assert target.profile == "SURROUND"
    assert (target.pose.x, target.pose.y) == (5.0, 4.0)


def test_transport_navigation_centers_formation_behind_box_and_faces_push_direction():
    target = compute_navigation_target(
        TASK_BOX,
        object_pose=MatrixPose(5.0, 4.0),
        group_center=MatrixPose(),
        goal_pose=MatrixPose(9.0, 4.0),
        transport_rear_distance=1.0,
    )
    assert target.profile == "TRANSPORT_APPROACH"
    assert math.isclose(target.pose.x, 4.0)
    assert math.isclose(target.pose.y, 4.0)
    assert math.isclose(target.pose.yaw, 0.0)
```

- [x] **Step 2: Run and confirm failure**

```bash
python3 -m pytest -q \
  src/robosar/robosar_emergency_response/test/test_response_task_logic.py \
  -k "navigation_centers or transport_navigation"
```

Expected: FAIL because `compute_navigation_target` is absent.

- [x] **Step 3: Add a typed navigation target**

```python
@dataclass(frozen=True)
class NavigationTarget:
    pose: MatrixPose
    profile: str


def compute_navigation_target(
    task_type: int,
    object_pose: MatrixPose,
    group_center: MatrixPose,
    goal_pose: MatrixPose | None,
    transport_rear_distance: float,
) -> NavigationTarget:
    if task_type != TASK_BOX:
        _, _, yaw = _safe_direction(
            object_pose.x - group_center.x,
            object_pose.y - group_center.y,
            object_pose.yaw,
        )
        return NavigationTarget(MatrixPose(object_pose.x, object_pose.y, yaw), "SURROUND")
    if goal_pose is None:
        raise ValueError("transport task requires goal_pose")
    dx, dy, yaw = _safe_direction(goal_pose.x - object_pose.x, goal_pose.y - object_pose.y)
    return NavigationTarget(
        MatrixPose(
            object_pose.x - dx * transport_rear_distance,
            object_pose.y - dy * transport_rear_distance,
            yaw,
        ),
        "TRANSPORT_APPROACH",
    )
```

Store the profile in `behavior_type`-derived navigation state or add it to the existing `scene_name/detail` adapter without changing `EmergencyTask.msg`; the central navigator can infer `SURROUND` from non-BOX and `TRANSPORT_APPROACH` from BOX.

- [x] **Step 4: Remove unused victim/fire staging distances**

Delete `staging_distance_victim` and `staging_distance_fire` from config and constructor wiring. Rename `staging_distance_box` to `transport_rear_distance` consistently in code and tests.

- [x] **Step 5: Run all response task tests**

```bash
python3 -m pytest -q src/robosar/robosar_emergency_response/test/test_response_task_logic.py
```

Expected: PASS.

- [x] **Step 6: Preserve the task-geometry change without committing the dirty shared tree**

```bash
git add \
  src/robosar/robosar_emergency_response/src/robosar_emergency_response/task_manager.py \
  src/robosar/robosar_emergency_response/config/tepper_response_group.yaml \
  src/robosar/robosar_emergency_response/test/test_response_task_logic.py
git commit -m "feat: derive formation goals from task geometry"
```

---

### Task 4: Build the clearance-aware center planner as a pure C++ library

**Files:**

- Create: `src/robosar_deform_vendor/plan_manager/include/plan_manager/clearance_grid.h`
- Create: `src/robosar_deform_vendor/plan_manager/src/clearance_grid.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_planner.h`
- Create: `src/robosar_deform_vendor/plan_manager/src/formation_planner.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/test/test_clearance_grid.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/test/test_formation_planner.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/CMakeLists.txt`

- [ ] **Step 1: Define a small explicit planner API**

Use this public contract:

```cpp
namespace plan_manager {

enum class PlanFailure {
  NONE,
  MAP_INVALID,
  START_OUTSIDE_MAP,
  GOAL_OUTSIDE_MAP,
  START_IN_COLLISION,
  GOAL_IN_COLLISION,
  NO_STATIC_PATH,
};

struct PlannerConfig {
  double minimum_envelope_radius;
  double clearance_cost_weight;
  bool treat_unknown_as_occupied;
};

struct CenterPlan {
  std::vector<Eigen::Vector2d> points;
  PlanFailure failure = PlanFailure::NONE;
  double minimum_clearance = 0.0;
};

class FormationPlanner {
 public:
  explicit FormationPlanner(PlannerConfig config);
  CenterPlan plan(
      const nav_msgs::OccupancyGrid& map,
      const Eigen::Vector2d& start,
      const Eigen::Vector2d& goal) const;
};

}  // namespace plan_manager
```

- [ ] **Step 2: Write failing clearance and validity tests**

Tests must cover:

```cpp
TEST(ClearanceGrid, RejectsMalformedDataLength) { /* width*height != data.size() */ }
TEST(ClearanceGrid, ComputesSymmetricDistanceFromSingleObstacle) { /* compare ±x/±y */ }
TEST(FormationPlanner, RejectsMissingMapInsteadOfReturningDirectLine) { /* MAP_INVALID */ }
TEST(FormationPlanner, RejectsOccupiedGoalWithoutMovingIt) { /* GOAL_IN_COLLISION */ }
```

- [ ] **Step 3: Write failing path-quality tests**

Construct maps in memory and assert:

```cpp
TEST(FormationPlanner, DoesNotCutOccupiedDiagonalCorner) { /* no diagonal through two walls */ }
TEST(FormationPlanner, ChoosesCenterOfWideCorridor) { /* mean y near corridor center */ }
TEST(FormationPlanner, RejectsCorridorNarrowerThanMinimumEnvelope) { /* NO_STATIC_PATH */ }
TEST(FormationPlanner, SmoothingPreservesEnvelopeClearance) { /* each sampled segment is valid */ }
```

- [ ] **Step 4: Register tests and confirm red state**

Set `CMAKE_CXX_STANDARD 14`, `CMAKE_CXX_STANDARD_REQUIRED ON`, and use `catkin_add_gtest` inside `if(CATKIN_ENABLE_TESTING)`. Run:

```bash
catkin_make --pkg plan_manager -DCATKIN_ENABLE_TESTING=ON
```

Expected: build/test failure until the classes are implemented.

- [ ] **Step 5: Implement the clearance grid**

Implementation rules:

1. Validate dimensions, resolution, origin and data length.
2. Seed all occupied/unknown-as-occupied cells into a priority queue.
3. Propagate 8-neighbor distances using 1 and `sqrt(2)` costs.
4. Convert cell distance to meters.
5. Expose `clearance(mx,my)` and `isFeasible(mx,my,envelope_radius)`.

This is one O(N log N) map update. Cache it in the ROS node later; do not recompute per robot or control tick.

- [ ] **Step 6: Implement weighted A***

For each neighbor:

```cpp
const double safe_gap = std::max(
    clearance - config_.minimum_envelope_radius,
    static_cast<double>(map.info.resolution));
const double wall_penalty = config_.clearance_cost_weight / (safe_gap * safe_gap);
const double tentative = current_cost + step_length * (1.0 + wall_penalty);
```

Reject diagonal neighbors if either orthogonal side cell is infeasible. Return an explicit failure; never project start/goal to another cell.

- [ ] **Step 7: Implement safe line-of-sight smoothing**

Greedily skip intermediate points only when sampling the full segment at half-cell increments shows clearance at least `minimum_envelope_radius`. Preserve exact world start and goal.

- [ ] **Step 8: Build and run C++ tests**

```bash
catkin_make --pkg plan_manager -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_plan_manager
catkin_test_results build/test_results/plan_manager
```

Expected: all planner tests PASS and zero failures.

- [ ] **Step 9: Commit**

```bash
git add \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/clearance_grid.h \
  src/robosar_deform_vendor/plan_manager/src/clearance_grid.cpp \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_planner.h \
  src/robosar_deform_vendor/plan_manager/src/formation_planner.cpp \
  src/robosar_deform_vendor/plan_manager/test/test_clearance_grid.cpp \
  src/robosar_deform_vendor/plan_manager/test/test_formation_planner.cpp \
  src/robosar_deform_vendor/plan_manager/CMakeLists.txt
git commit -m "feat: add clearance-aware formation center planner"
```

Review the staged diff so legacy `plan_manager.cpp` edits are not included in this commit.

---

### Task 5: Implement continuous affine formation geometry and quality

**Files:**

- Create: `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_control.h`
- Create: `src/robosar_deform_vendor/plan_manager/src/formation_control.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/test/test_formation_control.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/CMakeLists.txt`
- Modify: `src/robosar_deform_vendor/plan_manager/src/formation_assignment.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_assignment.h`

- [ ] **Step 1: Define geometry types**

```cpp
struct FormationLimits {
  double robot_radius;
  double nominal_side;
  double inter_robot_safety_gap;
  double max_scale_rate_per_second;
};

struct DeformationState {
  double longitudinal_scale = 1.0;
  double lateral_scale = 1.0;
  std::vector<Eigen::Vector2d> local_offsets;
};

struct FormationQuality {
  double rms_slot_error;
  double max_slot_error;
  double minimum_pair_distance;
  double center_error;
  bool collision_safe;
};

std::vector<Eigen::Vector2d> affineSlotTargets(
    const Eigen::Vector2d& center,
    double path_yaw,
    const std::vector<Eigen::Vector2d>& nominal_slots,
    const DeformationState& deformation);
```

- [ ] **Step 2: Write failing geometry tests**

```cpp
TEST(FormationControl, MinimumScaleComesFromRobotDiameterAndGap) { /* no magic 0.45 */ }
TEST(FormationControl, AffineTargetsKeepCentroidAtCenter) { /* mean(targets)==center */ }
TEST(FormationControl, ZeroMeanLocalOffsetsDoNotMoveCenter) { /* sum(delta)==0 */ }
TEST(FormationControl, LateralScaleCanShrinkWithoutLongitudinalScale) { /* rectangle */ }
TEST(FormationControl, ScaleChangeIsRateLimited) { /* dt bounds change */ }
TEST(FormationControl, QualityAllowsVisibleImperfectSquare) { /* RMS pass, no exact corners */ }
```

- [ ] **Step 3: Confirm tests fail, then implement minimal geometry**

Run:

```bash
catkin_make --pkg plan_manager -DCATKIN_ENABLE_TESTING=ON
```

Expected before implementation: compile failure for missing symbols.

Derive the minimum adjacent center distance as:

```text
minimum_side = 2 * robot_radius + inter_robot_safety_gap
minimum_scale = min(1, minimum_side / nominal_side)
```

For every update, compute clearance-derived target scales, clamp to the physical minimum, and apply `max_scale_rate_per_second * dt`. Normalize local offsets by subtracting their mean before slot generation.

- [ ] **Step 4: Remove cohesion logic from assignment**

Delete `cohesionLinearScale()` and its tests. Keep only deterministic slot assignment and slot rotation in `formation_assignment.*`.

- [ ] **Step 5: Run C++ tests**

```bash
catkin_make run_tests_plan_manager
catkin_test_results build/test_results/plan_manager
```

Expected: all formation geometry and existing assignment tests PASS.

- [ ] **Step 6: Commit**

```bash
git add \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_control.h \
  src/robosar_deform_vendor/plan_manager/src/formation_control.cpp \
  src/robosar_deform_vendor/plan_manager/test/test_formation_control.cpp \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_assignment.h \
  src/robosar_deform_vendor/plan_manager/src/formation_assignment.cpp \
  src/robosar_deform_vendor/plan_manager/CMakeLists.txt
git commit -m "feat: add continuous affine formation geometry"
```

---

### Task 6: Add deterministic local velocity safety projection

**Files:**

- Create: `src/robosar_deform_vendor/plan_manager/include/plan_manager/velocity_safety.h`
- Create: `src/robosar_deform_vendor/plan_manager/src/velocity_safety.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/test/test_velocity_safety.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/CMakeLists.txt`
- Reference: `src/robosar/robosar_controller/include/lazy_traffic_rvo.hpp`
- Reference: `src/robosar/robosar_controller/test/time_to_collision_test.cpp`

- [ ] **Step 1: Define deterministic inputs and outputs**

```cpp
struct DiscObstacle {
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  double radius;
  ros::Time stamp;
};

struct VelocitySafetyConfig {
  double robot_radius;
  double safety_gap;
  double time_horizon;
  double max_speed;
  double obstacle_timeout;
};

struct SafeVelocityResult {
  Eigen::Vector2d velocity;
  double minimum_time_to_collision;
  bool blocked;
};

SafeVelocityResult projectSafeVelocity(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& current_velocity,
    const Eigen::Vector2d& preferred_velocity,
    const std::vector<DiscObstacle>& obstacles,
    const VelocitySafetyConfig& config,
    const ros::Time& now);
```

- [ ] **Step 2: Port and improve the existing time-to-collision tests**

Cover moving apart, head-on, tangent, already overlapping, stale obstacle and zero preferred velocity. Add determinism:

```cpp
TEST(VelocitySafety, RepeatedCallsReturnIdenticalVelocity) {
  const auto first = projectSafeVelocity(/* fixed inputs */);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(projectSafeVelocity(/* same inputs */).velocity, first.velocity);
  }
}
```

- [ ] **Step 3: Confirm red state**

```bash
catkin_make --pkg plan_manager -DCATKIN_ENABLE_TESTING=ON
```

Expected: compile failure for the missing implementation.

- [ ] **Step 4: Implement bounded deterministic candidate selection**

Candidate set:

1. preferred velocity;
2. preferred direction at 75%, 50%, 25% speed;
3. heading rotations at ±15°, ±30°, ±45°, each at 75% and 50%;
4. zero velocity.

Reject candidates with predicted disc collision inside `time_horizon` or with a static scan endpoint inside the robot radius along the rollout. Score accepted candidates by:

```text
score = |candidate - preferred|^2
      + 0.2 * |candidate - current|^2
      - 0.1 * forward_progress
```

Constants are internal algorithm coefficients covered by tests, not environment-specific ROS parameters. Zero velocity is always feasible and guarantees a safe result when the local corridor is temporarily blocked.

- [ ] **Step 5: Run the tests**

```bash
catkin_make run_tests_plan_manager
catkin_test_results build/test_results/plan_manager
```

Expected: PASS, including repeated-call determinism.

- [ ] **Step 6: Commit**

```bash
git add \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/velocity_safety.h \
  src/robosar_deform_vendor/plan_manager/src/velocity_safety.cpp \
  src/robosar_deform_vendor/plan_manager/test/test_velocity_safety.cpp \
  src/robosar_deform_vendor/plan_manager/CMakeLists.txt
git commit -m "feat: add deterministic local collision projection"
```

---

### Task 7: Introduce the single group-level formation navigation node

**Files:**

- Create: `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_navigation.h`
- Create: `src/robosar_deform_vendor/plan_manager/src/formation_navigation.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/src/formation_navigation_node.cpp`
- Create: `src/robosar_deform_vendor/plan_manager/test/test_navigation_state.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/CMakeLists.txt`
- Modify: `src/robosar_deform_vendor/plan_manager/package.xml`
- Modify: `src/robosar/robosar_messages/msg/ResponseGroupStatus.msg`

- [ ] **Step 1: Extend status with observable navigation metrics**

Append fields without renumbering existing constants:

```text
string planning_state
float32 longitudinal_scale
float32 lateral_scale
float32 rms_slot_error
float32 max_slot_error
float32 minimum_clearance
bool motion_enabled
geometry_msgs/PoseStamped center_reference
geometry_msgs/PoseStamped[] slot_targets
```

Rebuild `robosar_messages` before consumers.

- [ ] **Step 2: Write failing navigation-state tests**

The pure state reducer must cover:

```cpp
TEST(NavigationState, StartsStoppedWithoutMapOrMotionPermission) { /* STOPPED */ }
TEST(NavigationState, MissingMapDeadlineProducesMapUnavailable) { /* ERROR */ }
TEST(NavigationState, StaticNoPathDoesNotProduceDirectFallback) { /* FAILED */ }
TEST(NavigationState, DynamicBlockRetriesOnlyAfterBackoffOrDataChange) { /* bounded A* */ }
TEST(NavigationState, StopPreservesGoalAndPathProgress) { /* PAUSED */ }
```

- [ ] **Step 3: Define one set of ROS inputs and outputs**

Inputs:

```text
/response_group/active_task       robosar_messages/EmergencyTask
/response_group/behavior_mode     std_msgs/String
/response_group/motion_enabled    std_msgs/Bool (latched)
/sim_map or /map                  nav_msgs/OccupancyGrid
/tb_i/odom                        nav_msgs/Odometry
/tb_i/scan                        sensor_msgs/LaserScan
```

Outputs:

```text
/tb_i/deform_cmd_vel              geometry_msgs/Twist
/response_group/status            robosar_messages/ResponseGroupStatus
/response_group/center_path       nav_msgs/Path
/response_group/formation_slots   visualization_msgs/MarkerArray
/response_group/state_text        visualization_msgs/Marker
```

- [ ] **Step 4: Implement startup readiness and map failure**

The node starts with zero commands. It accepts a map only when dimensions, resolution, frame and data length are valid. If `map_startup_timeout` elapses after a navigation task becomes runnable, publish `MAP_UNAVAILABLE`, publish zeros, and enter ERROR. Do not synthesize any path.

- [ ] **Step 5: Implement one center plan and shared path progress**

On a new task or material map change:

1. compute actual center from all fresh odom;
2. plan once with `FormationPlanner`;
3. store one path and one monotonically advancing projected arc length;
4. compute one lookahead center reference and path tangent;
5. publish the path once per change.

- [ ] **Step 6: Implement shared deformation and per-robot commands**

At control rate:

1. derive current longitudinal/lateral clearance from the cached field and fresh scans;
2. update the shared `DeformationState` with rate limits;
3. generate four slot targets;
4. form each preferred map-frame velocity from slot error plus center feed-forward;
5. project it through `projectSafeVelocity` using teammates and local obstacle snapshots;
6. convert map velocity to TB3 `(linear.x, angular.z)` with the existing alignment slowdown and acceleration limits;
7. publish four commands from this one callback.

- [ ] **Step 7: Build and run tests**

```bash
catkin_make --pkg robosar_messages plan_manager -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_plan_manager
catkin_test_results build/test_results/plan_manager
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add \
  src/robosar/robosar_messages/msg/ResponseGroupStatus.msg \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_navigation.h \
  src/robosar_deform_vendor/plan_manager/src/formation_navigation.cpp \
  src/robosar_deform_vendor/plan_manager/src/formation_navigation_node.cpp \
  src/robosar_deform_vendor/plan_manager/test/test_navigation_state.cpp \
  src/robosar_deform_vendor/plan_manager/CMakeLists.txt \
  src/robosar_deform_vendor/plan_manager/package.xml
git commit -m "feat: add single response formation navigator"
```

---

### Task 8: Replace rigid arrival barriers with formation-quality handoff

**Files:**

- Modify: `src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_control.h`
- Modify: `src/robosar_deform_vendor/plan_manager/src/formation_control.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/src/formation_navigation.cpp`
- Modify: `src/robosar_deform_vendor/plan_manager/test/test_formation_control.cpp`
- Modify: `src/robosar/robosar_emergency_response/config/response_group.yaml`
- Modify: `src/robosar/robosar_emergency_response/config/tepper_response_group.yaml`

- [ ] **Step 1: Write profile-specific arrival tests**

```cpp
TEST(FormationArrival, WaypointAllowsSmallNonRigidOffsets) { /* center + RMS + safety */ }
TEST(FormationArrival, SurroundRequiresTargetInsideConvexHull) { /* reject one-sided cluster */ }
TEST(FormationArrival, TransportRequiresTwoRowsBehindBox) { /* yaw and projected errors */ }
TEST(FormationArrival, DoesNotRequireNominalScaleNearGoal) { /* scale 0.72 can arrive */ }
TEST(FormationArrival, HoldTimeRejectsTransientPassThrough) { /* reset timer when quality drops */ }
```

- [ ] **Step 2: Implement one arrival evaluator**

Use `FormationQuality` plus profile geometry:

```cpp
struct ArrivalConfig {
  double center_tolerance;
  double rms_slot_tolerance;
  double max_slot_tolerance;
  double hold_seconds;
};

struct ArrivalEvidence {
  bool arrived;
  bool center_ok;
  bool formation_ok;
  bool safety_ok;
  bool profile_geometry_ok;
};
```

The navigator publishes DONE only when every boolean remains true for `hold_seconds`. It no longer consumes `/tb_i/deform_reached`.

- [ ] **Step 3: Simplify arrival configuration**

Replace `arrival_radius`, goal-region radii/fractions and transport goal-region radii with profile groups:

```yaml
arrival:
  waypoint: {center_tolerance: 0.30, rms_slot_tolerance: 0.35, max_slot_tolerance: 0.55, hold_seconds: 0.8}
  surround: {center_tolerance: 0.25, rms_slot_tolerance: 0.40, max_slot_tolerance: 0.60, hold_seconds: 0.8}
  transport: {center_tolerance: 0.40, rms_slot_tolerance: 0.45, max_slot_tolerance: 0.70, hold_seconds: 0.8}
  home: {center_tolerance: 0.30, rms_slot_tolerance: 0.40, max_slot_tolerance: 0.60, hold_seconds: 1.0}
```

These are task semantics, not environment-specific path tuning. Validate every value as positive at startup.

- [ ] **Step 4: Run tests**

```bash
catkin_make run_tests_plan_manager
catkin_test_results build/test_results/plan_manager
```

Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_control.h \
  src/robosar_deform_vendor/plan_manager/src/formation_control.cpp \
  src/robosar_deform_vendor/plan_manager/src/formation_navigation.cpp \
  src/robosar_deform_vendor/plan_manager/test/test_formation_control.cpp \
  src/robosar/robosar_emergency_response/config/response_group.yaml \
  src/robosar/robosar_emergency_response/config/tepper_response_group.yaml
git commit -m "feat: hand off tasks using formation quality"
```

---

### Task 9: Migrate launch files to one navigator and delete legacy paths

**Files:**

- Modify: `src/robosar/robosar_emergency_response/launch/response_group_deform.launch`
- Modify: `src/robosar/robosar_emergency_response/launch/response_deform_demo.launch`
- Modify: `src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch`
- Modify: `src/robosar/robosar_emergency_response/CMakeLists.txt`
- Modify: `src/robosar/robosar_emergency_response/package.xml`
- Modify: `src/robosar_deform_vendor/plan_manager/CMakeLists.txt`
- Delete: response-specific legacy files listed in section 2
- Test: `src/robosar/robosar_emergency_response/test/test_response_navigation.py`
- Test: `src/robosar/robosar_emergency_response/test/test_formation_geometry.py`

- [ ] **Step 1: Write launch-structure tests before changing XML**

```python
def test_response_launch_has_exactly_one_formation_navigator():
    root = ET.parse(_path("launch/response_group_deform.launch")).getroot()
    nodes = [n for n in root.findall("node") if n.attrib.get("type") == "formation_navigation_node"]
    assert len(nodes) == 1


def test_response_launch_has_no_bridge_or_python_arrival_coordinator():
    launch_text = Path(_path("launch/tepper_response_demo.launch")).read_text()
    assert "deform_dispatch_bridge_node.py" not in launch_text
    assert "response_group_coordinator_node.py" not in launch_text
    assert "response_plan_manager_one_run.xml" not in launch_text
```

- [ ] **Step 2: Run and confirm launch tests fail**

```bash
python3 -m pytest -q \
  src/robosar/robosar_emergency_response/test/test_response_navigation.py \
  -k "exactly_one_formation_navigator or no_bridge"
```

Expected: FAIL against the old launch structure.

- [ ] **Step 3: Start one navigator in both environments**

`response_group_deform.launch` must contain one `formation_navigation_node` and pass the map topic, robot names, geometry, speed, sensor freshness and replan parameters. Remove four repeated includes.

Both top-level launches pass their map topic only:

```xml
<arg name="map_topic" default="/map"/>       <!-- demo -->
<arg name="map_topic" default="/sim_map"/>   <!-- Tepper -->
```

Algorithm defaults remain in one shared response config; do not fork the behavior by environment.

- [ ] **Step 4: Simplify demo goal publication**

Remove `/response_group/formation_goal` and `/move_base_simple/goal` from the response chain. `response_goal_sequence_node.py` publishes one `EmergencyTask`; the navigator reads its `staging_pose` directly.

- [ ] **Step 5: Delete legacy runtime and cohesion/scaling parameters**

Delete the files listed in section 2 and remove their install/build entries. Remove these names everywhere:

```text
formation_scale_step
min_formation_scale
formation_expand_distance
blocked_recovery_linear_scale
allow_blocked_direct_recovery
blocked_direct_recovery_max_distance
formation_cohesion_error_threshold
formation_cohesion_error_slack
formation_cohesion_wait_linear_scale
```

Verify:

```bash
rg -n "formation_scale_step|min_formation_scale|blocked_recovery|formation_cohesion|direct fixed-square center path" \
  src/robosar/robosar_emergency_response \
  src/robosar_deform_vendor/plan_manager/src/formation_navigation.cpp \
  src/robosar_deform_vendor/plan_manager/include/plan_manager/formation_navigation.h \
  src/robosar_deform_vendor/plan_manager/src/formation_planner.cpp \
  src/robosar_deform_vendor/plan_manager/src/formation_control.cpp
```

Expected: no runtime matches; historical docs may still mention the removed implementation.

The compatibility `plan_manager_node` may still contain its old private parameters until its separate vendor-demo migration. The zero-match requirement applies to `robosar_emergency_response` and the new `formation_navigation*`, `formation_planner*`, and `formation_control*` files; do not break vendor launches merely to make a repository-wide grep empty.

- [ ] **Step 6: Build and run Python tests**

```bash
catkin_make --pkg robosar_messages plan_manager robosar_emergency_response
python3 -m pytest -q src/robosar/robosar_emergency_response/test
```

Expected: build succeeds and all Python tests PASS.

- [ ] **Step 7: Commit the cutover**

```bash
git add \
  src/robosar/robosar_emergency_response/launch/response_group_deform.launch \
  src/robosar/robosar_emergency_response/launch/response_deform_demo.launch \
  src/robosar/robosar_emergency_response/launch/tepper_response_demo.launch \
  src/robosar/robosar_emergency_response/CMakeLists.txt \
  src/robosar/robosar_emergency_response/package.xml \
  src/robosar_deform_vendor/plan_manager/CMakeLists.txt \
  src/robosar/robosar_emergency_response/test/test_response_navigation.py \
  src/robosar/robosar_emergency_response/test/test_formation_geometry.py
git add -u -- \
  src/robosar/robosar_emergency_response/scripts/deform_dispatch_bridge_node.py \
  src/robosar/robosar_emergency_response/scripts/response_group_coordinator_node.py \
  src/robosar/robosar_emergency_response/src/robosar_emergency_response/coordinator.py \
  src/robosar/robosar_emergency_response/src/robosar_emergency_response/dispatch.py \
  src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml
git commit -m "refactor: cut response navigation over to one center planner"
```

Use `git diff --cached --stat` and `git diff --cached` before committing because both directories contain pre-existing work.

---

### Task 10: Reduce cmd mux to arbitration plus hard safety

**Files:**

- Modify: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/cmd_mux.py`
- Modify: `src/robosar/robosar_emergency_response/scripts/response_cmd_mux_node.py`
- Modify: `src/robosar/robosar_emergency_response/config/response_safety.yaml`
- Test: `src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py`

- [ ] **Step 1: Write hard-safety tests**

Required cases:

```python
def test_hard_safety_stops_forward_motion_inside_clearance(): ...
def test_hard_safety_keeps_zero_command_zero(): ...
def test_box_contact_exclusion_ignores_only_expected_box_return(): ...
def test_box_contact_exclusion_does_not_ignore_teammate_or_wall(): ...
def test_stale_scan_stops_motion_instead_of_bypassing_safety(): ...
```

- [ ] **Step 2: Replace ordinary potential-field steering with a hard invariant**

The central navigator already handles normal scan avoidance. The mux now:

1. selects deform/task command;
2. rejects stale source commands;
3. rejects motion when the run gate is closed;
4. clamps velocity/acceleration;
5. sets forward velocity to zero when a non-excluded obstacle violates hard clearance;
6. always enforces teammate center-distance safety.

Do not generate a new path-following turn in the mux.

- [ ] **Step 3: Narrow BOX contact permission**

Replace `transport_task_bypass_safety` with a geometric scan exclusion centered on the known box bearing and expected contact distance. The exclusion applies only to rays consistent with the active box pose; all other scan rays and teammate checks remain active.

- [ ] **Step 4: Remove obsolete safety tuning**

Delete potential-field blend, min-forward-scale, recovery reverse-speed and formation-reduce parameters. Keep:

```yaml
hard_stop_distance: 0.20
teammate_min_center_distance: 0.22
scan_timeout: 0.30
command_timeout: 0.60
max_translational_velocity: 0.22
max_rotational_velocity: 1.20
```

- [ ] **Step 5: Run tests**

```bash
python3 -m pytest -q src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add \
  src/robosar/robosar_emergency_response/src/robosar_emergency_response/cmd_mux.py \
  src/robosar/robosar_emergency_response/scripts/response_cmd_mux_node.py \
  src/robosar/robosar_emergency_response/config/response_safety.yaml \
  src/robosar/robosar_emergency_response/test/test_checkpoint_controller.py
git commit -m "refactor: make response mux a hard safety boundary"
```

---

### Task 11: Add ROS and Gazebo acceptance tests

**Files:**

- Create: `src/robosar/robosar_emergency_response/test/response_motion_gate.test`
- Create: `src/robosar/robosar_emergency_response/test/test_response_motion_gate_ros.py`
- Create: `src/robosar/robosar_emergency_response/test/response_formation_integration.test`
- Create: `src/robosar/robosar_emergency_response/test/test_response_formation_integration_ros.py`
- Create: `src/robosar/robosar_emergency_response/scripts/response_acceptance_monitor.py`
- Modify: `src/robosar/robosar_emergency_response/CMakeLists.txt`
- Modify: `src/robosar/robosar_emergency_response/scripts/run_response_deform_demo.sh`
- Modify: `src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh`

- [x] **Step 1: Add automated motion-gate coverage**

The test publishes fresh nonzero deform and task commands before START and asserts ten consecutive `/tb_i/cmd_vel` messages are zero. Then it publishes START, verifies motion is allowed, publishes STOP, and verifies zero within 0.2 seconds.

- [x] **Step 2: Add no-map and no-path coverage**

Cases:

1. launch navigator without a map, publish a task, expect `MAP_UNAVAILABLE` and zero cmd;
2. publish a valid map with a sealed goal, expect `NO_STATIC_PATH` and no direct path;
3. publish a wide corridor map, expect a path whose mean lateral offset is near corridor center.

- [x] **Step 3: Add a reusable acceptance monitor**

The monitor subscribes center path/status, odom and cmd_vel and writes a JSON summary containing:

```json
{
  "nonzero_before_start": 0,
  "stop_latency_seconds": 0.0,
  "minimum_robot_distance": 0.0,
  "minimum_reported_clearance": 0.0,
  "maximum_center_path_error": 0.0,
  "maximum_rms_slot_error": 0.0,
  "goals_completed": 0,
  "home_completed": false,
  "failure_details": []
}
```

Actual numeric fields are populated from messages; zeros above are schema examples, not expected results. The process exits nonzero when any acceptance invariant fails.

- [x] **Step 4: Keep the acceptance harness lightweight**

Implementation review found that duplicating the pure planner/state tests in two ROS launch wrappers would add a second fixture without new behavior coverage. Gate selection is covered in pytest, map/no-path state is covered in gtest, and the reusable monitor observes the real ROS/Gazebo graph. The originally proposed `.test` wrappers were therefore intentionally not added.

In CMake:

```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/response_motion_gate.test)
  add_rostest(test/response_formation_integration.test)
endif()
```

- [x] **Step 5: Run automated tests**

```bash
catkin_make --pkg robosar_messages plan_manager robosar_emergency_response -DCATKIN_ENABLE_TESTING=ON
catkin_make run_tests_plan_manager run_tests_robosar_emergency_response
catkin_test_results
```

Expected: zero failures.

- [x] **Step 6: Run the waypoint demo headless**

```bash
./run_response_deform_demo.sh --headless --no-vglrun
```

In another shell on the demo master, publish START and later response HOME. The acceptance monitor must report five goals, no pre-start motion, no collisions, and `home_completed=true`.

- [x] **Step 7: Run Tepper scenario matrix**

Run and save one acceptance JSON per case:

1. START idle, STOP, START resume while EN_ROUTE;
2. STOP/START while TASK_CONTROL;
3. response HOME while stopped;
4. exploration HOME without response motion-state change;
5. search robots initially blocking the response path, then leaving;
6. SURROUND handoff;
7. TRANSPORT_APPROACH handoff and box completion;
8. missing `/sim_map` startup.

Use:

```bash
./run_tepper_response_demo.sh --headless --no-vglrun
```

Expected: every applicable case exits cleanly with no collision and the expected state transition; missing-map case fails explicitly while robots remain stopped.

- [x] **Step 8: Record the dirty-worktree checkpoint**

The commit step was deliberately skipped: the shared worktree contains extensive pre-existing user changes, and the user requested continuation in place. No files were staged or committed; exact verification state is recorded in the progress log.

```bash
git add \
  src/robosar/robosar_emergency_response/test \
  src/robosar/robosar_emergency_response/scripts/response_acceptance_monitor.py \
  src/robosar/robosar_emergency_response/scripts/run_response_deform_demo.sh \
  src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh \
  src/robosar/robosar_emergency_response/CMakeLists.txt
git commit -m "test: cover response navigation mission and safety flows"
```

---

### Task 12: Update documentation and run final verification

**Files:**

- Modify: `docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md`
- Modify: `docs/superpowers/specs/2026-07-07-response-formation-navigation-design.md`
- Modify: `README.md` if it still describes automatic response HOME or four DEFORM planners

- [x] **Step 1: Rewrite the current-implementation document**

Update node graph, topics, configuration and troubleshooting so it describes:

- one center navigator;
- default-closed motion gate;
- manual response HOME;
- clearance-weighted path;
- continuous affine slots;
- scan-based local safety;
- profile-specific arrival;
- explicit map/no-path errors.

Remove claims about four plan managers, scale enumeration, `deform_reached`, automatic home and dynamic obstacle cloud being required for correctness.

- [x] **Step 2: Verify no stale runtime references remain**

```bash
rg -n "deform_dispatch_bridge|response_group_coordinator|response_plan_manager_one_run|formation_scale_step|formation_cohesion|auto_return_home: true" \
  README.md docs src/robosar/robosar_emergency_response src/robosar_deform_vendor/plan_manager
```

Expected: matches only in historical analysis paragraphs that explicitly label the behavior as removed.

- [x] **Step 3: Run the full relevant verification suite**

```bash
catkin_make --pkg robosar_messages plan_manager robosar_emergency_response robosar_controller
python3 -m pytest -q src/robosar/robosar_emergency_response/test
catkin_make run_tests_plan_manager run_tests_robosar_emergency_response
catkin_test_results
./run_response_deform_demo.sh --dry-run --headless --no-vglrun
./run_tepper_response_demo.sh --dry-run --headless --no-vglrun
```

Expected:

- build exit code 0;
- pytest exit code 0;
- catkin test result reports zero failures;
- both dry runs resolve one formation navigator and the intended map topic.

- [x] **Step 4: Inspect final diff for accidental user-change loss**

```bash
git status --short
git diff --stat
git diff --check
```

Expected: no whitespace errors, no unexpected file reverts, and no unrelated VectorizedMultiAgentSimulator/submodule changes staged.

- [x] **Step 5: Record documentation without committing user work**

As in Task 11, no commit was created in the dirty shared worktree. The documents themselves are the handoff checkpoint.

```bash
git add \
  docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md \
  docs/superpowers/specs/2026-07-07-response-formation-navigation-design.md \
  README.md
git commit -m "docs: describe generalized response formation navigation"
```

Omit `README.md` from the commit if it required no change.

## 4. Test coverage map

### Requirement-to-task coverage

| Requirement | Implemented by | Verified by |
|---|---|---|
| R-001 unified run gate | Tasks 1, 2 | Task 11 motion-gate/startup tests |
| R-002 group-specific HOME | Task 2 | Task 11 HOME isolation/resume matrix |
| R-003 single center plan | Tasks 7, 9 | launch-structure test and demo monitor |
| R-004 corridor-center path | Task 4 | planner gtests and wide-corridor rostest |
| R-005 continuous non-rigid deformation | Task 5 | formation gtests and Gazebo deformation case |
| R-006 local dynamic avoidance | Tasks 6, 7, 10 | velocity gtests and dynamic-crossing Tepper case |
| R-007 map/no-path behavior | Tasks 4, 7 | no-map/no-path gtest and rostest |
| R-008 remove cohesion fallback | Tasks 5, 9 | source grep and formation-quality tests |
| R-009 task geometry | Tasks 3, 8 | Python geometry tests and Tepper task handoff |
| R-010 arrival judgment | Task 8 | profile-specific arrival gtests |
| R-011 observability | Tasks 7, 11 | status schema and acceptance JSON |
| R-012 environment-independent config | Tasks 9, 12 | shared-launch/config inspection and both demos |

```text
CONTROL COMMANDS
  START before goal ---------------- [unit + rostest]
  goal before START ---------------- [rostest]
  STOP in NAV_FORMATION ------------ [unit + rostest + Tepper]
  STOP in TASK_CONTROL ------------- [unit + Tepper]
  START resumes both phases -------- [unit + Tepper]
  exploration HOME isolation ------- [Tepper]
  response HOME from STOP ---------- [unit + Tepper]
  HOME pause/resume ---------------- [unit + Tepper]

GLOBAL PLANNING
  malformed/missing map ------------ [gtest + rostest]
  start/goal outside map ----------- [gtest]
  occupied start/goal -------------- [gtest]
  narrow static corridor ----------- [gtest]
  no diagonal corner cutting ------- [gtest]
  wide-corridor center preference -- [gtest + Gazebo]
  safe smoothing ------------------- [gtest]
  bounded replan rate -------------- [gtest + rostest]

FORMATION CONTROL
  physical minimum scale ----------- [gtest]
  independent longitudinal/lateral - [gtest + Gazebo]
  zero-mean local offset ----------- [gtest]
  continuous scale rate ------------ [gtest]
  imperfect visible square -------- [gtest + Gazebo]
  stale odom/scan ------------------ [gtest + rostest]
  dynamic crossing obstacle -------- [Tepper]

TASK HANDOFF
  waypoint center + quality -------- [gtest + demo]
  surround convex/angle coverage --- [gtest + Tepper]
  transport rear/two-row geometry -- [pytest + gtest + Tepper]
  task controller completion ------- [existing pytest + Tepper]
  home center + quality ------------ [pytest + demo/Tepper]
```

## 5. Performance budgets

- Clearance field: recompute only on a material map update; target under 100 ms for the Tepper map in the deployment container.
- Global A*: one instance, triggered by new goal/map or bounded replan; target under 100 ms on Tepper.
- Control loop: 10 Hz, all four robots in one callback; p95 callback time under 20 ms.
- Scan obstacle snapshot: bounded number of points per robot after angular/downsample filtering; no unbounded vectors.
- Debug paths/markers: replace previous message content; no append-only history.
- Dynamic replan: no more than 2 full plans per second during unchanged blockage.

The acceptance monitor records planning duration and control-loop overruns. A budget miss is a failed performance test, not a reason to reduce collision checks.

## 6. Failure modes and required evidence

| Failure mode | Test | Runtime handling | User-visible result |
|---|---|---|---|
| START is clicked while consumers are still starting | rostest with task source publishing latched permission before mux/navigator start | late consumers receive `/response_group/motion_enabled=true` | state and motion agree; no lost START |
| Map never appears | no-map rostest | zero command and ERROR | `MAP_UNAVAILABLE` |
| Center goal lies in wall | gtest + rostest | reject exact goal | `GOAL_IN_COLLISION` |
| Search robots block initial exit | Tepper scenario | local stop, bounded replans | `BLOCKED_DYNAMIC`, then resumes |
| One scan becomes stale | rostest | affected robot zero; group slows/stops | `SENSOR_STALE:tb_i` |
| One robot falls behind | formation gtest + Gazebo | continuous group quality slowdown | status reports max/RMS error; no leader hard-stop branch |
| Dynamic obstacle crosses one corner | Tepper scenario | local offset + safe velocity projection | visible temporary deformation, no collision |
| Transport box contact appears in scan | hard-safety pytest | only expected box rays excluded | box can be pushed; walls/robots remain protected |
| STOP during task action | state test + Tepper | zero cmd, preserve ACTING | START resumes TASK_CONTROL |
| STOP during HOME | state test + Tepper | zero cmd, preserve home task | START resumes RETURNING_HOME |

## 7. Scope explicitly excluded

- Re-training checkpoint policies.
- Replacing the exploration group planner/controller.
- ROS2/Webots bridging.
- General N-robot assignment beyond preserving current support; acceptance remains four TB3 response robots.
- Formal continuous-time collision proof. The implementation provides tested discrete-time safety projection plus a final hard stop; a formal CBF/MPC layer is a separate research-grade project.

## 8. Execution handoff

Tasks 1–12 and the single-chain migration are complete. The legacy response navigator, bridge and coordinator have been removed; do not reintroduce a parallel planner or fallback chain. The final clean run completed all seven tasks, response HOME, exploration HOME and a nine-car zero-speed STOP. Rejected candidate findings—including cross-group braking margin, post-transport egress, continuous formation yaw, moving-obstacle soft costs, atomic mission gates, asymmetric close-range yielding and coordinated cyclic roundflocking—are implemented and covered by tests. Exact evidence is in the progress log; further changes start a new regression/extension phase rather than reopening this plan.

Continuation rules established by the runtime findings:

- Treat planning width (`robot_width`) and longitudinal contact length (`robot_length`) as separate geometry. ALIGN is collision-free; only BOX/PUSH may close response-response spacing to the URDF length.
- START/STOP/HOME are authoritative state transitions at the final command owner. Never rely on killing an upstream planner to stop a robot, and never leave START as a level-triggered pending flag.
- A center start inside the conservative formation envelope may be released only when all actual robot footprints are map-safe, and only along nondecreasing clearance until the normal envelope becomes feasible.
- Keep slot identity stable for the duration of one task. Path replanning may change the center path but must not silently permute robots; formation yaw follows discrete path tangents continuously, with corner tangential speed bounded by the center-speed budget. Near-zero projected velocities use formation yaw rather than noisy candidate or unrelated task-terminal yaw.
- Do not run exploration HOME through an active response task as an acceptance shortcut; validate the two HOME chains sequentially after response task completion.

Because the current worktree contains many user edits in the same files, every commit must stage explicit paths, inspect cached hunks, and preserve unrelated changes. If a relevant file contains inseparable pre-existing edits, skip the commit and continue with a documented dirty-worktree checkpoint rather than rewriting user work.
