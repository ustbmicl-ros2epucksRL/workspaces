# Response DEFORM Demo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use $superpower-executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking via update_plan.

**Goal:** Build a ROS1/Gazebo/RViz response-demo closed loop where four `tb_0..tb_3` TurtleBots use DEFORM-style variable formation navigation through a multi-obstacle world and automatically visit a sequence of goals.

**Architecture:** Add a new `robosar_emergency_response` ROS package that owns response-group orchestration, command muxing, markers, response-demo goal sequencing, and launch/config. Keep DEFORM vendor code as the navigation controller and wrap it with response-specific launch files so its velocity output is remapped through a mux before reaching `/tb_i/cmd_vel`.

**Tech Stack:** ROS1 Noetic, catkin, rospy, geometry_msgs/nav_msgs/std_msgs/visualization_msgs, existing `robosar_messages`, copied DEFORM packages under `src/robosar_deform_vendor`.

---

### Task 1: Pure Logic Tests

**Files:**
- Create: `src/robosar/robosar_emergency_response/test/test_response_demo_logic.py`
- Create: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/__init__.py`
- Create: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/formation.py`
- Create: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/goal_sequence.py`
- Create: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/coordinator.py`
- Create: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/cmd_mux.py`
- Create: `src/robosar/robosar_emergency_response/src/robosar_emergency_response/dispatch.py`

- [ ] Write failing pytest coverage for 4-robot DEFORM slot generation, goal sequence advancement only after matching DONE status, arrival hold timing, mux mode/stale/safety selection, and goal dispatch pose conversion.
- [ ] Run: `PYTHONPATH=src/robosar/robosar_emergency_response/src pytest -q src/robosar/robosar_emergency_response/test/test_response_demo_logic.py`.
- [ ] Implement the minimal pure Python logic modules.
- [ ] Re-run the same pytest command and keep it green.

### Task 2: ROS Package And Nodes

**Files:**
- Create: `src/robosar/robosar_emergency_response/package.xml`
- Create: `src/robosar/robosar_emergency_response/CMakeLists.txt`
- Create: `src/robosar/robosar_emergency_response/setup.py`
- Create: `src/robosar/robosar_emergency_response/scripts/response_goal_sequence_node.py`
- Create: `src/robosar/robosar_emergency_response/scripts/deform_dispatch_bridge_node.py`
- Create: `src/robosar/robosar_emergency_response/scripts/response_group_coordinator_node.py`
- Create: `src/robosar/robosar_emergency_response/scripts/response_cmd_mux_node.py`

- [ ] Add package metadata and Python install wiring.
- [ ] Add ROS node wrappers that load params, publish/subscribe the response-demo topics, and delegate behavior to tested pure modules.
- [ ] Run Python syntax checks for all new scripts and modules.

### Task 3: Config And Launch

**Files:**
- Create: `src/robosar/robosar_emergency_response/config/response_group.yaml`
- Create: `src/robosar/robosar_emergency_response/config/response_goal_sequence.yaml`
- Create: `src/robosar/robosar_emergency_response/config/deform/frm_shape_response4.yaml`
- Create: `src/robosar/robosar_emergency_response/launch/response_deform_demo.launch`
- Create: `src/robosar/robosar_emergency_response/launch/response_group_gazebo.launch`
- Create: `src/robosar/robosar_emergency_response/launch/response_group_deform.launch`
- Create: `src/robosar/robosar_emergency_response/launch/response_plan_manager_one_run.xml`

- [ ] Configure four response robots, nominal DEFORM slots, arrival thresholds, mux timeouts, and a three-goal response-demo route through the DEFORM multi-obstacle world.
- [ ] Launch Gazebo with the copied DEFORM `test_multi_obs.world`, spawn only four response TurtleBots, publish `map -> world`, start four `plan_manager_node` instances with `/tb_i/deform_cmd_vel`, and start the response nodes.
- [ ] Run XML checks and `roslaunch --nodes` on the response-demo launch file if ROS tooling is available.

### Task 4: Verification

**Files:**
- Modify only if needed: response-demo files created above.

- [ ] Run unit tests.
- [ ] Run compile/syntax checks.
- [ ] Run targeted catkin validation for `robosar_emergency_response` and the DEFORM packages if dependencies are installed.
- [ ] If Gazebo can run in the environment, launch the response demo and verify topics/nodes; otherwise document the exact command and what remains unverified.

---

**Self-review:** This plan covers the response-demo requirements in `docs/ROBOSAR_DEFORM_INTEGRATION_DESIGN.md`: four response robots, DEFORM controller wiring, automated goal sequence, arrival barrier, cmd mux, Gazebo/RViz launch, and testable modular code. No ROS2/Webots or first-phase emergency task manager work is included.
