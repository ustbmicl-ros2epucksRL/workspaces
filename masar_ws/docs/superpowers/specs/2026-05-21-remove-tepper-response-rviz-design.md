# Remove `tepper_response_group.rviz` From Tepper Response Demo

## Goal

When starting `run_tepper_response_demo.sh`, do not launch the response-group
overlay RViz window backed by `tepper_response_group.rviz`.

## Scope

- Remove only the response overlay RViz logic tied to
  `tepper_response_group.rviz`.
- Keep the base Tepper simulator's existing `rviz` behavior unchanged.

## Current Behavior

`run_tepper_response_demo.sh` launches two independent RViz-related paths:

1. The base Tepper simulator launch receives `rviz:=...`.
2. The response overlay launch receives `response_rviz:=...`, which is threaded
   through `tepper_response_demo.launch` into `response_group_gazebo.launch`
   and loads `config/tepper_response_group.rviz`.

The second path is the window the user wants removed.

## Design

### Launch Changes

- Delete `response_rviz` and `response_rviz_config` args from
  `tepper_response_demo.launch`.
- Stop passing overlay RViz args into `response_group_gazebo.launch` from
  `tepper_response_demo.launch`.
- Leave `response_group_gazebo.launch` unchanged because other demos still use
  its generic RViz support.

### Script Changes

- Remove the `response_rviz:=...` passthrough assembly from
  `src/robosar/robosar_emergency_response/scripts/run_tepper_response_demo.sh`.
- Keep the base `rviz:=...` argument passed to
  `robosar_sim_bringup robosar_sim_tepper.launch`.

### File Cleanup

- Delete `src/robosar/robosar_emergency_response/config/tepper_response_group.rviz`
  because it will no longer be referenced.

### Test Updates

- Update tests that currently assert:
  - `tepper_response_group.rviz` is referenced by the launch chain.
  - `response_rviz:=...` appears in dry-run output.
- Preserve tests that validate the base Tepper simulator `rviz:=...` behavior.

## Non-Goals

- Do not remove RViz support from `response_group_gazebo.launch`.
- Do not change other response demos such as `response_deform_demo.launch`,
  `roundflocking_controller_demo.launch`, or `transport_controller_demo.launch`.
- Do not change the base Tepper simulator's own RViz controls.

## Verification

- Run targeted Python tests covering the Tepper response launch script and
  launch-file wiring.
- Run a dry-run of `run_tepper_response_demo.sh` and confirm that:
  - `rviz:=...` is still passed to the base Tepper simulator.
  - `response_rviz:=...` is no longer emitted.
