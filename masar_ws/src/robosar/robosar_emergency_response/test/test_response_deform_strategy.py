from pathlib import Path

import yaml


WORKSPACE_DIR = Path(__file__).resolve().parents[4]
SRC_DIR = WORKSPACE_DIR / "src"
ASTAR_CPP = SRC_DIR / "robosar_deform_vendor" / "planning" / "path_searching2d" / "src" / "dyn_a_star.cpp"
ASTAR_H = SRC_DIR / "robosar_deform_vendor" / "planning" / "path_searching2d" / "include" / "path_searching" / "dyn_a_star.h"
PLAN_MANAGER_LAUNCH = (
    SRC_DIR
    / "robosar"
    / "robosar_emergency_response"
    / "launch"
    / "response_plan_manager_one_run.xml"
)
GRID_MAP_CPP = (
    SRC_DIR
    / "robosar_deform_vendor"
    / "perception"
    / "plan_env2d"
    / "src"
    / "grid_map.cpp"
)
PLAN_MANAGER_CPP = (
    SRC_DIR
    / "robosar_deform_vendor"
    / "plan_manager"
    / "src"
    / "plan_manager.cpp"
)
PLAN_MANAGER_H = (
    SRC_DIR
    / "robosar_deform_vendor"
    / "plan_manager"
    / "include"
    / "plan_manager"
    / "plan_manager.h"
)
TRAJ_OPTI_CPP = (
    SRC_DIR
    / "robosar_deform_vendor"
    / "control"
    / "traj_opti"
    / "src"
    / "traj_opti.cpp"
)
TRAJ_OPTI_H = (
    SRC_DIR
    / "robosar_deform_vendor"
    / "control"
    / "traj_opti"
    / "include"
    / "traj_opti"
    / "traj_opti.h"
)


def test_astar_recovers_when_current_pose_is_marked_occupied():
    header = ASTAR_H.read_text()
    source = ASTAR_CPP.read_text()

    assert "findNearestFreeIndex" in header
    assert "findNearestFreeIndex" in source
    assert "use_esdf_check" in source
    assert "Recovered occupied start point" in source


def test_astar_simple_path_handles_search_failure_without_stale_path():
    source = ASTAR_CPP.read_text()

    assert "bool search_ok" in source
    assert "path.empty()" in source
    assert "path.push_back(start_pt)" in source
    assert "A* failed, falling back to a direct local segment" in source


def test_plan_manager_launch_exposes_robust_response_demo_parameters():
    launch = PLAN_MANAGER_LAUNCH.read_text()

    assert "plan_manager/max_frm_type" in launch
    assert "plan_manager/approach_goal_distance" in launch
    assert "plan_manager/approach_center_goal" in launch
    assert '<param name="grid_map/odom_depth_timeout" value="8.0"/>' in launch


def test_grid_map_depth_timeout_is_throttled_warning_not_runtime_error():
    source = GRID_MAP_CPP.read_text()
    timeout_block = source.split("odom or depth lost!", 1)[0].rsplit("if (md_.flag_use_depth_fusion", 1)[1]

    assert "ROS_WARN_THROTTLE" in timeout_block
    assert "ROS_ERROR" not in timeout_block


def test_response_mux_publishes_laser_based_formation_pressure():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_cmd_mux_node.py"
    ).read_text()
    config = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "config"
        / "response_group.yaml"
    ).read_text()

    assert "formation_reduce_distance" in config
    assert "formation_clear_distance" in config
    assert "formation_ignore_robot_angle_deg" in config
    assert "scan_sector_ranges" in source
    assert "all_min" in source
    assert "excluded_angles=excluded_angles" in source
    assert "/{}/is_avoid" in source
    assert "Int16" in source


def test_plan_manager_can_disable_internal_esdf_avoid_publication():
    source = PLAN_MANAGER_CPP.read_text()
    header = PLAN_MANAGER_H.read_text()

    assert "publish_esdf_avoid_state" in header
    assert "publish_esdf_avoid_state" in source
    assert "if (publish_esdf_avoid_state)" in source
    assert "is_avoid_obs_pub_.publish(temp_avoid_mode);" in source


def test_response_mux_excludes_known_teammates_from_actual_safety_sectors():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_cmd_mux_node.py"
    ).read_text()

    scan_callback = source.split("def _scan_callback", 1)[1].split(
        "def _odom_callback", 1
    )[0]

    assert "excluded_angles = self._teammate_bearings(robot_name)" in scan_callback
    assert "excluded_angles=excluded_angles" in scan_callback
    assert "excluded_half_angle=self.ignore_robot_half_angle" in scan_callback


def test_response_mux_exposes_runtime_stuck_recovery_parameters():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_cmd_mux_node.py"
    ).read_text()
    config = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "config"
        / "response_group.yaml"
    ).read_text()

    assert "wall_stop_distance" in config
    assert "wall_side_stop_distance" in config
    assert "safety_escape_turn_speed" in config
    assert "stuck_recovery_enabled" in config
    assert "recovery_reverse_speed" in config
    assert "apply_scan_safety" in source
    assert "apply_stuck_recovery" in source
    assert "_update_stuck_state" in source


def test_plan_manager_does_not_expand_formation_while_any_robot_near_obstacle():
    source = PLAN_MANAGER_CPP.read_text()

    assert "avoid_sum > 0" in source
    assert "suppress formation expansion" in source
    assert "formation_expansion_cooldown" in source
    assert "last_formation_reduce_time_" in source
    assert "is_avoid_obs.assign(fixed_num, 0)" not in source
    assert "widthHandleResult = true;" not in source


def test_plan_manager_throttles_timeout_reassignment_and_single_file_reduce_logs():
    header = PLAN_MANAGER_H.read_text()
    source = PLAN_MANAGER_CPP.read_text()

    assert "reassignment_cooldown" in header
    assert "last_reassignment_time_" in header
    assert "suppress reassignment for" in source
    assert "already at minimum formation type" in source


def test_response_demo_formation_switching_is_stable_but_not_overly_conservative():
    launch = PLAN_MANAGER_LAUNCH.read_text()

    assert '<param name="plan_manager/formation_reduce_hold_time" value="0.6"/>' in launch
    assert '<param name="plan_manager/formation_expand_hold_time" value="1.5"/>' in launch
    assert '<param name="plan_manager/formation_expansion_cooldown" value="4.0"/>' in launch
    assert '<param name="plan_manager/reassignment_cooldown" value="5.0"/>' in launch


def test_plan_manager_uses_rotated_slots_for_navigation_and_origin_slots_for_final_approach():
    header = PLAN_MANAGER_H.read_text()
    source = PLAN_MANAGER_CPP.read_text()

    assert "isDirectGoalApproachSafe" in header
    assert "getApproachGoal" in header
    assert "isDirectGoalApproachSafe(tb_global_goal)" in source
    assert "dist_to_rotated_goal=((swarm_des[tb_id]+global_goal)-tb_states_[tb_id].pos).norm()" in source
    assert "dist_to_final_goal < approach_goal_dist" in source
    assert "return approach_center_goal ? global_goal : global_goal + swarm_des_origin[tb_id]" in source
    assert "dist_to_rotated_goal < 1.0 && dist_to_final_goal < 1.0" not in source
    assert "tb_global_goal=global_goal+swarm_des[tb_id]" not in source
    assert "Direct goal approach blocked" in source


def test_plan_manager_keeps_reached_robots_active_until_group_goal_changes():
    source = PLAN_MANAGER_CPP.read_text()

    approach_success = source.split("if(trajController(tb_global_goal))", 1)[1].split(
        "break;", 1
    )[0]
    sub_goal = source.split("void PlanManager::subGlobalGoal", 1)[1].split(
        "std::vector<Eigen::Vector2d> PlanManager::generateFormationPositions", 1
    )[0]

    assert "group coordinator publishes the next goal" in approach_success
    assert "has_goal=false" not in approach_success
    assert "WAIT_ODOM_OR_TARGET" not in approach_success
    assert "changeFRMExecState(FRM_INIT_PATH)" in sub_goal


def test_plan_manager_never_normalizes_zero_length_goal_vectors_into_nan():
    header = PLAN_MANAGER_H.read_text()
    source = PLAN_MANAGER_CPP.read_text()

    assert "sanitizeControllerGoal" in header
    assert "sanitizeControllerGoal(goal)" in source
    assert "to_goal_dist < 1e-3" in source
    assert "std::isfinite" in source
    assert "(goal - tb_states_[tb_id].pos) / (goal - tb_states_[tb_id].pos).norm()" not in source


def test_traj_optimizer_falls_back_when_nmpc_or_casadi_rejects_invalid_values():
    header = TRAJ_OPTI_H.read_text()
    source = TRAJ_OPTI_CPP.read_text()

    assert "fallbackVelocity" in header
    assert "isFiniteVector" in header
    assert "safeSetLocalGoal" in header
    assert "casadi::CasadiException" in source
    assert "fallbackVelocity()" in source
    assert "Opti::set_value" not in source.split("void TrajectoryOptimizer::setLocalGoal", 1)[1].split(
        "void TrajectoryOptimizer::setGlobalGoal", 1
    )[0]


def test_plan_manager_uses_astar_path_when_path_optimizer_fails():
    source = PLAN_MANAGER_CPP.read_text()
    path_planning = source.split("void PlanManager::pathPlanning", 1)[1].split(
        "void PlanManager::widthHandleInExtend", 1
    )[0]

    assert "const int opt_ret = path_opti_->startOpt(path_)" in path_planning
    assert "Path optimizer failed; keeping A* fallback path" in path_planning


def test_coordinator_does_not_trust_stale_deform_reached_without_slot_barrier():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_group_coordinator_node.py"
    ).read_text()

    assert "result.all_inside" in source
    assert "deform_reached_and_inside" in source
    assert "goal_region_result.arrived" in source


def test_coordinator_resets_reached_flags_when_formation_assignment_changes():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_group_coordinator_node.py"
    ).read_text()
    formation_callback = source.split("def _formation_callback", 1)[1].split(
        "def _odom_callback", 1
    )[0]

    assert "self.deform_reached = {name: False for name in self.robot_names}" in formation_callback


def test_coordinator_checks_deform_reached_against_unrotated_final_slots():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_group_coordinator_node.py"
    ).read_text()

    timer_callback = source.split("def _timer_callback", 1)[1].split(
        "def _pose_stamped", 1
    )[0]

    assert "slot_rotation_yaw=0.0" in timer_callback
    assert "slot_rotation_yaw=self.formation_yaw" not in timer_callback


def test_response_group_config_exposes_goal_region_arrival_parameters():
    config = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "config"
        / "response_group.yaml"
    ).read_text()

    assert "enable_goal_region_arrival" in config
    assert "goal_region_robot_radius" in config
    assert "goal_region_center_radius" in config


def test_response_group_laser_safety_keeps_wall_corner_margin():
    with (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "config"
        / "response_group.yaml"
    ).open() as stream:
        config = yaml.safe_load(stream)

    assert config["wall_slowdown_distance"] >= 0.60
    assert config["wall_stop_distance"] >= 0.22
    assert config["wall_side_stop_distance"] >= 0.22
    assert config["safety_front_angle_deg"] >= 100.0
    assert config["formation_reduce_distance"] >= 0.50
    assert config["formation_clear_distance"] >= 0.65
    assert config["stuck_obstacle_distance"] >= 0.60


def test_response_mux_applies_laser_safety_once_with_sector_context():
    source = (
        SRC_DIR
        / "robosar"
        / "robosar_emergency_response"
        / "scripts"
        / "response_cmd_mux_node.py"
    ).read_text()
    timer_callback = source.split("def _timer_callback", 1)[1].split(
        "def _update_stuck_state", 1
    )[0]

    assert "apply_scan_safety(" in timer_callback
    assert "safety_min_range=self.min_ranges[name]" not in timer_callback
    assert timer_callback.count("select_command(") == 1


def test_plan_manager_recenters_local_goal_inside_detected_corridor():
    source = PLAN_MANAGER_CPP.read_text()
    header = PLAN_MANAGER_H.read_text()
    launch = PLAN_MANAGER_LAUNCH.read_text()

    assert "center_local_goal_in_corridor" in header
    assert "corridor_centering_max_width" in header
    assert "corridor_centering_max_shift" in header
    assert "getWidthBestPoint()" in source
    assert "corridor_centering_max_width" in source
    assert "Corridor centering adjusted local goal" in source
    assert "plan_manager/center_local_goal_in_corridor" in launch


def test_plan_manager_relaxes_formation_consensus_in_narrow_corridors():
    source = PLAN_MANAGER_CPP.read_text()
    header = PLAN_MANAGER_H.read_text()
    launch = PLAN_MANAGER_LAUNCH.read_text()

    assert "formation_consensus_local_goal_weight" in header
    assert "narrow_corridor_local_goal_weight" in header
    assert "avoid_sum > 0 && formation_type <= 1" in source
    assert "narrow corridor local-goal tracking" in source
    assert "plan_manager/narrow_corridor_local_goal_weight" in launch
