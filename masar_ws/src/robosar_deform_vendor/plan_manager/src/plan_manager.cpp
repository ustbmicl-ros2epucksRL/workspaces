#include "plan_manager/plan_manager.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

PlanManager::PlanManager() : nh_(), nh_pri_("~"){

//    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
//        ros::console::notifyLoggerLevelsChanged();
//    }

    readParameters();

    tb_id = fixed_id;
    if (formation_max_num < 1 || formation_max_num > fixed_num)
        formation_max_num = fixed_num;
    formation_min_type = std::max(1, std::min(formation_min_type, formation_max_num));
    if (start_frm_type < formation_min_type || start_frm_type > formation_max_num)
        start_frm_type = formation_max_num;

    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh_pri_);
    aster_.reset(new AStar);
    aster_->initGridMap(grid_map_,Eigen::Vector2d(80,80));
    find_region_.reset(new FindMaxFreeRegion);
    find_region_->initMapandMaxGap(grid_map_, (formation_max_num - 1) * formation_interval + formation_interval);
    path_opti_.reset(new PathOptimizer);
    path_opti_->setGridMapandObsClearance(grid_map_,obs_clearance);
    traj_opt_.reset(new TrajectoryOptimizer(nh_pri_));
    ot_cli_.reset(new OT_Transfer);
    ot_cli_->Init(nh_pri_);
    to_vis_.reset(new toVisualization);
    to_vis_->init(nh_pri_);

    swarm_des_origin.resize(fixed_num);
    swarm_alloc.resize(fixed_num);
    tb_states_.resize(fixed_num);
    odoms_sub.resize(fixed_num);
    realloc_.resize(fixed_num);
    all_width.resize(fixed_num,0.2);
    is_avoid_obs.resize(fixed_num, 0);
    is_avoid_obs_sub_.resize(fixed_num);

    for (int i = 0; i < fixed_num; i++) {

        realloc_[i] = i;

        if(i != tb_id) {
            other_idx.push_back(i);
        }
    }

    goal_sub = nh_.subscribe("/move_base_simple/goal", 1, &PlanManager::subGlobalGoal, this);
    vel_pub = nh_pri_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    is_avoid_obs_pub_ = nh_pri_.advertise<std_msgs::Int16>("/tb_" + std::to_string(fixed_id) + "/is_avoid", 1);
    deform_reached_pub_ = nh_pri_.advertise<std_msgs::Bool>("/tb_" + std::to_string(fixed_id) + "/deform_reached", 1, true);
    debug_pub_ = nh_pri_.advertise<std_msgs::String>("/tb_" + std::to_string(fixed_id) + "/deform_debug", 1);
    for (int id = 0; id < fixed_num; id++) {
        is_avoid_obs_sub_[id] = nh_.subscribe<std_msgs::Int16>("/tb_" + std::to_string(id) + "/is_avoid", 1, [=](const std_msgs::Int16ConstPtr& msg) {
            // ROS_WARN("receive avoid_mode info");
            is_avoid_obs[id] = msg->data;
        });
    }

    //get all turtlebot position info
    for (int id = 0; id < fixed_num; id++)
        odoms_sub[id] = nh_.subscribe<nav_msgs::Odometry>("/tb_" + std::to_string(id) + "/odom", 1, [=](const nav_msgs::OdometryConstPtr& msg){
            tb_states_[realloc_[id]].pos.x() = msg->pose.pose.position.x;
            tb_states_[realloc_[id]].pos.y() = msg->pose.pose.position.y;
            tb_states_[realloc_[id]].yaw = tf2::getYaw(msg->pose.pose.orientation);
            tb_states_[realloc_[id]].vel.x() = msg->twist.twist.linear.x;
            tb_states_[realloc_[id]].vel.y() = msg->twist.twist.angular.z;
            Eigen::Quaterniond q;
            tf2::fromMsg(msg->pose.pose.orientation, q);
            Eigen::Matrix2d r_2d = q.toRotationMatrix().block<2,2>(0, 0);
            Eigen::Vector2d unit_x_axis(1.0, 0.0);
            tb_states_[realloc_[id]].orient_vec = r_2d * unit_x_axis;

            if (realloc_[id] == tb_id) {
                Eigen::Vector3d state(tb_states_[tb_id].pos.x(), tb_states_[tb_id].pos.y(), tb_states_[tb_id].yaw);
                traj_opt_->updateState(state,tb_states_[tb_id].vel);
                has_odom = true;
            }
        });

    path_point_pub = nh_.advertise<geometry_msgs::Point>("/path_point_sharing", 1);
    path_point_sub = nh_.subscribe<geometry_msgs::Point>("/path_point_sharing", 1, &PlanManager::setPathPoint, this);

    frm_info_pub = nh_.advertise<plan_manager::RotationVec>("/formation_info", 1);
    frm_info_sub = nh_.subscribe("/formation_info", 4, &PlanManager::updateFRMInfo, this);

    width_pub = nh_.advertise<plan_manager::widthInfo>("/current_width", 1);
    width_sub = nh_.subscribe<plan_manager::widthInfo>("/current_width", 1, [=](const plan_manager::widthInfoConstPtr& msg){
        all_width[msg->id] = msg->width;
    });

    execTimer = nh_.createTimer(ros::Duration(0.2), &PlanManager::execCallback, this);
    pathTimer = nh_.createTimer(ros::Duration(0.4),&PlanManager::pathPlanning, this, false, false);
    exec_state_= FORMATION_EXEC_STATE::WAIT_ODOM_OR_TARGET;

    // sorted the distance between other robot and current robot, used to reduce traj constraints
    updateOtherTimer = nh_.createTimer(ros::Duration(0.4), &PlanManager::updateOtherCallback, this, false, false);

    //If many robots enter obstacle avoidance mode, shrink the formation.
    avoidModeTimer = nh_.createTimer(ros::Duration(2.0), &PlanManager::updateAvoidModeCallback, this, false, false);

    timeoutTimer = nh_.createTimer(ros::Duration(2.0), &PlanManager::timeoutCallback, this, false, false);

    if (task_type == TASK_TYPE::NAVIGATION) {
        if (fixed_id == leader_id)
            ROS_WARN("Task type is navigation");
        formation_type = start_frm_type; // fixed_num : the widthest formation, 1 : the narrowest formation
        setDesiredShape(formation_type);
    } else if ( task_type ==  TASK_TYPE::DEFORM) {
        if (fixed_id == leader_id)
            ROS_WARN("Task type is deform");
        for (int i = 1; i <= swarm_des_origin.size(); i++) {
            nh_pri_.param("desired_shape_positions/vehicle_pos_" + to_string(i) + "/x", swarm_des_origin[i - 1][0], -1.0);
            nh_pri_.param("desired_shape_positions/vehicle_pos_" + to_string(i) + "/y", swarm_des_origin[i - 1][1], -1.0);
        }
    } else {
        ROS_ERROR("task_type invalid");
        throw std::runtime_error("task_type invalid");
    }

    //init foramtion type
    swarm_des = swarm_des_origin;
    formation_yaw = 0.0; //The initial formation orientation is aligned with the x-axis.

    has_goal=false;
    has_odom= false;
    do_allocation=false;
    first_start= false;//startup to one do_allocation
    is_timeout = false;
    reported_reached_current_slot_ = false;
    reported_reached_slot_ = Eigen::Vector2d::Zero();
    avoid_reduce_since_ = ros::Time(0);
    width_reduce_since_ = ros::Time(0);
    expand_clear_since_ = ros::Time(0);
    last_formation_reduce_time_ = ros::Time(0);
    last_reassignment_time_ = ros::Time(0);
    last_slot_reassignment_check_time_ = ros::Time(0);
    last_slot_reassignment_apply_time_ = ros::Time(0);
}

PlanManager::~PlanManager() {
}

void PlanManager::timeoutCallback(const ros::TimerEvent&) {

    if(fixed_id != leader_id || !timeout_reassignment_enabled)
        return;

    if (is_timeout) {
        const double since_reassignment = last_reassignment_time_.isZero()
                                              ? std::numeric_limits<double>::infinity()
                                              : (ros::Time::now() - last_reassignment_time_).toSec();
        if (since_reassignment < reassignment_cooldown) {
            ROS_WARN_THROTTLE(2.0,
                              "suppress reassignment for %.1fs after the previous timeout reassignment",
                              reassignment_cooldown - since_reassignment);
            is_timeout = false;
            return;
        }
        ROS_WARN("Timeout, reassignment!");
        do_allocation= true;
        last_reassignment_time_ = ros::Time::now();
        is_timeout = false;
        return;
    }

    // When the formation gets "stuck" during navigation
    double cur_linear_vel = tb_states_[0].vel[0];
    double cur_angular_vel = tb_states_[0].vel[1];

    double linear_vel_th = 0.05;
    double angular_vel_th = 0.07;

    // ROS_WARN("Timeout true test, frm bias: %.2f, cur_lin_vel: %.2f, cur_ang_vel: %.2f ", cur_bias, cur_linear_vel, cur_angular_vel);
    if (cur_bias > 0.6 &&
        cur_linear_vel < linear_vel_th &&
        abs(cur_angular_vel) < angular_vel_th ) {
        is_timeout = true;
    }
}

void PlanManager::updateAvoidModeCallback(const ros::TimerEvent&) {

    if(fixed_id != leader_id)
        return;

    int in_avoid_num_th = std::max(
        1,
        static_cast<int>(std::ceil(std::max(0.0, formation_reduce_robot_fraction) * fixed_num)));
    in_avoid_num_th = std::min(fixed_num, in_avoid_num_th);

    int avoid_sum = 0;
    for (int i = 0; i < is_avoid_obs.size(); i++) {
        avoid_sum += is_avoid_obs[i];
    }

    const bool sustained_pressure = conditionHeld(
        avoid_reduce_since_,
        avoid_sum > 0 && avoid_sum >= in_avoid_num_th,
        formation_reduce_hold_time);

    if (sustained_pressure) {

        if(formation_type > formation_min_type) {
            formation_type -= 1;
            setDesiredShape(formation_type);
            do_allocation=true;
            markFormationReduced();
            rotateAndShareFRM();
            ROS_WARN("near-obstacle pressure held by %d robot(s), reduce formation", avoid_sum);
        } else {
            ROS_WARN_THROTTLE(2.0,
                              "near-obstacle pressure held by %d robot(s), already at minimum formation type",
                              avoid_sum);
        }
    } else if (avoid_sum > 0) {
        ROS_WARN_THROTTLE(2.0,
                          "near-obstacle pressure from %d robot(s), waiting for stable signal before formation change",
                          avoid_sum);
    }
}

void PlanManager::execCallback(const ros::TimerEvent &) {

    switch (exec_state_) {

        case FORMATION_EXEC_STATE::WAIT_ODOM_OR_TARGET:
        {
            if(has_goal && has_odom)
                changeFRMExecState(FRM_INIT_PATH);
            break;
        }

        case FORMATION_EXEC_STATE::FRM_INIT_PATH:
        {

            pathPlanning(ros::TimerEvent{});

            // frm_rrt need
            // grid_map_->updateESDFMap(Eigen::Vector2d{60, 30});

            pathTimer.start();
            updateOtherTimer.start();
            avoidModeTimer.start();
            timeoutTimer.start();

            changeFRMExecState(FRM_KEEP);
            break;
        }

        case FORMATION_EXEC_STATE::FRM_KEEP:
        {

            if (task_type == TASK_TYPE::NAVIGATION) {
                double dist_to_rotated_goal=((swarm_des[tb_id]+global_goal)-tb_states_[tb_id].pos).norm();
                tb_global_goal=getApproachGoal();
                double dist_to_final_goal=(tb_global_goal-tb_states_[tb_id].pos).norm();
                const bool near_final_slot = dist_to_final_goal < approach_goal_dist;
                const bool near_rotated_slot_and_final_area =
                    dist_to_rotated_goal < approach_goal_dist &&
                    dist_to_final_goal < approach_goal_dist + 0.5;
                if (near_final_slot || near_rotated_slot_and_final_area) {
                    if (isDirectGoalApproachSafe(tb_global_goal)) {
                        changeFRMExecState(APPROACH_GOAL);
                    } else {
                        ROS_WARN_THROTTLE(1.0,
                                          "Direct goal approach blocked for tb %d; keep formation navigation.",
                                          fixed_id);
                        trajPlanning();
                    }
                }
                else {
                        trajPlanning();
                }
            } else if (task_type == TASK_TYPE::DEFORM) {
                double traj_e;
                FRMBias(traj_e);
                // ROS_INFO("current bias: %.2f", traj_e);
                if (traj_e < 0.4) {
                    ROS_WARN("Convergence success!");
                    std::vector<double> c;
                    c.push_back(0.0);
                    c.push_back(0.0);
                    pubVelCmd(c);
                    execTimer.stop();
                    timeoutTimer.stop();
                }
                else
                    trajPlanningDeform();
            }

            break;
        }

        case FORMATION_EXEC_STATE::FRM_DEFORM:
        {
            break;
        }

        case FORMATION_EXEC_STATE::AVOID_OBS:
        {
            std_msgs::Int16 temp_avoid_mode;
            temp_avoid_mode.data = 1;

            double esdf_dist=grid_map_->getDistance(tb_states_[tb_id].pos);
            // std::cout<<"avoid current dist="<<esdf_dist<<std::endl;
            if (esdf_dist < 0.35)
            {
                if (publish_esdf_avoid_state) {
                    is_avoid_obs_pub_.publish(temp_avoid_mode);
                }
                Eigen::Vector2d avoid_pt = tb_states_[tb_id].pos;
                path_opti_->setWeightToLocalOpt(1.0,2.0);
                path_opti_->ESDFAndLocalPointOpt(avoid_pt,0.4);
                to_vis_->showGlobalGoal(avoid_pt);
                trajController(avoid_pt);
            } else {
                changeFRMExecState(FRM_KEEP);
                if (publish_esdf_avoid_state) {
                    std_msgs::Int16 temp_avoid_mode;
                    temp_avoid_mode.data = 0;
                    is_avoid_obs_pub_.publish(temp_avoid_mode);
                }
            }
            break;
        }

        // 1m obstacle-free environment around goal
        case FORMATION_EXEC_STATE::APPROACH_GOAL:
        {
            tb_global_goal=getApproachGoal();
            traj_opt_->setGlobalGoal(tb_global_goal);
            if ((tb_global_goal - reported_reached_slot_).norm() > 0.05) {
                reported_reached_current_slot_ = false;
            }

            if (!isDirectGoalApproachSafe(tb_global_goal)) {
                ROS_WARN_THROTTLE(1.0,
                                  "Direct goal approach blocked for tb %d during approach; return to formation navigation.",
                                  fixed_id);
                changeFRMExecState(FRM_KEEP);
                break;
            }

            if(trajController(tb_global_goal))
            {
                std_msgs::Bool reached_msg;
                reached_msg.data = true;
                deform_reached_pub_.publish(reached_msg);
                if (!reported_reached_current_slot_) {
                    reported_reached_current_slot_ = true;
                    reported_reached_slot_ = tb_global_goal;
                    ROS_WARN("tb %d reached current slot; keep participating until the group coordinator publishes the next goal.", fixed_id);
                }
            }
            break;
        }

        case FORMATION_EXEC_STATE::EMERGENCY_STOP:
        {
            std::vector<double> v_cmd={0.0,0.0};
            pubVelCmd(v_cmd);
            break;
        }

        default:
            std::cout<<"formation_exec_state error"<<std::endl;
    }
}

void PlanManager::updateOtherCallback(const ros::TimerEvent&) {

    auto cur_pos = tb_states_[tb_id].pos;
    auto cmp = [&cur_pos](Eigen::Vector2d pos1, Eigen::Vector2d pos2) {
        double dist1 = (pos1 - cur_pos).norm();
        double dist2 = (pos2 - cur_pos).norm();
        return dist1 > dist2;
    };
    std::priority_queue<Eigen::Vector2d, std::vector<Eigen::Vector2d>, decltype(cmp)> pq(cmp);

    for (int i = 0; i < fixed_num - 1; i++) {
        pq.push(tb_states_[realloc_[other_idx[i]]].pos);
    }

    for(int i = 0; i < other_num_to_opti; i++) {

        traj_opt_->updateOtherOdom(pq.top(), i);
        pq.pop();
    }

}

void PlanManager::trajPlanningDeform() {

        if(fixed_id == leader_id) {
            rotateAndShareDeform();
        }

        Eigen::Vector2d temp_pt = global_goal+swarm_des[tb_id];
        to_vis_->showGlobalGoal(temp_pt);

        FRMConsensus(local_goal);
        trajController(local_goal);
}

// main logic
void PlanManager::trajPlanning() {

    double esdf_dist = grid_map_->getDistance(tb_states_[tb_id].pos);

    if(avoid_obs_distance_threshold > 0.0 && esdf_dist < avoid_obs_distance_threshold)
    {
       changeFRMExecState(AVOID_OBS);
      return;
    }

    findPointInPath(fwd_dist, local_goal);

    bool widthHandleResult;
    widthHandleResult=widthHandle(tb_states_[tb_id].pos,local_goal);
    if (widthHandleResult) {

        widthHandleInExtend();

        //test: check current shap in RViz
        Eigen::Vector2d temp_pt=global_goal+swarm_des[tb_id];
        to_vis_->showGlobalGoal(temp_pt);

            if (fixed_id == leader_id) {

            // ROS_WARN("leader robot: %d, current width:%f",fixed_id, find_region_->getWidth());

            // DEBUG width result
            to_vis_->showUpdateArea(find_region_->getBox());
            std::vector<Eigen::Vector2d> one_line, other_line;
            find_region_->test(one_line, other_line);
            to_vis_->showWidthLineOne(one_line);
            to_vis_->showWidthLineOther(other_line);

            double traj_e;
            FRMBias(traj_e);
            cur_bias = traj_e;

            isReduceFRM();

            // running this function is not considered
            isExtendFRM();

            rotateAndShareFRM();
        }

        FRMConsensus(local_goal);

        //show formation consensus local point and line
        std::vector<Eigen::Vector2d> odom_and_local_line;
        odom_and_local_line.push_back(tb_states_[tb_id].pos);
        odom_and_local_line.push_back(local_goal);
        to_vis_->showOdomAndLocalLine(odom_and_local_line);

        trajController(local_goal);
    }
    else
    {
        // follow the path
        trajController(local_goal);
    }
}

bool PlanManager::trajController(Eigen::Vector2d goal) {

    goal = sanitizeControllerGoal(goal);

    //nmpc controller
    double w_des, v_des;
    calcDesireLinearVel(v_des,goal);
    calcDesireAngularVel(w_des,goal);

    Eigen::Vector2d x_ref={goal.x(),goal.y()};
    Eigen::Vector2d u_ref={v_des,w_des};
    if (!traj_opt_->safeSetLocalGoal(x_ref,u_ref)) {
        const std::vector<double> fallback_cmd = traj_opt_->fallbackVelocity();
        pubVelCmd(fallback_cmd);
        publishDebugInfo(goal, v_des, w_des, false, fallback_cmd);
        to_vis_->showPath(traj_opt_->getState());
        return false;
    }

    bool reach_goal;
    reach_goal=traj_opt_->run();
    const std::vector<double> control_velocity = traj_opt_->getControlVelocity();
    pubVelCmd(control_velocity);
    publishDebugInfo(goal, v_des, w_des, reach_goal, control_velocity);

    // show predict traj
    to_vis_->showPath(traj_opt_->getState());

    return reach_goal;
}

bool PlanManager::isDirectGoalApproachSafe(const Eigen::Vector2d& goal) {
    const Eigen::Vector2d start = tb_states_[tb_id].pos;
    const Eigen::Vector2d delta = goal - start;
    const double length = delta.norm();

    if (length < 1e-3) {
        return true;
    }

    const double sample_step = std::max(0.05, std::min(0.10, grid_map_->getResolution()));
    const int sample_count = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
    const double min_clearance = std::max(0.30, obs_clearance);

    for (int i = 0; i <= sample_count; ++i) {
        const double ratio = static_cast<double>(i) / static_cast<double>(sample_count);
        const Eigen::Vector2d sample = start + ratio * delta;

        if (!grid_map_->isInMap(sample)) {
            return false;
        }

        const int inflated_occ = grid_map_->getInflateOccupancy(sample);
        if (inflated_occ != 0) {
            return false;
        }

        const double dist = grid_map_->getDistance(sample);
        if (std::isnan(dist) || (std::isfinite(dist) && dist < min_clearance)) {
            return false;
        }
    }

    return true;
}

Eigen::Vector2d PlanManager::getApproachGoal() const {
    return approach_center_goal ? global_goal : global_goal + swarm_des[tb_id];
}

Eigen::Vector2d PlanManager::sanitizeControllerGoal(const Eigen::Vector2d& goal) const {
    const auto is_finite_vec = [](const Eigen::Vector2d& value) {
        return std::isfinite(value.x()) && std::isfinite(value.y());
    };

    Eigen::Vector2d sanitized = goal;
    if (!is_finite_vec(sanitized)) {
        sanitized = tb_global_goal;
    }

    Eigen::Vector2d to_goal = sanitized - tb_states_[tb_id].pos;
    double to_goal_dist = to_goal.norm();
    if (!is_finite_vec(to_goal) || !std::isfinite(to_goal_dist) || to_goal_dist < 1e-3) {
        Eigen::Vector2d fallback_dir = tb_global_goal - tb_states_[tb_id].pos;
        if (!is_finite_vec(fallback_dir) || fallback_dir.norm() < 1e-3) {
            fallback_dir = global_goal - tb_states_[tb_id].pos;
        }
        if (!is_finite_vec(fallback_dir) || fallback_dir.norm() < 1e-3) {
            fallback_dir = tb_states_[tb_id].orient_vec;
        }
        if (!is_finite_vec(fallback_dir) || fallback_dir.norm() < 1e-3) {
            fallback_dir = Eigen::Vector2d(1.0, 0.0);
        }

        sanitized = tb_states_[tb_id].pos + fallback_dir.normalized() * 0.20;
        ROS_WARN_THROTTLE(1.0,
                          "Controller goal for tb %d was degenerate; nudging goal to %.3f %.3f.",
                          fixed_id,
                          sanitized.x(),
                          sanitized.y());
    }

    return sanitized;
}

double PlanManager::normalizeFormationYaw(double yaw) const {
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw < -M_PI) {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}

double PlanManager::snapFormationYawForCorridor(double yaw) const {
    if (!formation_corridor_yaw_snap_enabled) {
        return normalizeFormationYaw(yaw);
    }

    const double free_width = find_region_->getWidth();
    if (!std::isfinite(free_width) || free_width > formation_corridor_yaw_snap_max_width) {
        return normalizeFormationYaw(yaw);
    }

    const double axis = M_PI / 2.0;
    const double snapped = std::round(yaw / axis) * axis;
    const double error = normalizeFormationYaw(yaw - snapped);
    if (std::abs(error) > std::max(0.0, formation_corridor_yaw_snap_tolerance)) {
        return normalizeFormationYaw(yaw);
    }

    return normalizeFormationYaw(snapped);
}

bool PlanManager::conditionHeld(ros::Time& since, bool condition, double hold_time) const {
    if (!condition) {
        since = ros::Time(0);
        return false;
    }

    const ros::Time now = ros::Time::now();
    if (since.isZero()) {
        since = now;
    }
    return (now - since).toSec() >= std::max(0.0, hold_time);
}

void PlanManager::markFormationReduced() {
    last_formation_reduce_time_ = ros::Time::now();
    width_reduce_since_ = ros::Time(0);
    avoid_reduce_since_ = ros::Time(0);
    expand_clear_since_ = ros::Time(0);
}

void PlanManager::pathPlanning(const ros::TimerEvent&) {
    if(fixed_id != leader_id) {
        path_ = aster_->astarSearchAndGetSimplePath(grid_map_->getResolution(), tb_states_[tb_id].pos, leader_path_point);
    } else {
        path_ = aster_->astarSearchAndGetSimplePath(grid_map_->getResolution(), tb_states_[tb_id].pos, tb_global_goal);
    }

    if (path_.empty()) {
        ROS_WARN_THROTTLE(1.0, "A* returned an empty path; using current pose and target as fallback path.");
        path_.push_back(tb_states_[tb_id].pos);
        path_.push_back(fixed_id != leader_id ? leader_path_point : tb_global_goal);
    }

    const std::vector<Eigen::Vector2d> astar_fallback_path = path_;
    const int opt_ret = path_opti_->startOpt(path_);
    if (opt_ret < 0) {
        ROS_WARN_THROTTLE(1.0, "Path optimizer failed; keeping A* fallback path.");
        path_ = astar_fallback_path;
    }
    path_ = aster_->refinePath(path_);

    to_vis_->showAstarLists(path_);

    if(fixed_id == leader_id) {
        double perception_range = 5.5;
        findPointInPath(perception_range, leader_path_point);
        geometry_msgs::Point tmp_pt;
        tmp_pt.x = leader_path_point.x();
        tmp_pt.y = leader_path_point.y();
        tmp_pt.z = 0.0;
        path_point_pub.publish(tmp_pt);
    }
}

void PlanManager::widthHandleInExtend() {

    if(formation_type == formation_max_num)
        return;

    plan_manager::widthInfo temp_width;
    temp_width.id = fixed_id;
    temp_width.width = find_region_->getWidth();
    width_pub.publish(temp_width);
}

bool PlanManager::widthHandle(const Eigen::Vector2d& start_pt, Eigen::Vector2d& end_pt) {

        find_region_->updateFormationGap(formation_width+formation_interval);
        find_region_->setEndpointandupdateBoundary(start_pt, end_pt);
        find_region_->updatePointsRegion();
        find_region_->solve_max_b();

        // [Debug]: show width box and lines
        to_vis_->showUpdateArea(find_region_->getBox());
        std::vector<Eigen::Vector2d> one_line, other_line;
        find_region_->test(one_line, other_line);
        to_vis_->showWidthLineOne(one_line);
        to_vis_->showWidthLineOther(other_line);

        if(find_region_->getWidth() < 0.1) {
            return false;
        }

        recenterLocalGoalInCorridor(end_pt);

        // [Debug]:show width local point
        to_vis_->showWidthMidPoint(end_pt);
        return true;
}

void PlanManager::recenterLocalGoalInCorridor(Eigen::Vector2d& local_goal) {
    if (!center_local_goal_in_corridor) {
        return;
    }

    const double free_width = find_region_->getWidth();
    if (!std::isfinite(free_width) || free_width > corridor_centering_max_width) {
        return;
    }

    Eigen::Vector2d centered_goal = find_region_->getWidthBestPoint();
    if (!std::isfinite(centered_goal.x()) || !std::isfinite(centered_goal.y())) {
        return;
    }

    Eigen::Vector2d shift = centered_goal - local_goal;
    const double shift_norm = shift.norm();
    if (!std::isfinite(shift_norm) || shift_norm < 1e-3) {
        return;
    }

    const double max_shift = std::max(0.0, corridor_centering_max_shift);
    if (shift_norm > max_shift && max_shift > 1e-3) {
        shift = shift / shift_norm * max_shift;
    }

    local_goal += shift;
    ROS_WARN_THROTTLE(1.0,
                      "Corridor centering adjusted local goal for tb %d by %.2f m.",
                      fixed_id,
                      shift.norm());
}

// if feasiable width less than formation width, to reset desired formation shape
void PlanManager::isReduceFRM() {

    const double free_width = find_region_->getWidth();
    if (free_width < formation_width) {
        int cur_type = std::floor(free_width / formation_interval) + 1;
        cur_type = std::max(formation_min_type, std::min(cur_type, formation_type - 1));
        std::cout << "[formation reconfiguration] current shape:" << cur_type<<std::endl;

        if (cur_type == formation_type) {
            conditionHeld(width_reduce_since_, false, formation_reduce_hold_time);
            return;
        }

        if (!conditionHeld(width_reduce_since_, true, formation_reduce_hold_time)) {
            ROS_WARN_THROTTLE(2.0,
                              "narrow passage width %.2f detected, waiting for stable signal before reducing formation",
                              free_width);
            return;
        }

        formation_type = cur_type;
        setDesiredShape(formation_type);

        do_allocation = true;
        markFormationReduced();
    } else {
        conditionHeld(width_reduce_since_, false, formation_reduce_hold_time);
    }
}

void PlanManager::isExtendFRM() {

    int avoid_sum = std::accumulate(is_avoid_obs.begin(), is_avoid_obs.end(), 0);
    if (avoid_sum > 0) {
        conditionHeld(expand_clear_since_, false, formation_expand_hold_time);
        ROS_WARN_THROTTLE(2.0, "suppress formation expansion while %d robot(s) report near-obstacle pressure", avoid_sum);
        return;
    }

    const double since_reduce = last_formation_reduce_time_.isZero()
                                    ? std::numeric_limits<double>::infinity()
                                    : (ros::Time::now() - last_formation_reduce_time_).toSec();
    if (since_reduce < formation_expansion_cooldown) {
        conditionHeld(expand_clear_since_, false, formation_expand_hold_time);
        ROS_WARN_THROTTLE(2.0,
                          "suppress formation expansion for %.1fs after reduction",
                          formation_expansion_cooldown - since_reduce);
        return;
    }

    // consider formation extend when the convergence of formation error less than threshold
    double thresh=0.4;
    double extend_e;
    FRMBias(extend_e);
    // ROS_WARN("current FrmBias:%f", extend_e);
    if (extend_e > thresh) {
        conditionHeld(expand_clear_since_, false, formation_expand_hold_time);
        return;
    }

    // method 1
    // consensus formation_width of all robots
    double min = formation_interval * (formation_max_num - 1);
    for (int i = 0; i < fixed_num; i++)
        if (all_width[i] < min)
            min = all_width[i];

    // method 2
    // std::vector<double> vec = all_width;
    // double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    // double min = sum / vec.size();

    // method 3
    // double min = all_width[fixed_id];

    // ROS_WARN("min width:%f",min);
    int new_type = std::max(formation_min_type, static_cast<int>(std::floor(min / formation_interval) + 1));

    // new_formation greater than current_formation
    if (new_type > formation_type) {
        if (!conditionHeld(expand_clear_since_, true, formation_expand_hold_time)) {
            ROS_WARN_THROTTLE(2.0,
                              "clear width supports formation type %d, waiting for stable signal before expansion",
                              new_type);
            return;
        }

        ROS_WARN("formation expansion. type: %d", new_type);
        formation_type = new_type;
        setDesiredShape(formation_type);
        do_allocation = true;
        conditionHeld(expand_clear_since_, false, formation_expand_hold_time);
    } else {
        conditionHeld(expand_clear_since_, false, formation_expand_hold_time);
    }
}

void PlanManager::rotateAndShareDeform() {

    if (first_start) {
        do_allocation= true;
        first_start= false;
    }

    swarm_alloc = swarm_des;
    if (do_allocation) {
        do_allocation = false;
        applyTaskAllocation(true, "deform event");
    } else {
        applyTaskAllocation(false, "deform online");
    }

    // update shape info
    plan_manager::RotationVec temp_rotation;
    temp_rotation.shape_type = formation_type;
    temp_rotation.remap_id.resize(fixed_num);
    for (int i = 0; i < fixed_num; i++)
        temp_rotation.remap_id[i] = realloc_[i];
    temp_rotation.yaw = 0.0;
    frm_info_pub.publish(temp_rotation);
}

// rotate desired formation based on orientation, and share Frm infomation to other robots
void PlanManager::rotateAndShareFRM() {

    // compute the diff angle between formation and local goal
    Eigen::Vector2d tb_center_(0.0, 0.0);
    for (const auto& i : tb_states_)
        tb_center_ += i.pos;
    tb_center_ /= fixed_num;
    auto diff_ = local_goal - tb_center_;
    double yaw = atan2(diff_.y(), diff_.x());
    yaw = snapFormationYawForCorridor(yaw);
    double yaw_e = normalizeFormationYaw(yaw - formation_yaw);
    bool force_allocation = do_allocation;
    bool yaw_allocation_request = false;

    if (first_start) {
        force_allocation = true;
        first_start = false;
    }

    if (abs(yaw_e) > M_PI / 2.5) {
        ROS_WARN("rotation angle is %.2f, task allocation", yaw_e * 180);
        yaw_allocation_request = true;
    }

    formation_yaw = yaw;
    Eigen::Matrix2d r_mat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix().block<2, 2>(0, 0);
    for (int i = 0; i < fixed_num; i++)
        swarm_des[i] = r_mat * swarm_des_origin[i];

    swarm_alloc = swarm_des;
    if (force_allocation)
    {
        do_allocation = false;
        applyTaskAllocation(true, "formation event");
    }
    else if (yaw_allocation_request)
    {
        applyTaskAllocation(false, "yaw improvement");
    }
    else
    {
        applyTaskAllocation(false, "online improvement");
    }
    tb_global_goal = getApproachGoal();

    // update shape info
    plan_manager::RotationVec temp_rotation;
    temp_rotation.shape_type = formation_type;
    temp_rotation.remap_id.resize(fixed_num);
    for (int i = 0; i < fixed_num; i++)
        temp_rotation.remap_id[i] = realloc_[i];
    temp_rotation.rotation_vec[0] = r_mat(0, 0);
    temp_rotation.rotation_vec[1] = r_mat(0, 1);
    temp_rotation.rotation_vec[2] = r_mat(1, 0);
    temp_rotation.rotation_vec[3] = r_mat(1, 1);
    temp_rotation.yaw = formation_yaw;
    frm_info_pub.publish(temp_rotation);
}

// other robot update FRM info
void PlanManager::updateFRMInfo(const plan_manager::RotationVecConstPtr &msg) {
    if (leader_id == fixed_id)
        return;

    for (int i = 0; i < fixed_num; i++)
        realloc_[i] = msg->remap_id[i];
    tb_id = realloc_[fixed_id];
    first_start = false;
    do_allocation = false;

    if (task_type == TASK_TYPE::DEFORM)
        return;

    Eigen::Matrix2d rotation_2d;
    rotation_2d(0, 0) = msg->rotation_vec[0];
    rotation_2d(0, 1) = msg->rotation_vec[1];
    rotation_2d(1, 0) = msg->rotation_vec[2];
    rotation_2d(1, 1) = msg->rotation_vec[3];
    formation_yaw = msg->yaw;

    if (formation_type != msg->shape_type) {
        formation_type = msg->shape_type;
        setDesiredShape(formation_type);
    }

    for (int i = 0; i < fixed_num; i++)
        swarm_des[i] = rotation_2d * swarm_des_origin[i];

    tb_global_goal = getApproachGoal();
}

void PlanManager::applyFormationSeparation(Eigen::Vector2d& pos) const {
    const auto is_finite_vec = [](const Eigen::Vector2d& value) {
        return std::isfinite(value.x()) && std::isfinite(value.y());
    };

    const double clearance = std::max(0.0, formation_min_robot_clearance);
    const double gain = std::max(0.0, formation_min_robot_clearance_gain);
    if (clearance <= 1e-3 || gain <= 1e-6 || tb_id < 0 || tb_id >= fixed_num) {
        return;
    }

    const Eigen::Vector2d current_pos = tb_states_[tb_id].pos;
    if (!is_finite_vec(current_pos)) {
        return;
    }

    Eigen::Vector2d separation(0.0, 0.0);
    for (int i = 0; i < fixed_num; i++) {
        if (i == tb_id || !is_finite_vec(tb_states_[i].pos)) {
            continue;
        }

        Eigen::Vector2d away = current_pos - tb_states_[i].pos;
        double distance = away.norm();
        if (!std::isfinite(distance) || distance >= clearance) {
            continue;
        }

        if (distance < 1e-4) {
            away = swarm_des[tb_id] - swarm_des[i];
            distance = away.norm();
        }
        if (!is_finite_vec(away) || distance < 1e-4) {
            away = tb_states_[tb_id].orient_vec;
            distance = away.norm();
        }
        if (!is_finite_vec(away) || distance < 1e-4) {
            away = Eigen::Vector2d(1.0, 0.0);
            distance = 1.0;
        }

        separation += away.normalized() * (clearance - std::max(0.0, distance));
    }

    const double separation_norm = separation.norm();
    if (!std::isfinite(separation_norm) || separation_norm < 1e-4) {
        return;
    }

    const double max_shift = clearance;
    if (separation_norm > max_shift && max_shift > 1e-3) {
        separation = separation / separation_norm * max_shift;
    }

    pos += gain * separation;
    ROS_WARN_THROTTLE(1.0,
                      "Formation separation adjusted local goal for tb %d by %.2f m.",
                      fixed_id,
                      (gain * separation).norm());
}

void PlanManager::preserveLocalGoalProgress(Eigen::Vector2d& pos, const Eigen::Vector2d& local_path_goal) const {
    const auto is_finite_vec = [](const Eigen::Vector2d& value) {
        return std::isfinite(value.x()) && std::isfinite(value.y());
    };

    const double min_progress = std::max(0.0, formation_min_local_goal_progress);
    if (min_progress <= 1e-3 || tb_id < 0 || tb_id >= fixed_num) {
        return;
    }

    const Eigen::Vector2d current_pos = tb_states_[tb_id].pos;
    if (!is_finite_vec(current_pos) || !is_finite_vec(pos) || !is_finite_vec(local_path_goal)) {
        return;
    }

    const double final_dist = (tb_global_goal - current_pos).norm();
    if (!std::isfinite(final_dist) || final_dist <= std::max(approach_goal_dist, min_progress + 0.20)) {
        return;
    }

    Eigen::Vector2d path_delta = local_path_goal - current_pos;
    const double path_dist = path_delta.norm();
    if (!std::isfinite(path_dist) || path_dist <= min_progress) {
        return;
    }

    Eigen::Vector2d consensus_delta = pos - current_pos;
    const double consensus_dist = consensus_delta.norm();
    if (std::isfinite(consensus_dist) && consensus_dist >= min_progress) {
        return;
    }

    const Eigen::Vector2d path_dir = path_delta / path_dist;
    Eigen::Vector2d lateral(0.0, 0.0);
    if (is_finite_vec(consensus_delta)) {
        lateral = consensus_delta - path_dir * consensus_delta.dot(path_dir);
        const double lateral_norm = lateral.norm();
        if (std::isfinite(lateral_norm) && lateral_norm > min_progress) {
            lateral = lateral / lateral_norm * min_progress;
        }
    }

    pos = current_pos + path_dir * min_progress + lateral;
    ROS_WARN_THROTTLE(1.0,
                      "Consensus local goal for tb %d collapsed to %.2f m while %.2f m from slot; preserving %.2f m path progress.",
                      fixed_id,
                      std::isfinite(consensus_dist) ? consensus_dist : 0.0,
                      final_dist,
                      min_progress);
}

void PlanManager::FRMConsensus(Eigen::Vector2d& pos) {
    const auto is_finite_vec = [](const Eigen::Vector2d& value) {
        return std::isfinite(value.x()) && std::isfinite(value.y());
    };

    const Eigen::Vector2d local_path_goal = pos;
    Eigen::Vector2d vec(0.0, 0.0);
    int neighbor_count = 0;

    for (int i = 0; i < fixed_num; i++) {
        if (i == tb_id) {
            continue;
        }
        if (!is_finite_vec(tb_states_[i].pos) || !is_finite_vec(swarm_des[tb_id]) || !is_finite_vec(swarm_des[i])) {
            continue;
        }
        vec += tb_states_[i].pos + (swarm_des[tb_id] - swarm_des[i]);
        neighbor_count++;
    }

    if (neighbor_count <= 0) {
        return;
    }

    double local_goal_weight = std::max(0.1, formation_consensus_local_goal_weight);

    if (is_finite_vec(pos)) {
        vec += local_goal_weight * pos;
        vec /= (neighbor_count + local_goal_weight);
    } else {
        vec /= neighbor_count;
    }
    pos = vec;
    applyFormationSeparation(pos);
    preserveLocalGoalProgress(pos, local_path_goal);
}

// it is find Point from static path
void PlanManager::findPointInPath(double forward_distance, Eigen::Vector2d &pt) {
    if (path_.empty()) {
        pt = sanitizeControllerGoal(tb_global_goal);
        return;
    }

    double to_goal_dist = (global_goal - tb_states_[tb_id].pos).norm();
    if (to_goal_dist<forward_distance+0.2) {
        pt = global_goal;
        return;
    }

    //find Closest Point On Path
    std::vector<Eigen::Vector2d> cur_path = path_;
    Eigen::Vector2d cur_pos = tb_states_[tb_id].pos;

    auto closestPointOnSegment = [](const Eigen::Vector2d& p, const Eigen::Vector2d& segStart, const Eigen::Vector2d& segEnd) -> Eigen::Vector2d {
        Eigen::Vector2d segVec = segEnd - segStart;
        Eigen::Vector2d vecToPoint = p - segStart;

        double segLengthSq = segVec.squaredNorm();
        double t = 0.0;

        if (segLengthSq > 1e-8) {
            t = std::max(0.0, std::min(1.0,vecToPoint.dot(segVec) / segLengthSq));
        }

        return segStart + t * segVec;
    };

    Eigen::Vector2d closestPoint = cur_path[0];
    int curPointNextIdx = 0;

    double minDistSq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < cur_path.size() - 1; ++i) {
        Eigen::Vector2d candidate = closestPointOnSegment(cur_pos, cur_path[i], cur_path[i + 1]);
        double distSq = (cur_pos - candidate).squaredNorm();
        if (distSq < minDistSq) {
            minDistSq = distSq;
            closestPoint = candidate;
            curPointNextIdx = i + 1;
        }
    }

    double distToClosestPoint = (cur_pos - closestPoint).norm();

    double cur_forward_distance;

    cur_forward_distance = forward_distance;

    // Bisection method
    auto findDesiredPointOnSeg = [](const double& dist_delta, const Eigen::Vector2d& start, const Eigen::Vector2d& end) -> Eigen::Vector2d {
        Eigen::Vector2d left_pt = start;
        Eigen::Vector2d right_pt = end;
        double mid_dist;
        Eigen::Vector2d mid_pt;

        do {
            mid_pt = (left_pt + right_pt) / 2;
            mid_dist = (mid_pt -start).norm();
            if(mid_dist > dist_delta)
                right_pt = mid_pt;
            else
                left_pt = mid_pt;

        } while (abs(mid_dist - dist_delta) > 0.1);
        return mid_pt;
    };

    double distToNextPoint = (closestPoint - cur_path[curPointNextIdx]).norm();
    if (distToNextPoint > cur_forward_distance) {
        pt = findDesiredPointOnSeg(cur_forward_distance, closestPoint, cur_path[curPointNextIdx]);
        return ;
    }

    double dist_sum = distToNextPoint;
    for (int i = curPointNextIdx; i < cur_path.size() - 1; i++) {
        dist_sum += (cur_path[i + 1] - cur_path[i]).norm();
        if (cur_forward_distance < dist_sum) {
            double remaining_dist = cur_forward_distance - (dist_sum - (cur_path[i + 1] - cur_path[i]).norm());
            pt = findDesiredPointOnSeg(remaining_dist, cur_path[i], cur_path[i + 1]);
            return;
        }
    }

    ROS_WARN("path less than present distance!!! return the last point in path");
    pt = cur_path.back();
}

void PlanManager::setPathPoint(const geometry_msgs::PointConstPtr &msg) {
    if (fixed_id == leader_id)
        return;

    leader_path_point << msg->x,msg->y;
}

bool PlanManager::computeTaskAllocation(std::vector<int>& next_realloc, double& current_cost, double& proposed_cost) {
    std::vector<geometry_msgs::Point> start_pts,goal_pts;
    start_pts.resize(fixed_num);
    goal_pts.resize(fixed_num);

    Eigen::Vector2d start_center(0.0, 0.0);
    Eigen::Vector2d goal_center(0.0, 0.0);
    for (int i = 0; i < fixed_num; i++) {
        start_center += tb_states_[i].pos;
        goal_center += swarm_alloc[i];
    }
    start_center /= fixed_num;
    goal_center /= fixed_num;

    for (int i = 0; i < fixed_num; i++) {
        const Eigen::Vector2d rel_start = tb_states_[i].pos - start_center;
        const Eigen::Vector2d rel_goal = swarm_alloc[i] - goal_center;
        start_pts[i].x = rel_start.x();
        start_pts[i].y = rel_start.y();
        goal_pts[i].x = rel_goal.x();
        goal_pts[i].y = rel_goal.y();
    }

    std::vector<int> result;
    if (ot_cli_->compute_assignment(start_pts, goal_pts, result) != 0) {
        return false;
    }
    if (result.size() != fixed_num) {
        ROS_WARN("task allocation returned %zu assignments for %d robots", result.size(), fixed_num);
        return false;
    }

    std::vector<int> seen(fixed_num, 0);
    for (int i = 0; i < fixed_num; i++) {
        if (result[i] < 0 || result[i] >= fixed_num) {
            ROS_WARN("task allocation returned invalid slot id %d for logical robot %d", result[i], i);
            return false;
        }
        seen[result[i]] += 1;
    }
    for (int i = 0; i < fixed_num; i++) {
        if (seen[i] != 1) {
            ROS_WARN("task allocation returned a non-permutation assignment");
            return false;
        }
    }

    current_cost = 0.0;
    proposed_cost = 0.0;
    for (int i = 0; i < fixed_num; i++) {
        const Eigen::Vector2d rel_start(start_pts[i].x, start_pts[i].y);
        const Eigen::Vector2d current_goal(goal_pts[i].x, goal_pts[i].y);
        const Eigen::Vector2d proposed_goal(goal_pts[result[i]].x, goal_pts[result[i]].y);
        current_cost += (rel_start - current_goal).squaredNorm();
        proposed_cost += (rel_start - proposed_goal).squaredNorm();
    }

    next_realloc.resize(fixed_num);
    for (int i = 0; i < fixed_num; i++)
        next_realloc[i] = result[realloc_[i]];

    return true;
}

bool PlanManager::applyTaskAllocation(bool force, const char* reason) {
    const ros::Time now = ros::Time::now();
    if (!force) {
        if (!slot_reassignment_enabled) {
            return false;
        }
        if (!last_slot_reassignment_check_time_.isZero() &&
            (now - last_slot_reassignment_check_time_).toSec() < slot_reassignment_cooldown) {
            return false;
        }
        last_slot_reassignment_check_time_ = now;
        if (!last_slot_reassignment_apply_time_.isZero() &&
            (now - last_slot_reassignment_apply_time_).toSec() < slot_reassignment_cooldown) {
            return false;
        }
    }

    std::vector<int> next_realloc;
    double current_cost = 0.0;
    double proposed_cost = 0.0;
    if (!computeTaskAllocation(next_realloc, current_cost, proposed_cost)) {
        return false;
    }

    const bool same_assignment = next_realloc == realloc_;
    const double improvement = current_cost - proposed_cost;
    const double min_improvement = std::max(
        std::max(0.0, slot_reassignment_min_cost_improvement),
        std::max(0.0, slot_reassignment_min_cost_improvement_ratio) * std::max(0.0, current_cost));

    if (!force && (same_assignment || improvement < min_improvement)) {
        return false;
    }

    realloc_ = next_realloc;
    tb_id = realloc_[fixed_id];
    reported_reached_current_slot_ = false;
    last_reassignment_time_ = now;
    last_slot_reassignment_apply_time_ = now;

    //print task allocation info
    std::cout << "[task allocation:" << reason << "] ";
    for (int i = 0; i<fixed_num; i++) {
        std::cout << i << "-->" << realloc_[i] << "\t";
    }
    std::cout << "cost " << current_cost << " -> " << proposed_cost
              << " improvement " << improvement;

    std::cout<<"\n";
    return true;
}

// task assignment and show assignment result
void PlanManager::taskAllocation() {
    applyTaskAllocation(true, "legacy call");
}

// compute formation bias
void PlanManager::FRMBias(double& e) {

    std::vector<Eigen::Vector2d> vec_des;
    vec_des.resize(fixed_num, Eigen::Vector2d(0.0, 0.0));

    for (int id = 0; id < fixed_num; id++) {
        for(int i = 0; i < fixed_num; i++) {
            if(i == id)
                continue;
            vec_des[id] += tb_states_[i].pos + (swarm_des[id] - swarm_des[i]);
        }
        vec_des[id] /= (fixed_num - 1);
    }

    e = 0.0;
    for (int i = 0; i < fixed_num; i++)
        e += (vec_des[i] - tb_states_[i].pos).norm();
}

// print change State
void PlanManager::changeFRMExecState(PlanManager::FORMATION_EXEC_STATE new_state) {

    static string state_str[7] = {"WAIT_ODOM_OR_TARGET", "FRM_INIT_PATH", "FRM_KEEP", "FRM_RECONFIG", "AVOID_OBS", "APPROACH_GOAL", "EMERGENCY_STOP"};
    int last_state = int(exec_state_);
    exec_state_ = new_state;
    std::cout << "tb " + std::to_string(fixed_id) + " from " + state_str[last_state] + " to " + state_str[int(new_state)] << std::endl;
}

// subscribe latest global goal
void PlanManager::subGlobalGoal(const geometry_msgs::PoseStampedConstPtr &msg) {
    global_goal.x() = msg->pose.position.x;
    global_goal.y() = msg->pose.position.y;

    leader_path_point << msg->pose.position.x, msg->pose.position.y;

    tb_global_goal = getApproachGoal();
    traj_opt_->setGlobalGoal(tb_global_goal);
    has_goal = true;
    first_start = true;
    reported_reached_current_slot_ = false;
    reported_reached_slot_ = tb_global_goal;

    std_msgs::Bool reached_msg;
    reached_msg.data = false;
    deform_reached_pub_.publish(reached_msg);

    if (has_odom) {
        changeFRMExecState(FRM_INIT_PATH);
    } else {
        changeFRMExecState(WAIT_ODOM_OR_TARGET);
    }
}

std::vector<Eigen::Vector2d> PlanManager::generateFormationPositions(int robot_num, int one_row_num, double frm_gap) {

    if(robot_num < one_row_num) {
        std::cout << "invaild number" << std::endl;
        return std::vector<Eigen::Vector2d>(1, Eigen::Vector2d(-1, -1));
    }

    Eigen::Vector2d tmp_pt;

    std::vector<Eigen::Vector2d> frm_pos;
    Eigen::Vector2d leader_pos;
    int lid;

    int rows = robot_num / one_row_num;
    int remaining = robot_num % one_row_num;

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < one_row_num; j++) {
            frm_pos.emplace_back(i * frm_gap, j * frm_gap);
        }
    }

    if (remaining > 0) {
        double base_x = rows * frm_gap;
        double center_y = (one_row_num - 1) * frm_gap / 2;

        if (remaining % 2 == 1) {
            leader_pos << base_x, center_y;
            lid = frm_pos.size();
            frm_pos.push_back(leader_pos);
            remaining -= 1;
            for (int i = 1; i <= remaining / 2; i++) {
                frm_pos.emplace_back(base_x, leader_pos.y() + i * frm_gap);
                frm_pos.emplace_back(base_x, leader_pos.y() - i * frm_gap);
            }
        } else {
            leader_pos << base_x, center_y + frm_gap / 2;
            lid = frm_pos.size();
            frm_pos.push_back(leader_pos);
            Eigen::Vector2d right_pos(base_x, center_y - frm_gap / 2);
            frm_pos.push_back(right_pos);
            remaining -= 2;
            for (int i = 1; i <= remaining / 2; i++) {
                frm_pos.emplace_back(leader_pos.x(), leader_pos.y() + i * frm_gap);
                frm_pos.emplace_back(right_pos.x(), right_pos.y() - i * frm_gap);
            }
        }
    } else {
        leader_pos << (rows - 1) * frm_gap, (one_row_num / 2) * frm_gap;
        lid = (rows - 1) * one_row_num + (one_row_num / 2);
    }

    // offset frm_pos based on leader_pos
    for (int i = 0; i < frm_pos.size(); i++) {
        frm_pos[i] -= leader_pos;
    }
    std::swap(frm_pos[0], frm_pos[lid]);

    // The response navigation goals represent the formation centroid. Keeping
    // legacy leader-centered slots pushes every follower to one side of a
    // corridor and can complete while the group center is still offset.
    Eigen::Vector2d centroid(0.0, 0.0);
    for (const auto& pos : frm_pos) {
        centroid += pos;
    }
    centroid /= static_cast<double>(frm_pos.size());
    for (int i = 0; i < frm_pos.size(); i++) {
        frm_pos[i] -= centroid;
    }

    // show formation info
    bool show_frm_pos = false;
    if (show_frm_pos) {
        std::cout << "robot_num:" << robot_num << " one_row_num:" << one_row_num << std::endl;;
        std::cout << "formation position:\n";
        for (int i = 0; i < frm_pos.size(); i++) {
            std::cout << i << "\t" << frm_pos[i].x() << "," << frm_pos[i].y() << (i == lid ? "* \n" : " \n");
        }
    }

    return frm_pos;
}

void PlanManager::calcDesireAngularVel(double &w, const Eigen::Vector2d& goal) {
    Eigen::Vector2d diff_vec = goal - tb_states_[tb_id].pos;
    const double to_goal_dist = diff_vec.norm();
    if (!std::isfinite(diff_vec.x()) || !std::isfinite(diff_vec.y()) || to_goal_dist < 1e-3) {
        w = 0.0;
        return;
    }

    Eigen::Vector2d to_goal_orient = diff_vec / to_goal_dist;

    //w \in [-1.5,-1.5]
    w = 0.5 * (1 - tb_states_[tb_id].orient_vec.dot(to_goal_orient)) * 1.5;

    double angular_diff = atan2(diff_vec.y(), diff_vec.x()) - tb_states_[tb_id].yaw;

    if (angular_diff > M_PI)
        angular_diff -= 2 * M_PI;
    if (angular_diff < -M_PI)
        angular_diff += 2 * M_PI;

    int sign_ = angular_diff > 0 ? 1 : -1;

    w = sign_ * w;
}

void PlanManager::calcDesireLinearVel(double &v, const Eigen::Vector2d& goal) {
    double x_e_threah = 0.5;
    Eigen::Vector2d diff_vec = goal - tb_states_[tb_id].pos;
    if (!std::isfinite(diff_vec.x()) || !std::isfinite(diff_vec.y())) {
        v = 0.0;
        return;
    }
    double x_e = diff_vec.norm();

    if(x_e > x_e_threah)
        v = 0.2;
    else
        v = x_e / x_e_threah * 0.2;
}

void PlanManager::pubVelCmd(const std::vector<double> &c) {
    if (c.size() < 2 || !std::isfinite(c[0]) || !std::isfinite(c[1])) {
        ROS_WARN_THROTTLE(1.0, "Reject invalid velocity command for tb %d; publishing zero.", fixed_id);
        geometry_msgs::Twist vel;
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
        vel_pub.publish(vel);
        return;
    }

    geometry_msgs::Twist vel;
    vel.linear.x = c[0];
    vel.angular.z = c[1];
    vel_pub.publish(vel);
}

void PlanManager::publishDebugInfo(const Eigen::Vector2d& controller_goal,
                                   double v_des,
                                   double w_des,
                                   bool reach_goal,
                                   const std::vector<double>& cmd) {
    if (debug_pub_.getNumSubscribers() <= 0) {
        return;
    }

    std::vector<double> other_distances;
    other_distances.reserve(std::max(0, fixed_num - 1));
    for (int i = 0; i < fixed_num; i++) {
        if (i == tb_id) {
            continue;
        }
        other_distances.push_back((tb_states_[i].pos - tb_states_[tb_id].pos).norm());
    }
    std::sort(other_distances.begin(), other_distances.end());

    std::ostringstream oss;
    static const char* state_str[7] = {
        "WAIT_ODOM_OR_TARGET",
        "FRM_INIT_PATH",
        "FRM_KEEP",
        "FRM_RECONFIG",
        "AVOID_OBS",
        "APPROACH_GOAL",
        "EMERGENCY_STOP"
    };
    const int state_index = static_cast<int>(exec_state_);
    const double controller_dist = (controller_goal - tb_states_[tb_id].pos).norm();
    const double tb_goal_dist = (tb_global_goal - tb_states_[tb_id].pos).norm();
    const double center_goal_dist = (global_goal - tb_states_[tb_id].pos).norm();
    const double esdf_dist = grid_map_->getDistance(tb_states_[tb_id].pos);

    oss << "fixed_id=" << fixed_id
        << " tb_id=" << tb_id
        << " state=" << (state_index >= 0 && state_index < 7 ? state_str[state_index] : "UNKNOWN")
        << " formation_type=" << formation_type
        << " has_goal=" << has_goal
        << " has_odom=" << has_odom
        << " has_depth=" << grid_map_->hasDepthObservation()
        << " pos=(" << tb_states_[tb_id].pos.x() << "," << tb_states_[tb_id].pos.y() << ")"
        << " yaw=" << tb_states_[tb_id].yaw
        << " global_goal=(" << global_goal.x() << "," << global_goal.y() << ")"
        << " tb_global_goal=(" << tb_global_goal.x() << "," << tb_global_goal.y() << ")"
        << " controller_goal=(" << controller_goal.x() << "," << controller_goal.y() << ")"
        << " center_goal_dist=" << center_goal_dist
        << " tb_goal_dist=" << tb_goal_dist
        << " controller_dist=" << controller_dist
        << " esdf_dist=" << esdf_dist
        << " v_des=" << v_des
        << " w_des=" << w_des
        << " cmd=(" << (cmd.size() > 0 ? cmd[0] : 0.0) << "," << (cmd.size() > 1 ? cmd[1] : 0.0) << ")"
        << " reach_goal=" << reach_goal
        << " realloc=[";
    for (int i = 0; i < realloc_.size(); i++) {
        if (i > 0) {
            oss << ",";
        }
        oss << realloc_[i];
    }
    oss << "] other_dist=[";
    for (int i = 0; i < other_distances.size(); i++) {
        if (i > 0) {
            oss << ",";
        }
        oss << other_distances[i];
    }
    oss << "] avoid=[";
    for (int i = 0; i < is_avoid_obs.size(); i++) {
        if (i > 0) {
            oss << ",";
        }
        oss << is_avoid_obs[i];
    }
    oss << "]";

    std_msgs::String msg;
    msg.data = oss.str();
    debug_pub_.publish(msg);
}

void PlanManager::readParameters() {
    nh_pri_.param("plan_manager/tb_id", fixed_id, -1);
    nh_pri_.param("plan_manager/tb_num", fixed_num, -1);
    nh_pri_.param("plan_manager/other_num_in_opti", other_num_to_opti, -1);
    nh_pri_.param("plan_manager/formation_interval", formation_interval, -1.0);
    nh_pri_.param("plan_manager/leader_id", leader_id, -1);
    nh_pri_.param("plan_manager/forward_distance", fwd_dist, -1.0);
    nh_pri_.param("plan_manager/obs_clearance", obs_clearance, -1.0);
    nh_pri_.param("plan_manager/approach_goal_distance", approach_goal_dist, 2.0);
    nh_pri_.param("plan_manager/approach_center_goal", approach_center_goal, false);
    nh_pri_.param("plan_manager/publish_esdf_avoid_state", publish_esdf_avoid_state, false);
    nh_pri_.param("plan_manager/center_local_goal_in_corridor", center_local_goal_in_corridor, true);
    nh_pri_.param("plan_manager/corridor_centering_max_width", corridor_centering_max_width, 2.4);
    nh_pri_.param("plan_manager/corridor_centering_max_shift", corridor_centering_max_shift, 0.45);
    nh_pri_.param("plan_manager/formation_corridor_yaw_snap_enabled", formation_corridor_yaw_snap_enabled, true);
    nh_pri_.param("plan_manager/formation_corridor_yaw_snap_max_width", formation_corridor_yaw_snap_max_width, 2.4);
    nh_pri_.param("plan_manager/formation_corridor_yaw_snap_tolerance", formation_corridor_yaw_snap_tolerance, 0.70);
    nh_pri_.param("plan_manager/formation_consensus_local_goal_weight", formation_consensus_local_goal_weight, 1.0);
    nh_pri_.param("plan_manager/formation_min_robot_clearance", formation_min_robot_clearance, 0.4);
    nh_pri_.param("plan_manager/formation_min_robot_clearance_gain", formation_min_robot_clearance_gain, 1.0);
    nh_pri_.param("plan_manager/formation_min_local_goal_progress", formation_min_local_goal_progress, 0.30);
    nh_pri_.param("plan_manager/formation_min_type", formation_min_type, 1);
    nh_pri_.param("plan_manager/formation_reduce_robot_fraction", formation_reduce_robot_fraction, 0.2);
    nh_pri_.param("plan_manager/avoid_obs_distance_threshold", avoid_obs_distance_threshold, 0.15);
    nh_pri_.param("plan_manager/formation_reduce_hold_time", formation_reduce_hold_time, 1.0);
    nh_pri_.param("plan_manager/formation_expand_hold_time", formation_expand_hold_time, 3.0);
    nh_pri_.param("plan_manager/formation_expansion_cooldown", formation_expansion_cooldown, 8.0);
    nh_pri_.param("plan_manager/reassignment_cooldown", reassignment_cooldown, 8.0);
    nh_pri_.param("plan_manager/slot_reassignment_enabled", slot_reassignment_enabled, true);
    nh_pri_.param("plan_manager/slot_reassignment_cooldown", slot_reassignment_cooldown, 2.0);
    nh_pri_.param("plan_manager/slot_reassignment_min_cost_improvement", slot_reassignment_min_cost_improvement, 0.08);
    nh_pri_.param("plan_manager/slot_reassignment_min_cost_improvement_ratio", slot_reassignment_min_cost_improvement_ratio, 0.10);
    nh_pri_.param("plan_manager/timeout_reassignment_enabled", timeout_reassignment_enabled, true);
    nh_pri_.param("plan_manager/frm_type", start_frm_type, -1);
    nh_pri_.param("plan_manager/max_frm_type", formation_max_num, fixed_num);
    nh_pri_.param("task_type", task_type, -1);
    nh_pri_.param("vehicle_number", vehicle_number, -1);
}
