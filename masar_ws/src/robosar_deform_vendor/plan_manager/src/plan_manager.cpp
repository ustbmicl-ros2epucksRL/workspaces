#include "plan_manager/plan_manager.h"

#include "plan_manager/formation_assignment.h"

#include <algorithm>
#include <array>
#include <boost/bind/bind.hpp>
#include <cmath>
#include <limits>
#include <queue>
#include <sstream>
#include <unordered_set>

namespace
{
constexpr double kPi = 3.14159265358979323846;
}

using boost::placeholders::_1;

PlanManager::PlanManager() : nh_(), nh_pri_("~")
{
    readParameters();
    resetFormationSlots();
    robot_states_.resize(static_cast<std::size_t>(fixed_num_));
    has_robot_state_.assign(static_cast<std::size_t>(fixed_num_), false);

    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &PlanManager::goalCallback, this);
    odom_sub_ = nh_.subscribe<nav_msgs::Odometry>(
        "/tb_" + std::to_string(fixed_id_) + "/odom",
        1,
        &PlanManager::odomCallback,
        this);
    teammate_odom_subs_.reserve(static_cast<std::size_t>(fixed_num_));
    for (int robot_id = 0; robot_id < fixed_num_; ++robot_id) {
        teammate_odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
            "/tb_" + std::to_string(robot_id) + "/odom",
            1,
            boost::bind(&PlanManager::teammateOdomCallback, this, _1, robot_id)));
    }
    map_sub_ = nh_.subscribe(map_topic_, 1, &PlanManager::mapCallback, this);
    if (use_dynamic_obstacle_models_) {
        model_states_sub_ = nh_.subscribe(
            model_states_topic_,
            1,
            &PlanManager::modelStatesCallback,
            this);
    }

    vel_pub_ = nh_pri_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    reached_pub_ = nh_pri_.advertise<std_msgs::Bool>(
        "/tb_" + std::to_string(fixed_id_) + "/deform_reached",
        1,
        true);
    debug_pub_ = nh_pri_.advertise<std_msgs::String>(
        "/tb_" + std::to_string(fixed_id_) + "/deform_debug",
        1);
    center_path_pub_ = nh_pri_.advertise<nav_msgs::Path>(
        "/response_group/center_path",
        1,
        true);

    const double rate = std::max(1.0, control_rate_hz_);
    control_timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &PlanManager::controlCallback, this);

    publishReached(false);
    ROS_WARN("tb %d using simplified fixed-square response formation controller", fixed_id_);
}

void PlanManager::readParameters()
{
    nh_pri_.param("plan_manager/tb_id", fixed_id_, 0);
    nh_pri_.param("plan_manager/tb_num", fixed_num_, 4);
    nh_pri_.param("plan_manager/formation_interval", formation_interval_, 0.8);
    nh_pri_.param("plan_manager/robot_radius", robot_radius_, 0.18);
    nh_pri_.param("plan_manager/formation_clearance", formation_clearance_, 0.18);
    nh_pri_.param("plan_manager/replanning_distance", replanning_distance_, 0.35);
    nh_pri_.param("plan_manager/path_goal_tolerance", path_goal_tolerance_, 0.35);
    nh_pri_.param("plan_manager/goal_tolerance", goal_tolerance_, 0.18);
    nh_pri_.param("plan_manager/lookahead_distance", lookahead_distance_, 0.6);
    nh_pri_.param("plan_manager/control_rate", control_rate_hz_, 10.0);
    nh_pri_.param("plan_manager/max_linear_velocity", max_linear_velocity_, 0.22);
    nh_pri_.param("plan_manager/max_angular_velocity", max_angular_velocity_, 1.4);
    nh_pri_.param("plan_manager/linear_gain", linear_gain_, 0.45);
    nh_pri_.param("plan_manager/angular_gain", angular_gain_, 1.8);
    nh_pri_.param("plan_manager/angular_slowdown_threshold", angular_slowdown_threshold_, 1.57079632679);
    nh_pri_.param("plan_manager/rotate_in_place_threshold", rotate_in_place_threshold_, 1.0471975512);
    nh_pri_.param("plan_manager/rotate_slots_by_goal_yaw", rotate_slots_by_goal_yaw_, false);
    nh_pri_.param("plan_manager/align_slots_to_path", align_slots_to_path_, false);
    nh_pri_.param("plan_manager/use_occupancy_grid_planner", use_occupancy_grid_planner_, true);
    nh_pri_.param("plan_manager/treat_unknown_as_occupied", treat_unknown_as_occupied_, false);
    nh_pri_.param("plan_manager/map_topic", map_topic_, std::string("/map"));
    nh_pri_.param("plan_manager/allow_formation_scaling", allow_formation_scaling_, true);
    nh_pri_.param("plan_manager/min_formation_scale", min_formation_scale_, 0.75);
    nh_pri_.param("plan_manager/formation_scale_step", formation_scale_step_, 0.10);
    nh_pri_.param("plan_manager/formation_expand_distance", formation_expand_distance_, 1.0);
    nh_pri_.param("plan_manager/blocked_recovery_linear_scale", blocked_recovery_linear_scale_, 0.6);
    nh_pri_.param("plan_manager/allow_blocked_direct_recovery", allow_blocked_direct_recovery_, false);
    nh_pri_.param("plan_manager/blocked_direct_recovery_max_distance", blocked_direct_recovery_max_distance_, 2.5);
    nh_pri_.param("plan_manager/use_dynamic_obstacle_models", use_dynamic_obstacle_models_, true);
    nh_pri_.param("plan_manager/model_states_topic", model_states_topic_, std::string("/gazebo/model_states"));
    nh_pri_.param("plan_manager/dynamic_obstacle_radius", dynamic_obstacle_radius_, 0.40);
    nh_pri_.param("plan_manager/dynamic_obstacle_replan_distance", dynamic_obstacle_replan_distance_, 0.20);
    nh_pri_.param(
        "plan_manager/dynamic_obstacle_model_prefixes",
        dynamic_obstacle_model_prefixes_,
        std::vector<std::string>{std::string("agent_")});
    nh_pri_.param(
        "plan_manager/dynamic_obstacle_model_names",
        dynamic_obstacle_model_names_,
        std::vector<std::string>());
    nh_pri_.param(
        "plan_manager/ignored_dynamic_obstacle_model_names",
        ignored_dynamic_obstacle_model_names_,
        std::vector<std::string>{
            std::string("tb_0"),
            std::string("tb_1"),
            std::string("tb_2"),
            std::string("tb_3"),
        });
    nh_pri_.param("plan_manager/occupied_threshold", occupied_threshold_, 65);

    fixed_num_ = std::max(1, fixed_num_);
    if (fixed_id_ < 0 || fixed_id_ >= fixed_num_) {
        ROS_WARN("plan_manager/tb_id %d is outside [0, %d); clamping to 0", fixed_id_, fixed_num_);
        fixed_id_ = 0;
    }
    formation_interval_ = std::max(0.05, formation_interval_);
    robot_radius_ = std::max(0.0, robot_radius_);
    formation_clearance_ = std::max(0.0, formation_clearance_);
    replanning_distance_ = std::max(0.05, replanning_distance_);
    path_goal_tolerance_ = std::max(0.05, path_goal_tolerance_);
    goal_tolerance_ = std::max(0.01, goal_tolerance_);
    lookahead_distance_ = std::max(0.05, lookahead_distance_);
    max_linear_velocity_ = std::max(0.0, max_linear_velocity_);
    max_angular_velocity_ = std::max(0.0, max_angular_velocity_);
    linear_gain_ = std::max(0.0, linear_gain_);
    angular_gain_ = std::max(0.0, angular_gain_);
    angular_slowdown_threshold_ = std::max(0.05, angular_slowdown_threshold_);
    rotate_in_place_threshold_ = std::max(0.0, rotate_in_place_threshold_);
    min_formation_scale_ = clamp(min_formation_scale_, 0.30, 1.0);
    formation_scale_step_ = clamp(formation_scale_step_, 0.02, 0.50);
    formation_expand_distance_ = std::max(0.10, formation_expand_distance_);
    blocked_recovery_linear_scale_ = clamp(blocked_recovery_linear_scale_, 0.10, 1.0);
    blocked_direct_recovery_max_distance_ = std::max(0.10, blocked_direct_recovery_max_distance_);
    dynamic_obstacle_radius_ = std::max(0.0, dynamic_obstacle_radius_);
    dynamic_obstacle_replan_distance_ = std::max(0.02, dynamic_obstacle_replan_distance_);
    occupied_threshold_ = std::min(std::max(occupied_threshold_, 1), 100);
}

void PlanManager::resetFormationSlots()
{
    slots_.clear();
    slots_.reserve(static_cast<std::size_t>(fixed_num_));

    if (fixed_num_ == 4) {
        const double half = 0.5 * formation_interval_;
        slots_.push_back(Eigen::Vector2d(half, half));
        slots_.push_back(Eigen::Vector2d(half, -half));
        slots_.push_back(Eigen::Vector2d(-half, -half));
        slots_.push_back(Eigen::Vector2d(-half, half));
    } else {
        const int columns = static_cast<int>(std::ceil(std::sqrt(static_cast<double>(fixed_num_))));
        const int rows = static_cast<int>(std::ceil(static_cast<double>(fixed_num_) / columns));
        const double x_origin = 0.5 * (columns - 1) * formation_interval_;
        const double y_origin = 0.5 * (rows - 1) * formation_interval_;

        for (int index = 0; index < fixed_num_; ++index) {
            const int row = index / columns;
            const int col = index % columns;
            slots_.push_back(Eigen::Vector2d(
                x_origin - col * formation_interval_,
                y_origin - row * formation_interval_));
        }
    }

    assigned_slot_indices_.clear();
    assigned_slot_indices_.reserve(static_cast<std::size_t>(fixed_num_));
    for (int index = 0; index < fixed_num_; ++index) {
        assigned_slot_indices_.push_back(index);
    }
    slot_assignment_ready_ = false;
}

void PlanManager::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    self_state_.pos.x() = msg->pose.pose.position.x;
    self_state_.pos.y() = msg->pose.pose.position.y;
    self_state_.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    has_odom_ = true;
    if (fixed_id_ >= 0 && fixed_id_ < static_cast<int>(robot_states_.size())) {
        robot_states_[static_cast<std::size_t>(fixed_id_)] = self_state_;
        has_robot_state_[static_cast<std::size_t>(fixed_id_)] = true;
    }
}

void PlanManager::teammateOdomCallback(const nav_msgs::OdometryConstPtr& msg, int robot_id)
{
    if (robot_id < 0 || robot_id >= fixed_num_) {
        return;
    }

    RobotState state;
    state.pos.x() = msg->pose.pose.position.x;
    state.pos.y() = msg->pose.pose.position.y;
    state.yaw = yawFromQuaternion(msg->pose.pose.orientation);
    robot_states_[static_cast<std::size_t>(robot_id)] = state;
    has_robot_state_[static_cast<std::size_t>(robot_id)] = true;
    if (robot_id == fixed_id_) {
        self_state_ = state;
        has_odom_ = true;
    }
}

void PlanManager::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    occupancy_grid_ = *msg;
    has_map_ = occupancy_grid_.info.width > 0 &&
               occupancy_grid_.info.height > 0 &&
               occupancy_grid_.info.resolution > 0.0 &&
               occupancy_grid_.data.size() ==
                   static_cast<std::size_t>(occupancy_grid_.info.width * occupancy_grid_.info.height);
    if (has_map_) {
        markPlanDirty();
    }
}

void PlanManager::modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg)
{
    std::vector<Eigen::Vector2d> positions;
    positions.reserve(msg->name.size());

    for (std::size_t i = 0; i < msg->name.size() && i < msg->pose.size(); ++i) {
        const std::string& name = msg->name[i];
        if (std::find(
                ignored_dynamic_obstacle_model_names_.begin(),
                ignored_dynamic_obstacle_model_names_.end(),
                name) != ignored_dynamic_obstacle_model_names_.end()) {
            continue;
        }

        const bool exact_match =
            std::find(
                dynamic_obstacle_model_names_.begin(),
                dynamic_obstacle_model_names_.end(),
                name) != dynamic_obstacle_model_names_.end();
        bool prefix_match = false;
        for (const std::string& prefix : dynamic_obstacle_model_prefixes_) {
            if (!prefix.empty() && startsWith(name, prefix)) {
                prefix_match = true;
                break;
            }
        }
        if (!exact_match && !prefix_match) {
            continue;
        }

        const geometry_msgs::Point& point = msg->pose[i].position;
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
            continue;
        }
        positions.emplace_back(point.x, point.y);
    }

    bool changed = positions.size() != dynamic_obstacle_positions_.size();
    if (!changed) {
        for (std::size_t i = 0; i < positions.size(); ++i) {
            if ((positions[i] - dynamic_obstacle_positions_[i]).norm() >
                dynamic_obstacle_replan_distance_) {
                changed = true;
                break;
            }
        }
    }

    dynamic_obstacle_positions_ = positions;
    if (changed && has_goal_) {
        markPlanDirty();
    }
}

void PlanManager::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    goal_center_.x() = msg->pose.position.x;
    goal_center_.y() = msg->pose.position.y;
    goal_yaw_ = yawFromQuaternion(msg->pose.orientation);
    has_goal_ = true;
    reached_ = false;
    markPlanDirty();
    publishReached(false);

    const Eigen::Vector2d target = slotTarget(goal_center_, rotate_slots_by_goal_yaw_ ? goal_yaw_ : 0.0, 1.0);
    ROS_WARN("tb %d accepted square slot target %.3f %.3f for center goal %.3f %.3f",
             fixed_id_,
             target.x(),
             target.y(),
             goal_center_.x(),
             goal_center_.y());
}

void PlanManager::controlCallback(const ros::TimerEvent&)
{
    if (!has_goal_ || !has_odom_) {
        publishZero();
        return;
    }

    if (!ensureSlotAssignment()) {
        publishZero();
        publishReached(false);
        return;
    }

    ensureCenterPath();
    if (path_blocked_) {
        const Eigen::Vector2d current_center = formationCenter();
        const Eigen::Vector2d center_delta = goal_center_ - current_center;
        const double center_distance = center_delta.norm();
        double recovery_yaw = rotate_slots_by_goal_yaw_ ? goal_yaw_ : 0.0;
        const double recovery_scale = commandedFormationScale(current_center);

        if (allow_blocked_direct_recovery_ &&
            std::isfinite(center_distance) &&
            center_distance > 1e-6 &&
            center_distance <= blocked_direct_recovery_max_distance_) {
            recovery_yaw = std::atan2(center_delta.y(), center_delta.x());
            const double step = std::min(lookahead_distance_, center_distance);
            const Eigen::Vector2d recovery_center =
                current_center + center_delta * (step / center_distance);
            const Eigen::Vector2d final_target = slotTarget(
                goal_center_,
                rotate_slots_by_goal_yaw_ ? goal_yaw_ : recovery_yaw,
                1.0);
            const Eigen::Vector2d control_target = slotTarget(
                recovery_center,
                recovery_yaw,
                recovery_scale);
            const double distance = (final_target - self_state_.pos).norm();
            if (std::isfinite(distance) && distance <= goal_tolerance_) {
                publishZero();
                if (!reached_) {
                    reached_ = true;
                    publishReached(true);
                    ROS_WARN("tb %d reached square slot target during blocked direct recovery", fixed_id_);
                }
                return;
            }
            geometry_msgs::Twist cmd = computeVelocityCommand(final_target, control_target);
            cmd.linear.x *= blocked_recovery_linear_scale_;
            vel_pub_.publish(cmd);
            publishDebug(final_target, control_target, cmd, distance, recovery_yaw);
            markPlanDirty();
            publishReached(false);
            return;
        }

        const Eigen::Vector2d recovery_target = slotTarget(current_center, recovery_yaw, recovery_scale);
        geometry_msgs::Twist cmd = computeVelocityCommand(recovery_target, recovery_target);
        cmd.linear.x *= blocked_recovery_linear_scale_;
        vel_pub_.publish(cmd);
        publishDebug(recovery_target, recovery_target, cmd, (recovery_target - self_state_.pos).norm(), recovery_yaw);
        markPlanDirty();
        publishReached(false);
        return;
    }

    double formation_yaw = rotate_slots_by_goal_yaw_ ? goal_yaw_ : 0.0;
    const double slot_scale = commandedFormationScale(formationCenter());
    const Eigen::Vector2d control_center = controlTargetFor(goal_center_, formation_yaw);
    const Eigen::Vector2d final_target = slotTarget(goal_center_, rotate_slots_by_goal_yaw_ ? goal_yaw_ : formation_yaw, 1.0);
    const Eigen::Vector2d target = slotTarget(control_center, formation_yaw, slot_scale);
    const double distance = (final_target - self_state_.pos).norm();
    if (!std::isfinite(distance)) {
        publishZero();
        publishReached(false);
        return;
    }

    if (distance <= goal_tolerance_) {
        publishZero();
        if (!reached_) {
            reached_ = true;
            publishReached(true);
            ROS_WARN("tb %d reached square slot target", fixed_id_);
        }
        return;
    }

    if (reached_) {
        reached_ = false;
        publishReached(false);
    }

    geometry_msgs::Twist cmd = computeVelocityCommand(final_target, target);
    vel_pub_.publish(cmd);
    publishDebug(final_target, target, cmd, distance, formation_yaw);
}

void PlanManager::publishZero()
{
    geometry_msgs::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    vel_pub_.publish(cmd);
}

void PlanManager::publishReached(bool reached)
{
    std_msgs::Bool msg;
    msg.data = reached;
    reached_pub_.publish(msg);
}

void PlanManager::publishCenterPath() const
{
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = has_map_ && !occupancy_grid_.header.frame_id.empty()
                              ? occupancy_grid_.header.frame_id
                              : "map";
    msg.poses.reserve(center_path_.size());
    for (const Eigen::Vector2d& point : center_path_) {
        geometry_msgs::PoseStamped pose;
        pose.header = msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.orientation.w = 1.0;
        msg.poses.push_back(pose);
    }
    center_path_pub_.publish(msg);
}

void PlanManager::publishDebug(const Eigen::Vector2d& target,
                               const Eigen::Vector2d& control_target,
                               const geometry_msgs::Twist& cmd,
                               double distance,
                               double formation_yaw) const
{
    if (debug_pub_.getNumSubscribers() <= 0) {
        return;
    }

    std::ostringstream oss;
    oss << "fixed_id=" << fixed_id_
        << " mode=fixed_square"
        << " has_goal=" << has_goal_
        << " has_odom=" << has_odom_
        << " pos=(" << self_state_.pos.x() << "," << self_state_.pos.y() << ")"
        << " yaw=" << self_state_.yaw
        << " center_goal=(" << goal_center_.x() << "," << goal_center_.y() << ")"
        << " slot=(" << slots_[fixed_id_].x() << "," << slots_[fixed_id_].y() << ")"
        << " assigned_slot=" << (fixed_id_ < static_cast<int>(assigned_slot_indices_.size())
                                     ? assigned_slot_indices_[static_cast<std::size_t>(fixed_id_)]
                                     : fixed_id_)
        << " formation_scale=" << active_formation_scale_
        << " formation_yaw=" << formation_yaw
        << " path_points=" << center_path_.size()
        << " path_blocked=" << path_blocked_
        << " dynamic_obstacles=" << dynamic_obstacle_positions_.size()
        << " has_map=" << has_map_
        << " target=(" << target.x() << "," << target.y() << ")"
        << " control_target=(" << control_target.x() << "," << control_target.y() << ")"
        << " distance=" << distance
        << " cmd=(" << cmd.linear.x << "," << cmd.angular.z << ")"
        << " reached=" << reached_;

    std_msgs::String msg;
    msg.data = oss.str();
    debug_pub_.publish(msg);
}

void PlanManager::markPlanDirty()
{
    path_dirty_ = true;
}

void PlanManager::ensureCenterPath()
{
    if (!has_goal_) {
        return;
    }

    const Eigen::Vector2d center = formationCenter();
    const bool goal_moved = (goal_center_ - planned_goal_center_).norm() > path_goal_tolerance_;
    bool off_path = false;
    if (!center_path_.empty()) {
        double min_path_distance = std::numeric_limits<double>::infinity();
        for (const Eigen::Vector2d& point : center_path_) {
            min_path_distance = std::min(min_path_distance, (point - center).norm());
        }
        off_path = min_path_distance > replanning_distance_;
    }
    if (!path_dirty_ && !center_path_.empty() && !off_path && !goal_moved) {
        return;
    }

    path_blocked_ = false;
    if (!planCenterPath()) {
        center_path_.clear();
        if (use_occupancy_grid_planner_ && has_map_) {
            center_path_.push_back(center);
            path_blocked_ = true;
            ROS_WARN("tb %d holding position: no collision-free formation center path", fixed_id_);
        } else {
            center_path_.push_back(center);
            center_path_.push_back(goal_center_);
        }
    }
    planned_start_center_ = center;
    planned_goal_center_ = goal_center_;
    path_dirty_ = false;
    publishCenterPath();
}

bool PlanManager::planCenterPath()
{
    const Eigen::Vector2d start = formationCenter();
    std::vector<Eigen::Vector2d> planned;

    if (use_occupancy_grid_planner_ && has_map_) {
        const double min_scale = allow_formation_scaling_ ? min_formation_scale_ : 1.0;
        auto try_plan_at_scale = [&](double scale) {
            if (planCenterPathOnGrid(start, goal_center_, planned, scale) && planned.size() >= 2) {
                center_path_ = planned;
                active_formation_scale_ = scale;
                ROS_DEBUG("tb %d planned fixed-square center path with %zu points at scale %.2f",
                          fixed_id_,
                          center_path_.size(),
                          active_formation_scale_);
                return true;
            }
            return false;
        };

        for (double scale = 1.0; scale > min_scale + 1e-6; scale -= formation_scale_step_) {
            if (try_plan_at_scale(scale)) {
                return true;
            }
        }
        if (try_plan_at_scale(min_scale)) {
            return true;
        }
        return false;
    }

    center_path_.clear();
    center_path_.push_back(start);
    center_path_.push_back(goal_center_);
    active_formation_scale_ = 1.0;
    ROS_WARN("tb %d using direct fixed-square center path", fixed_id_);
    return true;
}

bool PlanManager::planCenterPathOnGrid(const Eigen::Vector2d& start,
                                       const Eigen::Vector2d& goal,
                                       std::vector<Eigen::Vector2d>& path,
                                       double slot_scale) const
{
    const std::vector<std::uint8_t> occupancy = inflatedOccupancyGrid(slot_scale);
    if (occupancy.empty()) {
        return false;
    }

    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    if (!worldToGrid(start, start_x, start_y) || !worldToGrid(goal, goal_x, goal_y)) {
        ROS_WARN("tb %d center path start/goal outside map bounds", fixed_id_);
        return false;
    }

    if (!closestFreeCell(occupancy, start_x, start_y, start_x, start_y) ||
        !closestFreeCell(occupancy, goal_x, goal_y, goal_x, goal_y)) {
        ROS_WARN("tb %d could not find free start/goal cell for center path", fixed_id_);
        return false;
    }

    struct QueueNode
    {
        int index;
        double priority;
    };
    struct Compare
    {
        bool operator()(const QueueNode& lhs, const QueueNode& rhs) const
        {
            return lhs.priority > rhs.priority;
        }
    };

    const int width = static_cast<int>(occupancy_grid_.info.width);
    const int height = static_cast<int>(occupancy_grid_.info.height);
    const int start_index = gridIndex(start_x, start_y);
    const int goal_index = gridIndex(goal_x, goal_y);
    const std::array<int, 8> dx{{1, 1, 0, -1, -1, -1, 0, 1}};
    const std::array<int, 8> dy{{0, 1, 1, 1, 0, -1, -1, -1}};

    std::priority_queue<QueueNode, std::vector<QueueNode>, Compare> open;
    std::vector<double> cost(occupancy.size(), std::numeric_limits<double>::infinity());
    std::vector<int> parent(occupancy.size(), -1);
    std::vector<std::uint8_t> closed(occupancy.size(), 0);

    auto heuristic = [&](int mx, int my) {
        return std::hypot(static_cast<double>(mx - goal_x), static_cast<double>(my - goal_y));
    };

    cost[static_cast<std::size_t>(start_index)] = 0.0;
    open.push(QueueNode{start_index, heuristic(start_x, start_y)});

    bool found = false;
    while (!open.empty()) {
        const QueueNode current = open.top();
        open.pop();
        if (closed[static_cast<std::size_t>(current.index)] != 0) {
            continue;
        }
        closed[static_cast<std::size_t>(current.index)] = 1;
        if (current.index == goal_index) {
            found = true;
            break;
        }

        const int cx = current.index % width;
        const int cy = current.index / width;
        for (std::size_t i = 0; i < dx.size(); ++i) {
            const int nx = cx + dx[i];
            const int ny = cy + dy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height ||
                cellOccupied(occupancy, nx, ny)) {
                continue;
            }

            const int next_index = gridIndex(nx, ny);
            const double step_cost = (dx[i] != 0 && dy[i] != 0) ? 1.41421356237 : 1.0;
            const double next_cost = cost[static_cast<std::size_t>(current.index)] + step_cost;
            if (next_cost >= cost[static_cast<std::size_t>(next_index)]) {
                continue;
            }
            cost[static_cast<std::size_t>(next_index)] = next_cost;
            parent[static_cast<std::size_t>(next_index)] = current.index;
            open.push(QueueNode{next_index, next_cost + heuristic(nx, ny)});
        }
    }

    if (!found) {
        ROS_WARN("tb %d could not find center path through inflated map", fixed_id_);
        return false;
    }

    std::vector<int> reversed;
    for (int index = goal_index; index >= 0; index = parent[static_cast<std::size_t>(index)]) {
        reversed.push_back(index);
        if (index == start_index) {
            break;
        }
    }
    if (reversed.empty() || reversed.back() != start_index) {
        return false;
    }

    path.clear();
    path.reserve(reversed.size());
    for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
        const int mx = *it % width;
        const int my = *it / width;
        const Eigen::Vector2d point = gridToWorld(mx, my);
        if (path.empty() || (point - path.back()).norm() > occupancy_grid_.info.resolution * 0.5) {
            path.push_back(point);
        }
    }
    if (!path.empty()) {
        path.front() = start;
        path.back() = goal;
    }
    return path.size() >= 2;
}

std::vector<std::uint8_t> PlanManager::inflatedOccupancyGrid(double slot_scale) const
{
    const int width = static_cast<int>(occupancy_grid_.info.width);
    const int height = static_cast<int>(occupancy_grid_.info.height);
    if (!has_map_ || width <= 0 || height <= 0) {
        return {};
    }

    std::vector<std::uint8_t> occupancy(static_cast<std::size_t>(width * height), 0);
    std::vector<int> occupied_indices;
    occupied_indices.reserve(occupancy.size() / 8);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const int index = gridIndex(x, y);
            const int value = occupancy_grid_.data[static_cast<std::size_t>(index)];
            const bool occupied = value >= occupied_threshold_ ||
                                  (value < 0 && treat_unknown_as_occupied_);
            if (occupied) {
                occupancy[static_cast<std::size_t>(index)] = 1;
                occupied_indices.push_back(index);
            }
        }
    }

    const double safe_slot_scale = clamp(slot_scale, min_formation_scale_, 1.0);
    double envelope_radius = robot_radius_ + formation_clearance_;
    for (const Eigen::Vector2d& slot : slots_) {
        envelope_radius = std::max(
            envelope_radius,
            slot.norm() * safe_slot_scale + robot_radius_ + formation_clearance_);
    }
    const double resolution = std::max(static_cast<double>(occupancy_grid_.info.resolution), 1e-6);
    const int inflation_cells = static_cast<int>(std::ceil(envelope_radius / resolution));
    const int inflation_sq = inflation_cells * inflation_cells;

    for (const int index : occupied_indices) {
        const int cx = index % width;
        const int cy = index / width;
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
            for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                if (dx * dx + dy * dy > inflation_sq) {
                    continue;
                }
                const int nx = cx + dx;
                const int ny = cy + dy;
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                    continue;
                }
                occupancy[static_cast<std::size_t>(gridIndex(nx, ny))] = 1;
            }
        }
    }

    const double dynamic_radius = envelope_radius + dynamic_obstacle_radius_;
    for (const Eigen::Vector2d& obstacle : dynamic_obstacle_positions_) {
        markCircularObstacle(occupancy, obstacle, dynamic_radius);
    }
    return occupancy;
}

bool PlanManager::worldToGrid(const Eigen::Vector2d& world, int& mx, int& my) const
{
    if (!has_map_) {
        return false;
    }
    const double resolution = occupancy_grid_.info.resolution;
    const double origin_x = occupancy_grid_.info.origin.position.x;
    const double origin_y = occupancy_grid_.info.origin.position.y;
    mx = static_cast<int>(std::floor((world.x() - origin_x) / resolution));
    my = static_cast<int>(std::floor((world.y() - origin_y) / resolution));
    return mx >= 0 &&
           my >= 0 &&
           mx < static_cast<int>(occupancy_grid_.info.width) &&
           my < static_cast<int>(occupancy_grid_.info.height);
}

Eigen::Vector2d PlanManager::gridToWorld(int mx, int my) const
{
    const double resolution = occupancy_grid_.info.resolution;
    const double origin_x = occupancy_grid_.info.origin.position.x;
    const double origin_y = occupancy_grid_.info.origin.position.y;
    return Eigen::Vector2d(
        origin_x + (static_cast<double>(mx) + 0.5) * resolution,
        origin_y + (static_cast<double>(my) + 0.5) * resolution);
}

int PlanManager::gridIndex(int mx, int my) const
{
    return my * static_cast<int>(occupancy_grid_.info.width) + mx;
}

bool PlanManager::cellOccupied(const std::vector<std::uint8_t>& occupancy, int mx, int my) const
{
    return occupancy[static_cast<std::size_t>(gridIndex(mx, my))] != 0;
}

void PlanManager::markCircularObstacle(std::vector<std::uint8_t>& occupancy,
                                       const Eigen::Vector2d& center,
                                       double radius) const
{
    int center_x = 0;
    int center_y = 0;
    if (!worldToGrid(center, center_x, center_y)) {
        return;
    }

    const int width = static_cast<int>(occupancy_grid_.info.width);
    const int height = static_cast<int>(occupancy_grid_.info.height);
    const double resolution = std::max(static_cast<double>(occupancy_grid_.info.resolution), 1e-6);
    const int radius_cells = static_cast<int>(std::ceil(std::max(0.0, radius) / resolution));
    const int radius_sq = radius_cells * radius_cells;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
            if (dx * dx + dy * dy > radius_sq) {
                continue;
            }
            const int nx = center_x + dx;
            const int ny = center_y + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }
            occupancy[static_cast<std::size_t>(gridIndex(nx, ny))] = 1;
        }
    }
}

bool PlanManager::closestFreeCell(const std::vector<std::uint8_t>& occupancy,
                                  int seed_mx,
                                  int seed_my,
                                  int& free_mx,
                                  int& free_my) const
{
    const int width = static_cast<int>(occupancy_grid_.info.width);
    const int height = static_cast<int>(occupancy_grid_.info.height);
    const int seed_index = gridIndex(seed_mx, seed_my);
    if (seed_mx >= 0 && seed_mx < width && seed_my >= 0 && seed_my < height &&
        occupancy[static_cast<std::size_t>(seed_index)] == 0) {
        free_mx = seed_mx;
        free_my = seed_my;
        return true;
    }

    std::queue<int> queue;
    std::vector<std::uint8_t> visited(occupancy.size(), 0);
    queue.push(seed_index);
    visited[static_cast<std::size_t>(seed_index)] = 1;
    const std::array<int, 8> dx{{1, 1, 0, -1, -1, -1, 0, 1}};
    const std::array<int, 8> dy{{0, 1, 1, 1, 0, -1, -1, -1}};

    while (!queue.empty()) {
        const int index = queue.front();
        queue.pop();
        const int cx = index % width;
        const int cy = index / width;
        for (std::size_t i = 0; i < dx.size(); ++i) {
            const int nx = cx + dx[i];
            const int ny = cy + dy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                continue;
            }
            const int next_index = gridIndex(nx, ny);
            if (visited[static_cast<std::size_t>(next_index)] != 0) {
                continue;
            }
            if (occupancy[static_cast<std::size_t>(next_index)] == 0) {
                free_mx = nx;
                free_my = ny;
                return true;
            }
            visited[static_cast<std::size_t>(next_index)] = 1;
            queue.push(next_index);
        }
    }
    return false;
}

bool PlanManager::ensureSlotAssignment()
{
    if (slot_assignment_ready_) {
        return true;
    }
    if (!allRobotStatesAvailable()) {
        return false;
    }

    std::vector<Eigen::Vector2d> positions;
    positions.reserve(robot_states_.size());
    for (const RobotState& state : robot_states_) {
        positions.push_back(state.pos);
    }

    assigned_slot_indices_ = plan_manager::assignSlotsByCurrentGeometry(
        positions,
        has_robot_state_,
        slots_,
        0.0,
        assigned_slot_indices_);
    slot_assignment_ready_ = true;

    if (fixed_id_ >= 0 && fixed_id_ < static_cast<int>(assigned_slot_indices_.size())) {
        ROS_WARN("tb %d locked response formation slot %d", fixed_id_, assigned_slot_indices_[fixed_id_]);
    }
    return true;
}

bool PlanManager::allRobotStatesAvailable() const
{
    if (has_robot_state_.empty()) {
        return has_odom_;
    }
    for (bool has_state : has_robot_state_) {
        if (!has_state) {
            return false;
        }
    }
    return true;
}

Eigen::Vector2d PlanManager::formationCenter() const
{
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    int count = 0;
    for (std::size_t i = 0; i < robot_states_.size(); ++i) {
        if (!has_robot_state_[i]) {
            continue;
        }
        sum += robot_states_[i].pos;
        ++count;
    }
    if (count > 0) {
        return sum / static_cast<double>(count);
    }

    Eigen::Vector2d slot = assignedSlot(active_formation_scale_);
    const double yaw = rotate_slots_by_goal_yaw_ ? goal_yaw_ : self_state_.yaw;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    slot = Eigen::Vector2d(c * slot.x() - s * slot.y(), s * slot.x() + c * slot.y());
    return self_state_.pos - slot;
}

bool PlanManager::hasUsableCenter() const
{
    for (bool has_state : has_robot_state_) {
        if (has_state) {
            return true;
        }
    }
    return has_odom_;
}

Eigen::Vector2d PlanManager::assignedSlot(double slot_scale) const
{
    int slot_index = fixed_id_;
    if (fixed_id_ >= 0 &&
        fixed_id_ < static_cast<int>(assigned_slot_indices_.size()) &&
        assigned_slot_indices_[static_cast<std::size_t>(fixed_id_)] >= 0 &&
        assigned_slot_indices_[static_cast<std::size_t>(fixed_id_)] < static_cast<int>(slots_.size())) {
        slot_index = assigned_slot_indices_[static_cast<std::size_t>(fixed_id_)];
    }
    if (slot_index < 0 || slot_index >= static_cast<int>(slots_.size())) {
        return Eigen::Vector2d::Zero();
    }
    return slots_[static_cast<std::size_t>(slot_index)] * slot_scale;
}

double PlanManager::commandedFormationScale(const Eigen::Vector2d& center) const
{
    if (!allow_formation_scaling_) {
        return 1.0;
    }

    const double path_scale = clamp(active_formation_scale_, min_formation_scale_, 1.0);
    if (path_scale >= 1.0 - 1e-6) {
        return 1.0;
    }

    const double distance_to_goal = (goal_center_ - center).norm();
    if (!std::isfinite(distance_to_goal) || distance_to_goal <= goal_tolerance_) {
        return 1.0;
    }
    if (distance_to_goal >= formation_expand_distance_) {
        return path_scale;
    }

    const double expand = 1.0 - distance_to_goal / formation_expand_distance_;
    return clamp(path_scale + (1.0 - path_scale) * expand, path_scale, 1.0);
}

Eigen::Vector2d PlanManager::slotTarget(const Eigen::Vector2d& center,
                                        double formation_yaw,
                                        double slot_scale) const
{
    Eigen::Vector2d slot = assignedSlot(slot_scale);
    if (align_slots_to_path_ || rotate_slots_by_goal_yaw_) {
        slot = plan_manager::rotateSlot(slot, formation_yaw);
    }
    return center + slot;
}

Eigen::Vector2d PlanManager::controlTargetFor(const Eigen::Vector2d& center_target,
                                              double& formation_yaw) const
{
    if (center_path_.size() < 2 || !hasUsableCenter()) {
        const Eigen::Vector2d delta = center_target - formationCenter();
        if (delta.norm() > 1e-6) {
            formation_yaw = std::atan2(delta.y(), delta.x());
        }
        return center_target;
    }

    const Eigen::Vector2d center = formationCenter();
    int closest_index = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < center_path_.size(); ++i) {
        const double distance = (center_path_[i] - center).squaredNorm();
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_index = static_cast<int>(i);
        }
    }

    double slot_radius = 0.0;
    for (const Eigen::Vector2d& slot : slots_) {
        slot_radius = std::max(slot_radius, slot.norm() * active_formation_scale_);
    }
    double remaining = lookahead_distance_ + slot_radius;
    Eigen::Vector2d previous = center;
    for (std::size_t i = static_cast<std::size_t>(closest_index + 1); i < center_path_.size(); ++i) {
        const Eigen::Vector2d next = center_path_[i];
        const Eigen::Vector2d segment = next - previous;
        const double length = segment.norm();
        if (length < 1e-6) {
            previous = next;
            continue;
        }
        if (remaining <= length) {
            const Eigen::Vector2d target = previous + segment * (remaining / length);
            formation_yaw = std::atan2(segment.y(), segment.x());
            return target;
        }
        remaining -= length;
        previous = next;
    }

    const Eigen::Vector2d tail = center_path_.back() - center_path_[center_path_.size() - 2];
    if (tail.norm() > 1e-6) {
        formation_yaw = std::atan2(tail.y(), tail.x());
    }
    return center_path_.back();
}

geometry_msgs::Twist PlanManager::computeVelocityCommand(const Eigen::Vector2d& target,
                                                         const Eigen::Vector2d& control_target) const
{
    geometry_msgs::Twist cmd;
    const Eigen::Vector2d to_control = control_target - self_state_.pos;
    const Eigen::Vector2d to_target = target - self_state_.pos;
    const double control_distance = to_control.norm();
    const double target_distance = to_target.norm();

    if (!std::isfinite(control_distance) || control_distance < 1e-6 ||
        !std::isfinite(target_distance)) {
        return cmd;
    }

    const double desired_yaw = std::atan2(to_control.y(), to_control.x());
    const double yaw_error = normalizeAngle(desired_yaw - self_state_.yaw);
    const double angular = clamp(angular_gain_ * yaw_error,
                                 -max_angular_velocity_,
                                 max_angular_velocity_);

    const double alignment = std::max(
        0.0,
        1.0 - std::abs(yaw_error) / angular_slowdown_threshold_);
    double linear = std::min(max_linear_velocity_, linear_gain_ * target_distance) * alignment;
    if (std::abs(yaw_error) > rotate_in_place_threshold_) {
        linear = 0.0;
    }

    cmd.linear.x = linear;
    cmd.angular.z = angular;
    return cmd;
}

double PlanManager::yawFromQuaternion(const geometry_msgs::Quaternion& q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double PlanManager::normalizeAngle(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double PlanManager::clamp(double value, double low, double high)
{
    return std::min(std::max(value, low), high);
}

bool PlanManager::startsWith(const std::string& text, const std::string& prefix)
{
    return text.size() >= prefix.size() &&
           std::equal(prefix.begin(), prefix.end(), text.begin());
}
