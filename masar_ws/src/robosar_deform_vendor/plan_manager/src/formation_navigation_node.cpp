#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <robosar_messages/EmergencyObjectArray.h>
#include <robosar_messages/EmergencyTask.h>
#include <robosar_messages/ResponseGroupStatus.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "plan_manager/clearance_grid.h"
#include "plan_manager/dynamic_obstacle_planning.h"
#include "plan_manager/formation_control.h"
#include "plan_manager/formation_assignment.h"
#include "plan_manager/formation_navigation.h"
#include "plan_manager/formation_planner.h"
#include "plan_manager/velocity_safety.h"

namespace plan_manager
{
namespace
{

double clamp(double value, double lower, double upper)
{
    return std::max(lower, std::min(upper, value));
}

double normalizeAngle(double value)
{
    return std::atan2(std::sin(value), std::cos(value));
}

geometry_msgs::Quaternion yawQuaternion(double yaw)
{
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(quaternion);
}

struct RobotSnapshot
{
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double yaw = 0.0;
    ros::Time odom_stamp;
    ros::Time scan_stamp;
    std::vector<Eigen::Vector2d> scan_points;
};

struct PathReference
{
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    Eigen::Vector2d tangent = Eigen::Vector2d::UnitX();
    double progress = 0.0;
};

struct SceneObstacle
{
    std::uint8_t object_type = 0;
    int object_id = -1;
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double radius = 0.0;
};

PathReference pathReference(
    const std::vector<Eigen::Vector2d>& path,
    const Eigen::Vector2d& center,
    double previous_progress,
    double lookahead)
{
    PathReference output;
    if (path.empty()) return output;
    if (path.size() == 1) {
        output.point = path.front();
        return output;
    }
    std::vector<double> arc(path.size(), 0.0);
    for (std::size_t index = 1; index < path.size(); ++index)
        arc[index] = arc[index - 1] + (path[index] - path[index - 1]).norm();

    double best_distance = std::numeric_limits<double>::infinity();
    double best_progress = previous_progress;
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        const Eigen::Vector2d segment = path[index + 1] - path[index];
        const double length_squared = segment.squaredNorm();
        if (length_squared < 1e-12) continue;
        const double fraction = clamp((center - path[index]).dot(segment) / length_squared, 0.0, 1.0);
        const double progress = arc[index] + fraction * std::sqrt(length_squared);
        if (progress + 0.20 < previous_progress) continue;
        const double distance = (center - (path[index] + fraction * segment)).squaredNorm();
        if (distance < best_distance) {
            best_distance = distance;
            best_progress = std::max(previous_progress, progress);
        }
    }
    output.progress = best_progress;
    const double target = std::min(arc.back(), best_progress + std::max(0.0, lookahead));
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        if (target > arc[index + 1]) continue;
        const double length = std::max(1e-12, arc[index + 1] - arc[index]);
        const double fraction = clamp((target - arc[index]) / length, 0.0, 1.0);
        output.point = path[index] + fraction * (path[index + 1] - path[index]);
        output.tangent = (path[index + 1] - path[index]).normalized();
        return output;
    }
    output.point = path.back();
    output.tangent = (path.back() - path[path.size() - 2]).normalized();
    return output;
}

NavigationEvent navigationEvent(PlanFailure failure)
{
    switch (failure) {
    case PlanFailure::NONE: return NavigationEvent::NONE;
    case PlanFailure::MAP_INVALID: return NavigationEvent::MAP_INVALID;
    case PlanFailure::START_OUTSIDE_MAP: return NavigationEvent::START_OUTSIDE_MAP;
    case PlanFailure::GOAL_OUTSIDE_MAP: return NavigationEvent::GOAL_OUTSIDE_MAP;
    case PlanFailure::START_IN_COLLISION: return NavigationEvent::START_IN_COLLISION;
    case PlanFailure::GOAL_IN_COLLISION: return NavigationEvent::GOAL_IN_COLLISION;
    case PlanFailure::NO_STATIC_PATH: return NavigationEvent::STATIC_NO_PATH;
    }
    return NavigationEvent::MAP_INVALID;
}

const char* planFailureName(PlanFailure failure)
{
    switch (failure) {
    case PlanFailure::NONE: return "NONE";
    case PlanFailure::MAP_INVALID: return "MAP_INVALID";
    case PlanFailure::START_OUTSIDE_MAP: return "START_OUTSIDE_MAP";
    case PlanFailure::GOAL_OUTSIDE_MAP: return "GOAL_OUTSIDE_MAP";
    case PlanFailure::START_IN_COLLISION: return "START_IN_COLLISION";
    case PlanFailure::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
    case PlanFailure::NO_STATIC_PATH: return "NO_STATIC_PATH";
    }
    return "UNKNOWN";
}

}  // namespace

class FormationNavigationNode
{
public:
    FormationNavigationNode()
      : private_nh_("~"), tf_listener_(tf_buffer_),
        state_machine_(parameter("map_startup_timeout", 5.0)),
        planner_(plannerConfig())
    {
        private_nh_.param("frame_id", frame_id_, std::string("map"));
        private_nh_.param("map_topic", map_topic_, std::string("/map"));
        if (!private_nh_.getParam("robot_names", robot_names_))
            robot_names_ = {"tb_0", "tb_1", "tb_2", "tb_3"};
        if (!private_nh_.getParam("external_robot_names", external_robot_names_))
            external_robot_names_ = {"agent_0", "agent_1", "agent_2", "agent_3", "agent_4"};
        if (robot_names_.size() != 4)
            throw std::runtime_error("formation_navigation_node requires exactly four robots");

        nominal_side_ = positiveParameter("formation_interval", 0.60);
        robot_radius_ = positiveParameter("robot_radius", 0.11);
        safety_gap_ = positiveParameter("inter_robot_safety_gap", 0.04);
        formation_clearance_ = positiveParameter("formation_clearance", 0.06);
        transport_box_half_length_ = positiveParameter("transport_box_half_length", 0.25);
        emergency_victim_radius_ = positiveParameter("emergency_victim_radius", 0.12);
        emergency_fire_radius_ = positiveParameter("emergency_fire_radius", 0.10);
        control_rate_ = positiveParameter("control_rate", 10.0);
        odom_timeout_ = positiveParameter("odom_timeout", 0.8);
        scan_timeout_ = positiveParameter("scan_timeout", 0.8);
        scan_self_filter_radius_ = positiveParameter("scan_self_filter_radius", 0.18);
        lookahead_ = positiveParameter("lookahead_distance", 0.60);
        center_speed_ = positiveParameter("center_speed", 0.16);
        slot_gain_ = positiveParameter("slot_gain", 0.85);
        max_linear_ = positiveParameter("max_linear_velocity", 0.20);
        max_angular_ = positiveParameter("max_angular_velocity", 1.20);
        angular_gain_ = positiveParameter("angular_gain", 1.8);
        max_linear_accel_ = positiveParameter("max_linear_acceleration", 0.35);
        max_angular_accel_ = positiveParameter("max_angular_acceleration", 2.0);

        limits_ = FormationLimits{robot_radius_, nominal_side_, safety_gap_,
                                  positiveParameter("max_scale_rate", 0.25)};
        nominal_slots_ = squareNominalSlots(nominal_side_);
        assigned_nominal_slots_ = nominal_slots_;
        velocity_config_ = VelocitySafetyConfig{
            robot_radius_, safety_gap_, positiveParameter("collision_time_horizon", 2.0),
            max_linear_, positiveParameter("obstacle_timeout", 0.8)};
        loadArrivalConfigs();

        status_pub_ = nh_.advertise<robosar_messages::ResponseGroupStatus>(
            "/response_group/status", 10);
        path_pub_ = nh_.advertise<nav_msgs::Path>("/response_group/center_path", 1, true);
        slots_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "/response_group/formation_slots", 1);
        state_marker_pub_ = nh_.advertise<visualization_msgs::Marker>(
            "/response_group/state_text", 1);
        task_sub_ = nh_.subscribe("/response_group/active_task", 10,
                                  &FormationNavigationNode::taskCallback, this);
        mode_sub_ = nh_.subscribe("/response_group/behavior_mode", 10,
                                  &FormationNavigationNode::modeCallback, this);
        motion_sub_ = nh_.subscribe("/response_group/motion_enabled", 1,
                                    &FormationNavigationNode::motionCallback, this);
        map_sub_ = nh_.subscribe(map_topic_, 1, &FormationNavigationNode::mapCallback, this);
        scene_sub_ = nh_.subscribe("/robosar_sim/emergency_objects", 1,
                                   &FormationNavigationNode::sceneCallback, this);
        for (const auto& name : robot_names_) {
            cmd_pubs_.push_back(nh_.advertise<geometry_msgs::Twist>(
                "/" + name + "/deform_cmd_vel", 10));
            odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
                "/" + name + "/odom", 20,
                [this, name](const nav_msgs::Odometry::ConstPtr& msg) { odomCallback(msg, name); }));
            scan_subs_.push_back(nh_.subscribe<sensor_msgs::LaserScan>(
                "/" + name + "/scan", 10,
                [this, name](const sensor_msgs::LaserScan::ConstPtr& msg) { scanCallback(msg, name); }));
            last_commands_[name] = geometry_msgs::Twist();
        }
        timer_ = nh_.createTimer(ros::Duration(1.0 / control_rate_),
                                 &FormationNavigationNode::controlCallback, this);
        publishZero();
    }

private:
    double parameter(const std::string& name, double fallback)
    {
        double value = fallback;
        private_nh_.param(name, value, fallback);
        return value;
    }

    double positiveParameter(const std::string& name, double fallback)
    {
        const double value = parameter(name, fallback);
        if (!std::isfinite(value) || value <= 0.0)
            throw std::runtime_error(name + " must be finite and positive");
        return value;
    }

    PlannerConfig plannerConfig()
    {
        const double side = parameter("formation_interval", 0.60);
        const double radius = parameter("robot_radius", 0.11);
        const double gap = parameter("inter_robot_safety_gap", 0.04);
        const double clearance = parameter("formation_clearance", 0.06);
        const FormationLimits limits{radius, side, gap, 0.0};
        PlannerConfig config;
        config.minimum_envelope_radius = minimumCircularEnvelopeRadius(limits, clearance);
        config.clearance_cost_weight = parameter("clearance_cost_weight", 0.08);
        private_nh_.param("treat_unknown_as_occupied", config.treat_unknown_as_occupied, true);
        return config;
    }

    ArrivalConfig arrivalConfig(const std::string& prefix, ArrivalConfig fallback)
    {
        private_nh_.param(prefix + "/center_tolerance", fallback.center_tolerance,
                          fallback.center_tolerance);
        private_nh_.param(prefix + "/rms_slot_tolerance", fallback.rms_slot_tolerance,
                          fallback.rms_slot_tolerance);
        private_nh_.param(prefix + "/max_slot_tolerance", fallback.max_slot_tolerance,
                          fallback.max_slot_tolerance);
        private_nh_.param(prefix + "/hold_seconds", fallback.hold_seconds,
                          fallback.hold_seconds);
        if (fallback.center_tolerance <= 0.0 || fallback.rms_slot_tolerance <= 0.0 ||
            fallback.max_slot_tolerance <= 0.0 || fallback.hold_seconds < 0.0)
            throw std::runtime_error(prefix + " arrival values are invalid");
        return fallback;
    }

    void loadArrivalConfigs()
    {
        arrival_configs_[ArrivalProfile::WAYPOINT] = arrivalConfig(
            "arrival/waypoint", {0.30, 0.35, 0.55, 0.8});
        arrival_configs_[ArrivalProfile::SURROUND] = arrivalConfig(
            "arrival/surround", {0.25, 0.40, 0.60, 0.8});
        arrival_configs_[ArrivalProfile::TRANSPORT] = arrivalConfig(
            "arrival/transport", {0.40, 0.45, 0.70, 0.8});
        arrival_configs_[ArrivalProfile::HOME] = arrivalConfig(
            "arrival/home", {0.30, 0.40, 0.60, 1.0});
    }

    void taskCallback(const robosar_messages::EmergencyTask::ConstPtr& msg)
    {
        if (msg->task_id == active_task_id_) return;
        active_task_ = *msg;
        active_task_id_ = msg->task_id;
        nominal_target_pose_ = msg->staging_pose;
        if (nominal_target_pose_.header.frame_id.empty() && msg->has_goal_pose)
            nominal_target_pose_ = msg->goal_pose;
        nominal_target_pose_.header.frame_id = nominal_target_pose_.header.frame_id.empty() ?
            frame_id_ : nominal_target_pose_.header.frame_id;
        target_pose_ = nominal_target_pose_;
        profile_target_pose_ = msg->precise_pose;
        if (profile_target_pose_.header.frame_id.empty())
            profile_target_pose_.header.frame_id = nominal_target_pose_.header.frame_id;
        target_yaw_ = tf2::getYaw(nominal_target_pose_.pose.orientation);
        if (msg->task_id < 0) profile_ = ArrivalProfile::HOME;
        else if (msg->task_type == robosar_messages::EmergencyTask::BOX)
            profile_ = ArrivalProfile::TRANSPORT;
        else profile_ = ArrivalProfile::SURROUND;
        state_machine_.resetGoal();
        arrival_evaluator_.reset();
        path_.clear();
        assigned_nominal_slots_ = nominal_slots_;
        assignment_ready_ = false;
        formation_yaw_ready_ = false;
        plan_failure_ = PlanFailure::NONE;
    }

    void modeCallback(const std_msgs::String::ConstPtr& msg) { mode_ = msg->data; }
    void motionCallback(const std_msgs::Bool::ConstPtr& msg) { motion_enabled_ = msg->data; }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        if (msg->header.frame_id.empty()) {
            ROS_ERROR_THROTTLE(2.0, "Rejected occupancy grid with empty frame_id");
            return;
        }
        std::unique_ptr<ClearanceGrid> candidate(new ClearanceGrid(*msg, true));
        if (!candidate->valid()) {
            ROS_ERROR_THROTTLE(2.0, "Rejected invalid occupancy grid: %s", candidate->error().c_str());
            return;
        }
        map_ = *msg;
        frame_id_ = msg->header.frame_id;
        clearance_grid_ = std::move(candidate);
        map_ready_ = true;
    }

    double emergencyObjectRadius(std::uint8_t object_type) const
    {
        if (object_type == robosar_messages::EmergencyTask::VICTIM)
            return emergency_victim_radius_;
        if (object_type == robosar_messages::EmergencyTask::FIRE)
            return emergency_fire_radius_;
        if (object_type == robosar_messages::EmergencyTask::BOX)
            return std::sqrt(2.0) * transport_box_half_length_;
        return 0.0;
    }

    void sceneCallback(const robosar_messages::EmergencyObjectArray::ConstPtr& msg)
    {
        std::vector<SceneObstacle> next;
        next.reserve(msg->objects.size());
        for (const auto& object : msg->objects) {
            const double radius = emergencyObjectRadius(object.object_type);
            if (radius <= 0.0) continue;
            geometry_msgs::PoseStamped pose = object.precise_pose;
            if (pose.header.frame_id.empty()) pose.header.frame_id = msg->header.frame_id;
            if (pose.header.frame_id.empty()) {
                ROS_ERROR_THROTTLE(2.0, "Emergency-object snapshot contains an empty frame_id");
                continue;
            }
            try {
                if (pose.header.frame_id != frame_id_)
                    pose = tf_buffer_.transform(pose, frame_id_, ros::Duration(0.05));
            } catch (const tf2::TransformException& error) {
                ROS_WARN_THROTTLE(2.0, "Waiting for emergency-object transform to %s: %s",
                                  frame_id_.c_str(), error.what());
                continue;
            }
            if (!std::isfinite(pose.pose.position.x) ||
                !std::isfinite(pose.pose.position.y)) continue;
            next.push_back({object.object_type, object.object_id,
                            Eigen::Vector2d(pose.pose.position.x, pose.pose.position.y),
                            radius});
        }
        std::sort(next.begin(), next.end(), [](const SceneObstacle& left,
                                               const SceneObstacle& right) {
            if (left.object_type != right.object_type)
                return left.object_type < right.object_type;
            return left.object_id < right.object_id;
        });
        scene_obstacles_ = std::move(next);
        scene_obstacle_stamp_ = ros::Time::now();
    }

    bool isActiveTaskObject(const SceneObstacle& obstacle) const
    {
        return active_task_id_ >= 0 &&
            active_task_.task_type == obstacle.object_type &&
            active_task_.object_id == obstacle.object_id;
    }

    std::vector<SceneObstacle> nonActiveSceneObstacles() const
    {
        std::vector<SceneObstacle> output;
        for (const SceneObstacle& obstacle : scene_obstacles_) {
            if (!isActiveTaskObject(obstacle)) output.push_back(obstacle);
        }
        return output;
    }

    std::vector<MovingDisc> currentExternalDiscs(const ros::Time& now) const
    {
        std::vector<MovingDisc> output;
        output.reserve(external_robot_names_.size());
        for (const std::string& name : external_robot_names_) {
            const auto found = robots_.find(name);
            if (found == robots_.end() || found->second.odom_stamp.isZero() ||
                (now - found->second.odom_stamp).toSec() > odom_timeout_) {
                continue;
            }
            Eigen::Vector2d velocity = found->second.velocity;
            const double speed = velocity.norm();
            if (speed > max_linear_ && speed > 1e-9) {
                velocity *= max_linear_ / speed;
            }
            output.push_back({name, found->second.position, velocity, robot_radius_});
        }
        return output;
    }

    std::vector<CircularMapObstacle> predictedExternalObstacles(
        const ros::Time& now) const
    {
        const double resolution = map_ready_
            ? std::max(1e-3, static_cast<double>(map_.info.resolution))
            : robot_radius_;
        return sweptCircularObstacles(
            currentExternalDiscs(now), velocity_config_.time_horizon, resolution);
    }

    std::vector<CircularMapObstacle> planningSceneObstacles(
        const Eigen::Vector2d& actual_center) const
    {
        std::vector<CircularMapObstacle> output;
        const double envelope = minimumCircularEnvelopeRadius(limits_, formation_clearance_);
        for (const SceneObstacle& obstacle : nonActiveSceneObstacles()) {
            // A just-completed surround leaves the formation center on top of
            // the old target while every robot is physically clear of it. Do
            // not make that valid release pose an impossible A* start; the
            // per-robot velocity layer still enforces the obstacle immediately.
            if ((actual_center - obstacle.center).norm() <= obstacle.radius + envelope)
                continue;
            output.push_back({obstacle.center, obstacle.radius});
        }
        return output;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg, const std::string& name)
    {
        geometry_msgs::PoseStamped input;
        input.header = msg->header;
        input.pose = msg->pose.pose;
        if (input.header.frame_id.empty()) input.header.frame_id = frame_id_;
        try {
            const geometry_msgs::PoseStamped transformed = tf_buffer_.transform(
                input, frame_id_, ros::Duration(0.05));
            RobotSnapshot& robot = robots_[name];
            robot.position = Eigen::Vector2d(
                transformed.pose.position.x, transformed.pose.position.y);
            robot.yaw = tf2::getYaw(transformed.pose.orientation);
            const double body_speed = msg->twist.twist.linear.x;
            robot.velocity = Eigen::Vector2d(
                body_speed * std::cos(robot.yaw), body_speed * std::sin(robot.yaw));
            robot.odom_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        } catch (const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(2.0, "Waiting for %s odom transform to %s: %s",
                              name.c_str(), frame_id_.c_str(), error.what());
        }
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg, const std::string& name)
    {
        auto found = robots_.find(name);
        if (found == robots_.end()) return;
        RobotSnapshot& robot = found->second;
        robot.scan_points.clear();
        const std::size_t stride = std::max<std::size_t>(1, msg->ranges.size() / 180);
        for (std::size_t index = 0; index < msg->ranges.size(); index += stride) {
            const double range = msg->ranges[index];
            if (!std::isfinite(range) || range < msg->range_min ||
                range <= scan_self_filter_radius_ || range > msg->range_max || range > 2.5)
                continue;
            const double angle = robot.yaw + msg->angle_min + index * msg->angle_increment;
            robot.scan_points.push_back(robot.position + range * Eigen::Vector2d(
                std::cos(angle), std::sin(angle)));
        }
        robot.scan_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    }

    bool odomReady(const ros::Time& now) const
    {
        for (const auto& name : robot_names_) {
            const auto found = robots_.find(name);
            if (found == robots_.end() || found->second.odom_stamp.isZero() ||
                (now - found->second.odom_stamp).toSec() > odom_timeout_) return false;
        }
        return true;
    }

    Eigen::Vector2d center() const
    {
        Eigen::Vector2d result = Eigen::Vector2d::Zero();
        for (const auto& name : robot_names_) result += robots_.at(name).position;
        return result / static_cast<double>(robot_names_.size());
    }

    bool plan(const Eigen::Vector2d& actual_center)
    {
        geometry_msgs::PoseStamped transformed = nominal_target_pose_;
        try {
            transformed = tf_buffer_.transform(
                nominal_target_pose_, frame_id_, ros::Duration(0.10));
        } catch (const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(2.0, "Waiting for goal transform to %s: %s",
                              frame_id_.c_str(), error.what());
            return false;
        }
        target_pose_ = transformed;
        goal_ = Eigen::Vector2d(transformed.pose.position.x, transformed.pose.position.y);
        target_yaw_ = tf2::getYaw(transformed.pose.orientation);

        const nav_msgs::OccupancyGrid planning_map = overlayCircularObstacles(
            map_, planningSceneObstacles(actual_center));
        const ros::Time planning_time = ros::Time::now();
        const std::vector<CircularMapObstacle> dynamic_obstacles =
            predictedExternalObstacles(planning_time);
        std::unique_ptr<ClearanceGrid> planning_grid(new ClearanceGrid(planning_map, true));
        if (!planning_grid->valid()) {
            plan_failure_ = PlanFailure::MAP_INVALID;
            path_.clear();
            ROS_ERROR("Emergency-object occupancy overlay produced an invalid map: %s",
                      planning_grid->error().c_str());
            return false;
        }

        if (profile_ == ArrivalProfile::TRANSPORT) {
            geometry_msgs::PoseStamped transformed_box = profile_target_pose_;
            try {
                transformed_box = tf_buffer_.transform(
                    profile_target_pose_, frame_id_, ros::Duration(0.10));
            } catch (const tf2::TransformException& error) {
                ROS_WARN_THROTTLE(2.0, "Waiting for transport-object transform to %s: %s",
                                  frame_id_.c_str(), error.what());
                return false;
            }
            profile_target_ = Eigen::Vector2d(
                transformed_box.pose.position.x, transformed_box.pose.position.y);
            const Eigen::Vector2d forward(std::cos(target_yaw_), std::sin(target_yaw_));
            const double minimum_scale = minimumFormationScale(limits_);
            const double minimum_rear = transport_box_half_length_ + robot_radius_ +
                safety_gap_ + 0.5 * nominal_side_ * minimum_scale;
            const double nominal_rear = std::max(
                minimum_rear, (profile_target_ - goal_).dot(forward));
            const TransportApproachSearch search{
                minimum_rear,
                std::max(minimum_rear + nominal_side_, nominal_rear + nominal_side_),
                nominal_side_,
                planning_grid->resolution(),
            };
            const double required_clearance =
                minimumCircularEnvelopeRadius(limits_, formation_clearance_);
            const TransportApproachResult approach = resolveTransportApproach(
                profile_target_, forward, goal_, search,
                [&planning_grid, required_clearance](const Eigen::Vector2d& candidate) {
                    return planning_grid->clearanceAtWorld(candidate) >= required_clearance;
                });
            if (!approach.found) {
                plan_failure_ = PlanFailure::GOAL_IN_COLLISION;
                path_.clear();
                minimum_clearance_ = 0.0;
                ROS_ERROR("No collision-free formation center exists in the transport "
                          "approach region; no direct fallback will be used");
                return false;
            }
            if ((approach.point - goal_).norm() > 1e-6) {
                ROS_INFO("Resolved transport approach center from (%.3f, %.3f) to "
                         "(%.3f, %.3f)", goal_.x(), goal_.y(),
                         approach.point.x(), approach.point.y());
            }
            goal_ = approach.point;
            target_pose_.pose.position.x = goal_.x();
            target_pose_.pose.position.y = goal_.y();
        }
        // A task controller may end in a compact, non-square contact geometry
        // whose centroid cannot contain the planner's conservative minimum
        // circular envelope. Physical transport can also leave a front robot
        // touching a wall/box boundary. Permit the planner's monotone
        // start-release only while every measured robot center is still in
        // free occupancy space. The A* clearance gradient plus per-robot
        // lidar/velocity safety then recover the full footprint and formation
        // envelope; an actually occupied center remains a hard failure.
        const bool actual_centers_clear = allRobotCentersOutsideOccupiedSpace(
            *planning_grid, robotPositions());
        const CenterPlan result = planner_.plan(
            planning_map, actual_center, goal_, actual_centers_clear,
            dynamic_obstacles);
        minimum_clearance_ = result.minimum_clearance;
        if (result.failure != PlanFailure::NONE) {
            plan_failure_ = result.failure;
            path_.clear();
            ROS_ERROR("Formation center planning failed: %s; no direct fallback will be used",
                      planFailureName(result.failure));
            return false;
        }
        plan_failure_ = PlanFailure::NONE;
        path_ = result.points;
        planning_clearance_grid_ = std::move(planning_grid);
        if (path_.size() >= 2) {
            const double initial_yaw = std::atan2(
                path_[1].y() - path_[0].y(), path_[1].x() - path_[0].x());
            if (!assignment_ready_) {
                const std::vector<Eigen::Vector2d> positions = robotPositions();
                const std::vector<int> assignment = assignSlotsByCurrentGeometry(
                    positions, std::vector<bool>(positions.size(), true), nominal_slots_,
                    initial_yaw, {0, 1, 2, 3});
                assigned_nominal_slots_.clear();
                for (int slot : assignment) {
                    assigned_nominal_slots_.push_back(
                        nominal_slots_.at(static_cast<std::size_t>(slot)));
                }
                assignment_ready_ = true;
            }
            if (!formation_yaw_ready_) {
                formation_yaw_ = initial_yaw;
                formation_yaw_ready_ = true;
            }
        }
        plan_ready_event_ = true;
        publishPath();
        return true;
    }

    void controlCallback(const ros::TimerEvent& event)
    {
        const ros::Time now = ros::Time::now();
        refreshExternalRobots(now);
        const bool has_goal = active_task_id_ != std::numeric_limits<int>::min();
        NavigationInput input;
        input.has_goal = has_goal;
        input.motion_enabled = motion_enabled_ && mode_ == "NAV_FORMATION";
        input.map_ready = map_ready_;
        input.odom_ready = odomReady(now);
        if (plan_failure_ != PlanFailure::NONE)
            input.planning_event = navigationEvent(plan_failure_);
        else if (plan_ready_event_) input.planning_event = NavigationEvent::PLAN_READY;
        NavigationDecision decision = state_machine_.update(input, now.toSec());
        plan_ready_event_ = false;

        if (decision.request_plan && input.map_ready && input.odom_ready) {
            plan(center());
            input.planning_event = plan_failure_ != PlanFailure::NONE ?
                navigationEvent(plan_failure_) :
                (path_.empty() ? NavigationEvent::NONE : NavigationEvent::PLAN_READY);
            decision = state_machine_.update(input, now.toSec());
            plan_ready_event_ = false;
        }
        if (!decision.command_motion || path_.empty() || !input.odom_ready) {
            publishZero();
            publishStatus(decision, FormationQuality(), ArrivalEvidence());
            return;
        }

        const Eigen::Vector2d actual_center = center();
        PathReference reference = pathReference(
            path_, actual_center, state_machine_.pathProgress(), lookahead_);
        state_machine_.setPathProgress(reference.progress);
        const double path_yaw = std::atan2(reference.tangent.y(), reference.tangent.x());
        const double dt = event.current_real.isZero() ? 0.1 :
            std::max(0.001, (event.current_real - event.last_real).toSec());
        if (!formation_yaw_ready_) {
            formation_yaw_ = path_yaw;
            formation_yaw_ready_ = true;
        } else {
            formation_yaw_ = updateFormationYaw(
                formation_yaw_, path_yaw, center_speed_,
                nominal_side_ / std::sqrt(2.0), dt);
        }
        updateDeformation(reference.point, dt);
        slot_targets_ = affineSlotTargets(
            reference.point, formation_yaw_, assigned_nominal_slots_, deformation_);
        const std::vector<Eigen::Vector2d> positions = robotPositions();
        const FormationQuality quality = evaluateFormationQuality(
            positions, slot_targets_, goal_, 2.0 * robot_radius_ + safety_gap_);
        const ArrivalEvidence arrival = arrival_evaluator_.evaluate(
            profile_, arrival_configs_.at(profile_), quality, positions,
            profile_ == ArrivalProfile::TRANSPORT ? profile_target_ : goal_,
            target_yaw_, now.toSec());
        if (arrival.arrived) {
            input.planning_event = NavigationEvent::ARRIVED;
            decision = state_machine_.update(input, now.toSec());
            publishZero();
            publishStatus(decision, quality, arrival, reference.point);
            publishMarkers(decision, formation_yaw_);
            return;
        }

        const double formation_speed_scale = formationQualitySpeedScale(
            quality.max_slot_error, nominal_side_);
        double goal_speed_scale = 1.0;
        if ((goal_ - actual_center).norm() < lookahead_) {
            goal_speed_scale = clamp(
                (goal_ - actual_center).norm() / lookahead_, 0.15, 1.0);
        }
        bool all_blocked = true;
        for (std::size_t index = 0; index < robot_names_.size(); ++index) {
            const std::string& name = robot_names_[index];
            const RobotSnapshot& robot = robots_.at(name);
            Eigen::Vector2d preferred = slot_gain_ * (slot_targets_[index] - robot.position) +
                center_speed_ * formation_speed_scale * goal_speed_scale * reference.tangent;
            const SafeVelocityResult safe = projectSafeVelocity(
                robot.position, robot.velocity, preferred, obstaclesFor(name, now),
                velocity_config_, now);
            all_blocked = all_blocked && safe.blocked;
            publishRobotCommand(name, safe.velocity, formation_yaw_, now,
                                (event.current_real - event.last_real).toSec());
        }
        if (all_blocked) {
            input.planning_event = NavigationEvent::DYNAMIC_BLOCKED;
            decision = state_machine_.update(input, now.toSec());
            publishZero();
        }
        publishStatus(decision, quality, arrival, reference.point);
        publishMarkers(decision, formation_yaw_);
    }

    void refreshExternalRobots(const ros::Time& now)
    {
        for (const auto& name : external_robot_names_) {
            try {
                const geometry_msgs::TransformStamped transformed =
                    tf_buffer_.lookupTransform(
                        frame_id_, name + "/base_link", ros::Time(0),
                        ros::Duration(0.01));
                RobotSnapshot& robot = robots_[name];
                const Eigen::Vector2d position(
                    transformed.transform.translation.x,
                    transformed.transform.translation.y);
                const double dt = robot.odom_stamp.isZero()
                    ? 0.0 : (now - robot.odom_stamp).toSec();
                if (dt > 1e-3 && dt < 2.0)
                    robot.velocity = (position - robot.position) / dt;
                else robot.velocity = Eigen::Vector2d::Zero();
                robot.position = position;
                robot.yaw = tf2::getYaw(transformed.transform.rotation);
                robot.odom_stamp = now;
            } catch (const tf2::TransformException& error) {
                ROS_WARN_THROTTLE(
                    2.0, "Waiting for external robot %s TF in %s: %s",
                    name.c_str(), frame_id_.c_str(), error.what());
            }
        }
    }

    void updateDeformation(const Eigen::Vector2d& reference, double dt)
    {
        const ClearanceGrid* grid = planning_clearance_grid_ ?
            planning_clearance_grid_.get() : clearance_grid_.get();
        double clearance = grid ? grid->clearanceAtWorld(reference) : 0.0;
        const double corner_radius = nominal_side_ / std::sqrt(2.0);
        const double available = clearance - robot_radius_ - formation_clearance_;
        const double scale = clamp(available / std::max(1e-6, corner_radius),
                                   minimumFormationScale(limits_), 1.0);
        deformation_ = updateDeformationScales(deformation_, scale, scale, limits_, dt);
    }

    std::vector<Eigen::Vector2d> robotPositions() const
    {
        std::vector<Eigen::Vector2d> output;
        for (const auto& name : robot_names_) output.push_back(robots_.at(name).position);
        return output;
    }

    std::vector<DiscObstacle> obstaclesFor(const std::string& name, const ros::Time& now) const
    {
        std::vector<DiscObstacle> obstacles;
        for (const auto& other : robot_names_) {
            if (other == name) continue;
            const RobotSnapshot& robot = robots_.at(other);
            obstacles.push_back({robot.position, robot.velocity, robot_radius_,
                                 robot.odom_stamp, true});
        }
        for (const auto& other : external_robot_names_) {
            const auto found = robots_.find(other);
            if (found == robots_.end() || found->second.odom_stamp.isZero() ||
                (now - found->second.odom_stamp).toSec() > odom_timeout_) continue;
            const RobotSnapshot& robot = found->second;
            // Exploration yields early, but response navigation still owns a
            // hard TTC invariant.  Mark cross-group robots for early radial
            // separation so differential-drive turn/deceleration latency is
            // paid before the physical safety discs overlap.
            obstacles.push_back({
                robot.position, robot.velocity, robot_radius_, robot.odom_stamp, true,
            });
        }
        const RobotSnapshot& robot = robots_.at(name);
        if (!robot.scan_stamp.isZero() && (now - robot.scan_stamp).toSec() <= scan_timeout_) {
            for (const auto& point : robot.scan_points)
                obstacles.push_back({point, Eigen::Vector2d::Zero(), 0.03, robot.scan_stamp});
        }
        // The center planner deliberately omits the active task object so the
        // formation can center on a victim/fire or stage behind a box.  That
        // topological exception must not leak into per-robot safety: every
        // robot still has to travel around the object's physical footprint.
        // During TASK_CONTROL this navigation node is paused, so retaining the
        // active box here does not prevent the transport controller's intended
        // robot-box and robot-robot contact.
        for (const SceneObstacle& obstacle : scene_obstacles_) {
            obstacles.push_back({obstacle.center, Eigen::Vector2d::Zero(), obstacle.radius,
                                 scene_obstacle_stamp_.isZero() ? now : scene_obstacle_stamp_});
        }
        return obstacles;
    }

    void publishRobotCommand(
        const std::string& name, const Eigen::Vector2d& map_velocity,
        double terminal_yaw, const ros::Time&, double dt)
    {
        const RobotSnapshot& robot = robots_.at(name);
        const DifferentialDriveTarget target = differentialDriveTarget(
            map_velocity, robot.yaw, terminal_yaw,
            std::max(1e-4, 0.05 * max_linear_));
        double linear = target.linear_velocity;
        double angular = clamp(
            angular_gain_ * target.heading_error, -max_angular_, max_angular_);
        geometry_msgs::Twist& previous = last_commands_[name];
        const double safe_dt = clamp(dt, 0.001, 0.25);
        linear = clamp(linear, previous.linear.x - max_linear_accel_ * safe_dt,
                       previous.linear.x + max_linear_accel_ * safe_dt);
        angular = clamp(angular, previous.angular.z - max_angular_accel_ * safe_dt,
                        previous.angular.z + max_angular_accel_ * safe_dt);
        geometry_msgs::Twist command;
        command.linear.x = clamp(linear, -max_linear_, max_linear_);
        command.angular.z = angular;
        previous = command;
        const auto index = std::find(robot_names_.begin(), robot_names_.end(), name) - robot_names_.begin();
        cmd_pubs_[index].publish(command);
    }

    void publishZero()
    {
        geometry_msgs::Twist zero;
        for (std::size_t index = 0; index < cmd_pubs_.size(); ++index) {
            cmd_pubs_[index].publish(zero);
            last_commands_[robot_names_[index]] = zero;
        }
    }

    geometry_msgs::PoseStamped pose(const Eigen::Vector2d& point, double yaw = 0.0) const
    {
        geometry_msgs::PoseStamped output;
        output.header.stamp = ros::Time::now();
        output.header.frame_id = frame_id_;
        output.pose.position.x = point.x();
        output.pose.position.y = point.y();
        output.pose.orientation = yawQuaternion(yaw);
        return output;
    }

    void publishPath()
    {
        nav_msgs::Path message;
        message.header.stamp = ros::Time::now();
        message.header.frame_id = frame_id_;
        for (const auto& point : path_) message.poses.push_back(pose(point));
        path_pub_.publish(message);
    }

    void publishStatus(
        const NavigationDecision& decision,
        const FormationQuality& quality,
        const ArrivalEvidence& arrival,
        const Eigen::Vector2d& reference = Eigen::Vector2d::Zero())
    {
        robosar_messages::ResponseGroupStatus status;
        status.header.stamp = ros::Time::now();
        status.header.frame_id = frame_id_;
        status.active_task_id = active_task_id_;
        status.active_task_type = active_task_.task_type;
        status.behavior_type = active_task_.behavior_type;
        status.detail = decision.detail;
        status.planning_state = navigationStateName(decision.state);
        status.motion_enabled = motion_enabled_ && mode_ == "NAV_FORMATION";
        status.assigned_robots = robot_names_;
        status.longitudinal_scale = deformation_.longitudinal_scale;
        status.lateral_scale = deformation_.lateral_scale;
        status.rms_slot_error = std::isfinite(quality.rms_slot_error) ? quality.rms_slot_error : 0.0;
        status.max_slot_error = std::isfinite(quality.max_slot_error) ? quality.max_slot_error : 0.0;
        status.minimum_clearance = minimum_clearance_;
        status.hold_remaining = arrival.hold_remaining;
        if (odomReady(status.header.stamp)) status.formation_center = pose(center());
        status.target_pose = target_pose_;
        status.center_reference = pose(reference);
        for (const auto& target : slot_targets_) status.slot_targets.push_back(pose(target));
        if (decision.state == NavigationState::DONE) status.state = status.DONE;
        else if (decision.state == NavigationState::FAILED || decision.state == NavigationState::ERROR)
            status.state = status.FAILED;
        else if (active_task_id_ == std::numeric_limits<int>::min()) status.state = status.IDLE;
        else status.state = status.NAVIGATING;
        status_pub_.publish(status);
    }

    void publishMarkers(const NavigationDecision& decision, double yaw)
    {
        visualization_msgs::MarkerArray array;
        for (std::size_t index = 0; index < slot_targets_.size(); ++index) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = ros::Time::now();
            marker.ns = "response_slots";
            marker.id = index;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = pose(slot_targets_[index], yaw).pose;
            marker.scale.x = marker.scale.y = 0.18;
            marker.scale.z = 0.06;
            marker.color.g = marker.color.a = 0.9;
            array.markers.push_back(marker);
        }
        slots_pub_.publish(array);
        visualization_msgs::Marker text;
        text.header.frame_id = frame_id_;
        text.header.stamp = ros::Time::now();
        text.ns = "response_state";
        text.id = 0;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose = pose(odomReady(text.header.stamp) ? center() : Eigen::Vector2d::Zero()).pose;
        text.pose.position.z = 0.8;
        text.scale.z = 0.25;
        text.color.r = text.color.g = text.color.b = text.color.a = 1.0;
        text.text = decision.detail;
        state_marker_pub_.publish(text);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    NavigationStateMachine state_machine_;
    FormationPlanner planner_;
    ArrivalEvaluator arrival_evaluator_;
    FormationLimits limits_;
    VelocitySafetyConfig velocity_config_;
    DeformationState deformation_;
    std::vector<Eigen::Vector2d> nominal_slots_;
    std::vector<Eigen::Vector2d> assigned_nominal_slots_;
    std::vector<Eigen::Vector2d> slot_targets_;
    std::unordered_map<ArrivalProfile, ArrivalConfig> arrival_configs_;
    std::unordered_map<std::string, RobotSnapshot> robots_;
    std::unordered_map<std::string, geometry_msgs::Twist> last_commands_;
    std::vector<std::string> robot_names_;
    std::vector<std::string> external_robot_names_;
    std::vector<ros::Publisher> cmd_pubs_;
    std::vector<ros::Subscriber> odom_subs_;
    std::vector<ros::Subscriber> scan_subs_;
    ros::Publisher status_pub_, path_pub_, slots_pub_, state_marker_pub_;
    ros::Subscriber task_sub_, mode_sub_, motion_sub_, map_sub_, scene_sub_;
    ros::Timer timer_;
    nav_msgs::OccupancyGrid map_;
    std::unique_ptr<ClearanceGrid> clearance_grid_;
    std::unique_ptr<ClearanceGrid> planning_clearance_grid_;
    std::vector<SceneObstacle> scene_obstacles_;
    ros::Time scene_obstacle_stamp_;
    robosar_messages::EmergencyTask active_task_;
    geometry_msgs::PoseStamped nominal_target_pose_;
    geometry_msgs::PoseStamped target_pose_;
    geometry_msgs::PoseStamped profile_target_pose_;
    std::vector<Eigen::Vector2d> path_;
    Eigen::Vector2d goal_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d profile_target_ = Eigen::Vector2d::Zero();
    std::string frame_id_, map_topic_, mode_ = "STOP";
    int active_task_id_ = std::numeric_limits<int>::min();
    ArrivalProfile profile_ = ArrivalProfile::WAYPOINT;
    bool motion_enabled_ = false, map_ready_ = false;
    bool assignment_ready_ = false, formation_yaw_ready_ = false;
    PlanFailure plan_failure_ = PlanFailure::NONE;
    bool plan_ready_event_ = false;
    double target_yaw_ = 0.0, formation_yaw_ = 0.0, minimum_clearance_ = 0.0;
    double nominal_side_, robot_radius_, safety_gap_, formation_clearance_;
    double transport_box_half_length_, emergency_victim_radius_, emergency_fire_radius_;
    double control_rate_, odom_timeout_, scan_timeout_, scan_self_filter_radius_;
    double lookahead_, center_speed_, slot_gain_;
    double max_linear_, max_angular_, angular_gain_, max_linear_accel_, max_angular_accel_;
};

}  // namespace plan_manager

int main(int argc, char** argv)
{
    ros::init(argc, argv, "formation_navigation_node");
    try {
        plan_manager::FormationNavigationNode node;
        ros::spin();
    } catch (const std::exception& error) {
        ROS_FATAL("formation_navigation_node startup failed: %s", error.what());
        return 1;
    }
    return 0;
}
