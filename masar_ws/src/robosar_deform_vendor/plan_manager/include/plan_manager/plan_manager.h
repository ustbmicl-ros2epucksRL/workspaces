#ifndef PLAN_MANAGER_PLAN_MANAGER_H
#define PLAN_MANAGER_PLAN_MANAGER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <Eigen/Eigen>
#include <cstdint>
#include <unordered_map>
#include <string>
#include <vector>

struct RobotState
{
    Eigen::Vector2d pos = Eigen::Vector2d::Zero();
    double yaw = 0.0;
};

class PlanManager
{
public:
    PlanManager();
    ~PlanManager() = default;

private:
    void readParameters();
    void resetFormationSlots();
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void teammateOdomCallback(const nav_msgs::OdometryConstPtr& msg, int robot_id);
    void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
    void modelStatesCallback(const gazebo_msgs::ModelStatesConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
    void controlCallback(const ros::TimerEvent&);
    void publishZero();
    void publishReached(bool reached);
    void publishCenterPath() const;
    void publishDebug(const Eigen::Vector2d& target,
                      const Eigen::Vector2d& control_target,
                      const geometry_msgs::Twist& cmd,
                      double distance,
                      double formation_yaw) const;

    void markPlanDirty();
    void ensureCenterPath();
    bool planCenterPath();
    bool planCenterPathOnGrid(const Eigen::Vector2d& start,
                              const Eigen::Vector2d& goal,
                              std::vector<Eigen::Vector2d>& path,
                              double slot_scale) const;
    std::vector<std::uint8_t> inflatedOccupancyGrid(double slot_scale) const;
    bool worldToGrid(const Eigen::Vector2d& world, int& mx, int& my) const;
    Eigen::Vector2d gridToWorld(int mx, int my) const;
    int gridIndex(int mx, int my) const;
    bool cellOccupied(const std::vector<std::uint8_t>& occupancy, int mx, int my) const;
    void markCircularObstacle(std::vector<std::uint8_t>& occupancy,
                              const Eigen::Vector2d& center,
                              double radius) const;
    bool closestFreeCell(const std::vector<std::uint8_t>& occupancy,
                         int seed_mx,
                         int seed_my,
                         int& free_mx,
                         int& free_my) const;
    bool ensureSlotAssignment();
    bool allRobotStatesAvailable() const;
    Eigen::Vector2d formationCenter() const;
    bool hasUsableCenter() const;
    Eigen::Vector2d assignedSlot(double slot_scale = 1.0) const;
    double commandedFormationScale(const Eigen::Vector2d& center) const;
    Eigen::Vector2d slotTarget(const Eigen::Vector2d& center,
                               double formation_yaw,
                               double slot_scale) const;
    Eigen::Vector2d controlTargetFor(const Eigen::Vector2d& center,
                                     double& formation_yaw) const;
    geometry_msgs::Twist computeVelocityCommand(const Eigen::Vector2d& target,
                                                const Eigen::Vector2d& control_target) const;

    static double yawFromQuaternion(const geometry_msgs::Quaternion& q);
    static double normalizeAngle(double angle);
    static double clamp(double value, double low, double high);
    static bool startsWith(const std::string& text, const std::string& prefix);

    ros::NodeHandle nh_;
    ros::NodeHandle nh_pri_;
    ros::Subscriber goal_sub_;
    ros::Subscriber odom_sub_;
    std::vector<ros::Subscriber> teammate_odom_subs_;
    ros::Subscriber map_sub_;
    ros::Subscriber model_states_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher reached_pub_;
    ros::Publisher debug_pub_;
    ros::Publisher center_path_pub_;
    ros::Timer control_timer_;

    int fixed_id_ = 0;
    int fixed_num_ = 4;
    double formation_interval_ = 0.8;
    double robot_radius_ = 0.18;
    double formation_clearance_ = 0.18;
    double replanning_distance_ = 0.35;
    double path_goal_tolerance_ = 0.35;
    double goal_tolerance_ = 0.18;
    double lookahead_distance_ = 0.6;
    double control_rate_hz_ = 10.0;
    double max_linear_velocity_ = 0.22;
    double max_angular_velocity_ = 1.4;
    double linear_gain_ = 0.45;
    double angular_gain_ = 1.8;
    double angular_slowdown_threshold_ = 1.57079632679;
    double rotate_in_place_threshold_ = 1.0471975512;
    bool rotate_slots_by_goal_yaw_ = false;
    bool align_slots_to_path_ = false;
    bool use_occupancy_grid_planner_ = true;
    bool treat_unknown_as_occupied_ = false;
    bool allow_formation_scaling_ = true;
    double min_formation_scale_ = 0.75;
    double formation_scale_step_ = 0.10;
    double formation_expand_distance_ = 1.0;
    double blocked_recovery_linear_scale_ = 0.6;
    bool allow_blocked_direct_recovery_ = false;
    double blocked_direct_recovery_max_distance_ = 2.5;
    bool use_dynamic_obstacle_models_ = true;
    double dynamic_obstacle_radius_ = 0.40;
    double dynamic_obstacle_replan_distance_ = 0.20;
    std::string map_topic_ = "/map";
    std::string model_states_topic_ = "/gazebo/model_states";
    std::vector<std::string> dynamic_obstacle_model_prefixes_;
    std::vector<std::string> dynamic_obstacle_model_names_;
    std::vector<std::string> ignored_dynamic_obstacle_model_names_;
    int occupied_threshold_ = 65;

    RobotState self_state_;
    std::vector<RobotState> robot_states_;
    std::vector<bool> has_robot_state_;
    bool has_odom_ = false;
    bool has_goal_ = false;
    bool has_map_ = false;
    bool path_dirty_ = true;
    bool path_blocked_ = false;
    bool reached_ = false;
    nav_msgs::OccupancyGrid occupancy_grid_;
    std::vector<Eigen::Vector2d> dynamic_obstacle_positions_;
    Eigen::Vector2d goal_center_ = Eigen::Vector2d::Zero();
    double goal_yaw_ = 0.0;
    std::vector<Eigen::Vector2d> slots_;
    std::vector<int> assigned_slot_indices_;
    bool slot_assignment_ready_ = false;
    double active_formation_scale_ = 1.0;
    std::vector<Eigen::Vector2d> center_path_;
    Eigen::Vector2d planned_start_center_ = Eigen::Vector2d::Zero();
    Eigen::Vector2d planned_goal_center_ = Eigen::Vector2d::Zero();
};

#endif
