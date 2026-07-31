#ifndef PLAN_MANAGER_FORMATION_NAVIGATION_H
#define PLAN_MANAGER_FORMATION_NAVIGATION_H

#include <Eigen/Core>

#include <cstdint>
#include <functional>
#include <string>

namespace plan_manager
{

enum class NavigationState
{
    STOPPED,
    PAUSED,
    WAITING_FOR_MAP,
    WAITING_FOR_ODOM,
    PLANNING,
    TRACKING,
    BLOCKED_DYNAMIC,
    DONE,
    FAILED,
    ERROR,
};

enum class NavigationEvent
{
    NONE,
    PLAN_READY,
    MAP_INVALID,
    START_OUTSIDE_MAP,
    GOAL_OUTSIDE_MAP,
    START_IN_COLLISION,
    GOAL_IN_COLLISION,
    STATIC_NO_PATH,
    DYNAMIC_BLOCKED,
    ARRIVED,
};

struct NavigationInput
{
    bool has_goal = false;
    bool motion_enabled = false;
    bool map_ready = false;
    bool odom_ready = false;
    NavigationEvent planning_event = NavigationEvent::NONE;
};

struct NavigationDecision
{
    NavigationState state = NavigationState::STOPPED;
    std::string detail = "STOPPED";
    bool command_motion = false;
    bool request_plan = false;
};

struct TransportApproachSearch
{
    double minimum_rear_distance = 0.0;
    double maximum_rear_distance = 0.0;
    double lateral_search_distance = 0.0;
    double sample_step = 0.0;
};

struct TransportApproachResult
{
    Eigen::Vector2d point = Eigen::Vector2d::Zero();
    bool found = false;
};

TransportApproachResult resolveTransportApproach(
    const Eigen::Vector2d& box_center,
    const Eigen::Vector2d& push_direction,
    const Eigen::Vector2d& nominal_goal,
    const TransportApproachSearch& config,
    const std::function<bool(const Eigen::Vector2d&)>& feasible);

class NavigationStateMachine
{
public:
    explicit NavigationStateMachine(double map_startup_timeout);
    NavigationDecision update(const NavigationInput& input, double now_seconds);
    void resetGoal();
    bool hasGoal() const;
    void setPathProgress(double progress);
    double pathProgress() const;

private:
    double map_startup_timeout_;
    NavigationState state_ = NavigationState::STOPPED;
    bool has_goal_ = false;
    double runnable_since_ = -1.0;
    double path_progress_ = 0.0;
};

const char* navigationStateName(NavigationState state);

}  // namespace plan_manager

#endif
