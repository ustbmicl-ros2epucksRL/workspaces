#include "plan_manager/formation_navigation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plan_manager
{

TransportApproachResult resolveTransportApproach(
    const Eigen::Vector2d& box_center,
    const Eigen::Vector2d& push_direction,
    const Eigen::Vector2d& nominal_goal,
    const TransportApproachSearch& config,
    const std::function<bool(const Eigen::Vector2d&)>& feasible)
{
    TransportApproachResult result;
    const bool valid_geometry = box_center.allFinite() && push_direction.allFinite() &&
        nominal_goal.allFinite() && std::isfinite(config.minimum_rear_distance) &&
        std::isfinite(config.maximum_rear_distance) &&
        std::isfinite(config.lateral_search_distance) && std::isfinite(config.sample_step) &&
        config.minimum_rear_distance >= 0.0 &&
        config.maximum_rear_distance >= config.minimum_rear_distance &&
        config.lateral_search_distance >= 0.0 && config.sample_step > 0.0 &&
        static_cast<bool>(feasible);
    const double direction_norm = push_direction.norm();
    if (!valid_geometry || direction_norm <= 1e-9) return result;

    const Eigen::Vector2d forward = push_direction / direction_norm;
    const Eigen::Vector2d lateral(-forward.y(), forward.x());
    const int rear_steps = static_cast<int>(std::ceil(
        (config.maximum_rear_distance - config.minimum_rear_distance) /
        config.sample_step));
    const int lateral_steps = static_cast<int>(std::ceil(
        config.lateral_search_distance / config.sample_step));
    double best_score = std::numeric_limits<double>::infinity();
    double best_lateral = std::numeric_limits<double>::infinity();
    double best_rear = std::numeric_limits<double>::infinity();

    for (int rear_index = 0; rear_index <= rear_steps; ++rear_index) {
        const double rear = std::min(
            config.maximum_rear_distance,
            config.minimum_rear_distance + rear_index * config.sample_step);
        for (int lateral_index = -lateral_steps; lateral_index <= lateral_steps;
             ++lateral_index) {
            const double lateral_offset = std::max(
                -config.lateral_search_distance,
                std::min(config.lateral_search_distance,
                         lateral_index * config.sample_step));
            const Eigen::Vector2d candidate =
                box_center - rear * forward + lateral_offset * lateral;
            if (!feasible(candidate)) continue;

            const double score = (candidate - nominal_goal).squaredNorm();
            const double absolute_lateral = std::abs(lateral_offset);
            const bool better_score = score < best_score - 1e-12;
            const bool equal_score = std::abs(score - best_score) <= 1e-12;
            const bool better_tie = equal_score &&
                (absolute_lateral < best_lateral - 1e-12 ||
                 (std::abs(absolute_lateral - best_lateral) <= 1e-12 &&
                  rear < best_rear));
            if (!better_score && !better_tie) continue;
            result.point = candidate;
            result.found = true;
            best_score = score;
            best_lateral = absolute_lateral;
            best_rear = rear;
        }
    }
    return result;
}

NavigationStateMachine::NavigationStateMachine(double map_startup_timeout)
  : map_startup_timeout_(std::max(0.0, map_startup_timeout))
{
}

NavigationDecision NavigationStateMachine::update(
    const NavigationInput& input, double now_seconds)
{
    NavigationDecision result;
    if (!input.has_goal) {
        resetGoal();
        result.state = state_ = NavigationState::STOPPED;
        result.detail = "WAITING_FOR_GOAL";
        return result;
    }
    has_goal_ = true;
    const auto planningFailureDetail = [](NavigationEvent event) -> const char* {
        switch (event) {
        case NavigationEvent::MAP_INVALID: return "MAP_INVALID";
        case NavigationEvent::START_OUTSIDE_MAP: return "START_OUTSIDE_MAP";
        case NavigationEvent::GOAL_OUTSIDE_MAP: return "GOAL_OUTSIDE_MAP";
        case NavigationEvent::START_IN_COLLISION: return "START_IN_COLLISION";
        case NavigationEvent::GOAL_IN_COLLISION: return "GOAL_IN_COLLISION";
        case NavigationEvent::STATIC_NO_PATH: return "NO_STATIC_PATH";
        default: return nullptr;
        }
    };
    if (const char* failure = planningFailureDetail(input.planning_event)) {
        result.state = state_ = NavigationState::FAILED;
        result.detail = failure;
        return result;
    }
    if (state_ == NavigationState::FAILED || state_ == NavigationState::ERROR) {
        result.state = state_;
        result.detail =
            state_ == NavigationState::FAILED ? "PLANNING_FAILED" : "MAP_UNAVAILABLE";
        return result;
    }
    if (!input.motion_enabled) {
        result.state = NavigationState::PAUSED;
        result.detail = "PAUSED";
        return result;
    }
    if (runnable_since_ < 0.0) {
        runnable_since_ = now_seconds;
    }
    if (!input.map_ready) {
        if (now_seconds - runnable_since_ > map_startup_timeout_) {
            result.state = state_ = NavigationState::ERROR;
            result.detail = "MAP_UNAVAILABLE";
        } else {
            result.state = state_ = NavigationState::WAITING_FOR_MAP;
            result.detail = "WAITING_FOR_MAP";
        }
        return result;
    }
    if (!input.odom_ready) {
        result.state = state_ = NavigationState::WAITING_FOR_ODOM;
        result.detail = "WAITING_FOR_ODOM";
        return result;
    }
    if (input.planning_event == NavigationEvent::ARRIVED) {
        result.state = state_ = NavigationState::DONE;
        result.detail = "ARRIVED";
        return result;
    }
    if (input.planning_event == NavigationEvent::PLAN_READY) {
        result.state = state_ = NavigationState::TRACKING;
        result.detail = "TRACKING";
        result.command_motion = true;
        return result;
    }
    if (input.planning_event == NavigationEvent::DYNAMIC_BLOCKED) {
        result.state = NavigationState::BLOCKED_DYNAMIC;
        result.detail = "BLOCKED_DYNAMIC";
        return result;
    }
    if (state_ == NavigationState::TRACKING) {
        result.state = state_;
        result.detail = "TRACKING";
        result.command_motion = true;
        return result;
    }
    result.state = state_ = NavigationState::PLANNING;
    result.detail = "PLANNING";
    result.request_plan = true;
    return result;
}

void NavigationStateMachine::resetGoal()
{
    state_ = NavigationState::STOPPED;
    has_goal_ = false;
    runnable_since_ = -1.0;
    path_progress_ = 0.0;
}

bool NavigationStateMachine::hasGoal() const { return has_goal_; }
void NavigationStateMachine::setPathProgress(double progress)
{
    path_progress_ = std::max(path_progress_, std::max(0.0, progress));
}
double NavigationStateMachine::pathProgress() const { return path_progress_; }

const char* navigationStateName(NavigationState state)
{
    switch (state) {
    case NavigationState::STOPPED: return "STOPPED";
    case NavigationState::PAUSED: return "PAUSED";
    case NavigationState::WAITING_FOR_MAP: return "WAITING_FOR_MAP";
    case NavigationState::WAITING_FOR_ODOM: return "WAITING_FOR_ODOM";
    case NavigationState::PLANNING: return "PLANNING";
    case NavigationState::TRACKING: return "TRACKING";
    case NavigationState::BLOCKED_DYNAMIC: return "BLOCKED_DYNAMIC";
    case NavigationState::DONE: return "DONE";
    case NavigationState::FAILED: return "FAILED";
    case NavigationState::ERROR: return "ERROR";
    }
    return "UNKNOWN";
}

}  // namespace plan_manager
