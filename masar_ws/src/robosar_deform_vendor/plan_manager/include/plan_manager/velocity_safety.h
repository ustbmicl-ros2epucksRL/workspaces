#ifndef PLAN_MANAGER_VELOCITY_SAFETY_H
#define PLAN_MANAGER_VELOCITY_SAFETY_H

#include <Eigen/Core>
#include <ros/time.h>

#include <vector>

namespace plan_manager
{

struct DiscObstacle
{
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double radius = 0.0;
    ros::Time stamp;
    // Robot obstacles can actively change course and cannot be handled as
    // instantaneous holonomic points. Prioritize early radial separation.
    bool requires_separation_response = false;
};

struct VelocitySafetyConfig
{
    double robot_radius = 0.0;
    double safety_gap = 0.0;
    double time_horizon = 0.0;
    double max_speed = 0.0;
    double obstacle_timeout = 0.0;
};

struct SafeVelocityResult
{
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double minimum_time_to_collision = 0.0;
    bool blocked = false;
};

double predictedTimeToCollision(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& candidate_velocity,
    const DiscObstacle& obstacle,
    const VelocitySafetyConfig& config);

SafeVelocityResult projectSafeVelocity(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& current_velocity,
    const Eigen::Vector2d& preferred_velocity,
    const std::vector<DiscObstacle>& obstacles,
    const VelocitySafetyConfig& config,
    const ros::Time& now);

}  // namespace plan_manager

#endif
