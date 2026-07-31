#ifndef PLAN_MANAGER_DYNAMIC_OBSTACLE_PLANNING_H
#define PLAN_MANAGER_DYNAMIC_OBSTACLE_PLANNING_H

#include <Eigen/Core>

#include <string>
#include <vector>

#include "plan_manager/clearance_grid.h"

namespace plan_manager
{

struct MovingDisc
{
    std::string name;
    Eigen::Vector2d position = Eigen::Vector2d::Zero();
    Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
    double radius = 0.0;
};

// Convert moving robots into a short swept occupancy prediction. Sampling is
// derived from map resolution rather than from a fixed list of formation sizes.
std::vector<CircularMapObstacle> sweptCircularObstacles(
    const std::vector<MovingDisc>& obstacles,
    double prediction_horizon,
    double sample_spacing);

}  // namespace plan_manager

#endif
