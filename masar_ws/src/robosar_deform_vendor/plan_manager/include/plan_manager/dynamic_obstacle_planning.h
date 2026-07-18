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

// True when a robot snapshot moved enough that a previously planned dynamic
// occupancy overlay is stale. Names make the comparison stable across ordering.
bool movingObstacleSnapshotChanged(
    const std::vector<MovingDisc>& current,
    const std::vector<MovingDisc>& planned,
    double distance_threshold);

// Dynamic traffic is a time-varying planning cost, so material motion can make
// either a blocked direct route available or the current route unsafe. Refresh
// the center route in both cases instead of only reacting to new intersections.
bool dynamicRouteNeedsRefresh(
    const std::vector<MovingDisc>& current,
    const std::vector<MovingDisc>& planned,
    double distance_threshold);

// Test only the untravelled portion of a center path. Obstacles behind the
// formation must not cause endless replanning after the group has passed them.
bool remainingPathIntersectsObstacles(
    const std::vector<Eigen::Vector2d>& path,
    const Eigen::Vector2d& current_center,
    const std::vector<CircularMapObstacle>& obstacles,
    double center_clearance);

}  // namespace plan_manager

#endif
