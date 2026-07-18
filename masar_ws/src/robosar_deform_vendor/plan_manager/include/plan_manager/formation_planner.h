#ifndef PLAN_MANAGER_FORMATION_PLANNER_H
#define PLAN_MANAGER_FORMATION_PLANNER_H

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>

#include <vector>

#include "plan_manager/clearance_grid.h"

namespace plan_manager
{

class ClearanceGrid;

enum class PlanFailure
{
    NONE,
    MAP_INVALID,
    START_OUTSIDE_MAP,
    GOAL_OUTSIDE_MAP,
    START_IN_COLLISION,
    GOAL_IN_COLLISION,
    NO_STATIC_PATH,
};

struct PlannerConfig
{
    double minimum_envelope_radius = 0.0;
    double clearance_cost_weight = 0.0;
    bool treat_unknown_as_occupied = true;
};

struct CenterPlan
{
    std::vector<Eigen::Vector2d> points;
    PlanFailure failure = PlanFailure::NONE;
    double minimum_clearance = 0.0;
};

// Validate only the physical collision footprint before allowing a compact
// formation to release. Inter-robot/wall comfort gaps are recovered by the
// monotone egress path and velocity safety layer; requiring them at the start
// would make a safe release from controlled transport contact impossible.
bool allRobotFootprintsCollisionFree(
    const ClearanceGrid& grid,
    const std::vector<Eigen::Vector2d>& robot_positions,
    double robot_radius);

// A controlled task can finish with a robot touching an obstacle, so its
// conservative circular footprint may overlap map inflation even though the
// measured body center is not in occupied space. This weaker predicate is
// only used to authorize the planner's monotone start-release expansion; the
// path and per-robot velocity safety layers remain active during the egress.
bool allRobotCentersOutsideOccupiedSpace(
    const ClearanceGrid& grid,
    const std::vector<Eigen::Vector2d>& robot_positions);

class FormationPlanner
{
public:
    explicit FormationPlanner(PlannerConfig config);

    CenterPlan plan(
        const nav_msgs::OccupancyGrid& map,
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& goal,
        bool allow_validated_start_release = false,
        const std::vector<CircularMapObstacle>& dynamic_obstacles = {}) const;

private:
    PlannerConfig config_;
};

}  // namespace plan_manager

#endif
