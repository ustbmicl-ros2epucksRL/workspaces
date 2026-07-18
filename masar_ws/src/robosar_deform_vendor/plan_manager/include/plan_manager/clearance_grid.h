#ifndef PLAN_MANAGER_CLEARANCE_GRID_H
#define PLAN_MANAGER_CLEARANCE_GRID_H

#include <Eigen/Core>
#include <nav_msgs/OccupancyGrid.h>

#include <string>
#include <vector>

namespace plan_manager
{

struct CircularMapObstacle
{
    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    double radius = 0.0;
};

// Returns a copy of map with the physical footprint of each obstacle marked
// occupied. Formation/robot inflation remains the planner's responsibility.
nav_msgs::OccupancyGrid overlayCircularObstacles(
    const nav_msgs::OccupancyGrid& map,
    const std::vector<CircularMapObstacle>& obstacles);

class ClearanceGrid
{
public:
    explicit ClearanceGrid(
        const nav_msgs::OccupancyGrid& map,
        bool treat_unknown_as_occupied = true);

    bool valid() const;
    const std::string& error() const;
    int width() const;
    int height() const;
    double resolution() const;

    bool worldToMap(const Eigen::Vector2d& world, int* map_x, int* map_y) const;
    Eigen::Vector2d mapToWorld(int map_x, int map_y) const;
    double clearance(int map_x, int map_y) const;
    double clearanceAtWorld(const Eigen::Vector2d& world) const;
    bool isFeasible(int map_x, int map_y, double envelope_radius) const;
    bool segmentFeasible(
        const Eigen::Vector2d& start,
        const Eigen::Vector2d& end,
        double envelope_radius) const;

private:
    bool inside(int map_x, int map_y) const;
    std::size_t index(int map_x, int map_y) const;
    bool validate();
    void buildDistanceField();

    nav_msgs::OccupancyGrid map_;
    bool treat_unknown_as_occupied_;
    bool valid_ = false;
    std::string error_;
    double origin_yaw_ = 0.0;
    std::vector<double> clearance_meters_;
};

}  // namespace plan_manager

#endif
