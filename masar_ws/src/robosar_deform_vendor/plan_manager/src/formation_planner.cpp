#include "plan_manager/formation_planner.h"

#include "plan_manager/clearance_grid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace plan_manager
{
namespace
{

struct OpenNode
{
    double estimated_total;
    double cost;
    std::size_t index;
};

struct WorseOpenNode
{
    bool operator()(const OpenNode& left, const OpenNode& right) const
    {
        if (std::abs(left.estimated_total - right.estimated_total) > 1e-12) {
            return left.estimated_total > right.estimated_total;
        }
        if (std::abs(left.cost - right.cost) > 1e-12) {
            return left.cost > right.cost;
        }
        return left.index > right.index;
    }
};

double traversalMultiplier(
    double clearance,
    double resolution,
    const PlannerConfig& config)
{
    const double gap = std::max(
        clearance - std::max(0.0, config.minimum_envelope_radius),
        resolution);
    return 1.0 + std::max(0.0, config.clearance_cost_weight) / (gap * gap);
}

double dynamicTraversalPenalty(
    const Eigen::Vector2d& point,
    double resolution,
    const PlannerConfig& config,
    const std::vector<CircularMapObstacle>& dynamic_obstacles)
{
    double penalty = 0.0;
    for (const CircularMapObstacle& obstacle : dynamic_obstacles) {
        if (!obstacle.center.allFinite() || !std::isfinite(obstacle.radius) ||
            obstacle.radius <= 0.0) {
            continue;
        }
        const double edge_distance =
            (point - obstacle.center).norm() - obstacle.radius;
        const double gap = std::max(edge_distance, resolution);
        penalty += std::max(0.0, config.clearance_cost_weight) / (gap * gap);
    }
    return penalty;
}

double segmentCost(
    const ClearanceGrid& grid,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    const PlannerConfig& config,
    const std::vector<CircularMapObstacle>& dynamic_obstacles)
{
    const double distance = (end - start).norm();
    if (distance <= 1e-12) {
        return 0.0;
    }
    const double spacing = std::max(1e-6, 0.5 * grid.resolution());
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
    const double ds = distance / static_cast<double>(steps);
    double cost = 0.0;
    for (int step = 1; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const Eigen::Vector2d point = start + ratio * (end - start);
        cost += ds * (traversalMultiplier(
            grid.clearanceAtWorld(point), grid.resolution(), config) +
            dynamicTraversalPenalty(
                point, grid.resolution(), config, dynamic_obstacles));
    }
    return cost;
}

double polylineCost(
    const ClearanceGrid& grid,
    const std::vector<Eigen::Vector2d>& points,
    std::size_t first,
    std::size_t last,
    const PlannerConfig& config,
    const std::vector<CircularMapObstacle>& dynamic_obstacles)
{
    double cost = 0.0;
    for (std::size_t index = first + 1; index <= last; ++index) {
        cost += segmentCost(
            grid, points[index - 1], points[index], config, dynamic_obstacles);
    }
    return cost;
}

std::vector<Eigen::Vector2d> smoothPath(
    const ClearanceGrid& grid,
    const std::vector<Eigen::Vector2d>& raw,
    const PlannerConfig& config,
    const std::vector<CircularMapObstacle>& dynamic_obstacles)
{
    if (raw.size() <= 2) {
        return raw;
    }
    std::vector<Eigen::Vector2d> smoothed;
    smoothed.push_back(raw.front());
    std::size_t anchor = 0;
    while (anchor + 1 < raw.size()) {
        std::size_t chosen = anchor + 1;
        for (std::size_t candidate = raw.size() - 1; candidate > anchor + 1; --candidate) {
            if (!grid.segmentFeasible(
                    raw[anchor], raw[candidate], config.minimum_envelope_radius)) {
                continue;
            }
            const double direct = segmentCost(
                grid, raw[anchor], raw[candidate], config, dynamic_obstacles);
            const double original = polylineCost(
                grid, raw, anchor, candidate, config, dynamic_obstacles);
            if (direct <= original * 1.02 + 1e-9) {
                chosen = candidate;
                break;
            }
        }
        smoothed.push_back(raw[chosen]);
        anchor = chosen;
    }
    return smoothed;
}

}  // namespace

FormationPlanner::FormationPlanner(PlannerConfig config) : config_(config)
{
    config_.minimum_envelope_radius = std::max(0.0, config_.minimum_envelope_radius);
    config_.clearance_cost_weight = std::max(0.0, config_.clearance_cost_weight);
}

bool allRobotFootprintsCollisionFree(
    const ClearanceGrid& grid,
    const std::vector<Eigen::Vector2d>& robot_positions,
    double robot_radius)
{
    const double required_clearance = std::max(0.0, robot_radius);
    if (!grid.valid() || robot_positions.empty()) {
        return false;
    }
    for (const Eigen::Vector2d& position : robot_positions) {
        if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
            grid.clearanceAtWorld(position) + 1e-12 < required_clearance) {
            return false;
        }
    }
    return true;
}

bool allRobotCentersOutsideOccupiedSpace(
    const ClearanceGrid& grid,
    const std::vector<Eigen::Vector2d>& robot_positions)
{
    if (!grid.valid() || robot_positions.empty()) {
        return false;
    }
    for (const Eigen::Vector2d& position : robot_positions) {
        if (!std::isfinite(position.x()) || !std::isfinite(position.y()) ||
            grid.clearanceAtWorld(position) <= 0.0) {
            return false;
        }
    }
    return true;
}

CenterPlan FormationPlanner::plan(
    const nav_msgs::OccupancyGrid& map,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& goal,
    bool allow_validated_start_release,
    const std::vector<CircularMapObstacle>& dynamic_obstacles) const
{
    CenterPlan result;
    const ClearanceGrid grid(map, config_.treat_unknown_as_occupied);
    if (!grid.valid()) {
        result.failure = PlanFailure::MAP_INVALID;
        return result;
    }

    int start_x = 0;
    int start_y = 0;
    int goal_x = 0;
    int goal_y = 0;
    if (!grid.worldToMap(start, &start_x, &start_y)) {
        result.failure = PlanFailure::START_OUTSIDE_MAP;
        return result;
    }
    if (!grid.worldToMap(goal, &goal_x, &goal_y)) {
        result.failure = PlanFailure::GOAL_OUTSIDE_MAP;
        return result;
    }
    const bool start_feasible =
        grid.isFeasible(start_x, start_y, config_.minimum_envelope_radius);
    const double start_clearance = grid.clearance(start_x, start_y);
    // A task controller can leave the centroid one map cell inside the
    // conservative formation envelope even though every robot and the center
    // itself are collision-free. Permit only this marginal, discretization-
    // sized condition to move monotonically toward higher clearance. A start
    // that is occupied or deeper inside inflation remains a hard failure.
    const bool marginal_start_release = !start_feasible &&
        start_clearance > 0.0 &&
        start_clearance + grid.resolution() + 1e-12 >=
            config_.minimum_envelope_radius;
    const bool validated_start_release = !start_feasible &&
        allow_validated_start_release && start_clearance > 0.0;
    const bool start_release = marginal_start_release || validated_start_release;
    if (!start_feasible && !start_release) {
        result.failure = PlanFailure::START_IN_COLLISION;
        return result;
    }
    if (!grid.isFeasible(goal_x, goal_y, config_.minimum_envelope_radius)) {
        result.failure = PlanFailure::GOAL_IN_COLLISION;
        return result;
    }

    const int width = grid.width();
    const int height = grid.height();
    const std::size_t cell_count = static_cast<std::size_t>(width * height);
    const auto flatIndex = [width](int x, int y) {
        return static_cast<std::size_t>(y * width + x);
    };
    const std::size_t start_index = flatIndex(start_x, start_y);
    const std::size_t goal_index = flatIndex(goal_x, goal_y);
    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> costs(cell_count, infinity);
    std::vector<std::size_t> parents(cell_count, cell_count);
    std::vector<bool> closed(cell_count, false);
    std::priority_queue<OpenNode, std::vector<OpenNode>, WorseOpenNode> open;
    costs[start_index] = 0.0;
    open.push(OpenNode{(goal - start).norm(), 0.0, start_index});

    const int directions[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };
    const auto traversable = [&grid, &config = config_, start_release]
                             (int from_x, int from_y, int to_x, int to_y) {
        if (grid.isFeasible(to_x, to_y, config.minimum_envelope_radius)) {
            return true;
        }
        if (!start_release ||
            grid.isFeasible(from_x, from_y, config.minimum_envelope_radius)) {
            return false;
        }
        const double clearance = grid.clearance(to_x, to_y);
        return clearance > 0.0 &&
            clearance + 0.5 * grid.resolution() + 1e-12 >=
                grid.clearance(from_x, from_y);
    };
    while (!open.empty()) {
        const OpenNode current = open.top();
        open.pop();
        if (closed[current.index] || current.cost > costs[current.index] + 1e-12) {
            continue;
        }
        closed[current.index] = true;
        if (current.index == goal_index) {
            break;
        }
        const int x = static_cast<int>(current.index % static_cast<std::size_t>(width));
        const int y = static_cast<int>(current.index / static_cast<std::size_t>(width));
        for (const auto& direction : directions) {
            const int nx = x + direction[0];
            const int ny = y + direction[1];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height ||
                !traversable(x, y, nx, ny)) {
                continue;
            }
            const bool diagonal = direction[0] != 0 && direction[1] != 0;
            if (diagonal &&
                (!traversable(x, y, x + direction[0], y) ||
                 !traversable(x, y, x, y + direction[1]))) {
                continue;
            }
            const double step = grid.resolution() * (diagonal ? std::sqrt(2.0) : 1.0);
            const Eigen::Vector2d next_world = grid.mapToWorld(nx, ny);
            const double next_cost = costs[current.index] + step * (
                traversalMultiplier(
                    grid.clearance(nx, ny), grid.resolution(), config_) +
                dynamicTraversalPenalty(
                    next_world, grid.resolution(), config_, dynamic_obstacles));
            const std::size_t next_index = flatIndex(nx, ny);
            if (next_cost + 1e-12 >= costs[next_index]) {
                continue;
            }
            costs[next_index] = next_cost;
            parents[next_index] = current.index;
            open.push(OpenNode{
                next_cost + (goal - next_world).norm(), next_cost, next_index});
        }
    }

    if (!closed[goal_index]) {
        result.failure = PlanFailure::NO_STATIC_PATH;
        return result;
    }

    std::vector<Eigen::Vector2d> reverse_cells;
    std::size_t cursor = goal_index;
    while (true) {
        const int x = static_cast<int>(cursor % static_cast<std::size_t>(width));
        const int y = static_cast<int>(cursor / static_cast<std::size_t>(width));
        reverse_cells.push_back(grid.mapToWorld(x, y));
        if (cursor == start_index) {
            break;
        }
        cursor = parents[cursor];
        if (cursor >= cell_count) {
            result.failure = PlanFailure::NO_STATIC_PATH;
            return result;
        }
    }
    std::reverse(reverse_cells.begin(), reverse_cells.end());

    std::vector<Eigen::Vector2d> raw;
    raw.reserve(reverse_cells.size() + 2);
    raw.push_back(start);
    for (std::size_t index = 1; index + 1 < reverse_cells.size(); ++index) {
        raw.push_back(reverse_cells[index]);
    }
    if ((goal - raw.back()).norm() > 1e-12) {
        raw.push_back(goal);
    }
    result.points = smoothPath(grid, raw, config_, dynamic_obstacles);
    result.minimum_clearance = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector2d& point : result.points) {
        result.minimum_clearance = std::min(
            result.minimum_clearance, grid.clearanceAtWorld(point));
    }
    if (!std::isfinite(result.minimum_clearance)) {
        result.minimum_clearance = 0.0;
    }
    result.failure = PlanFailure::NONE;
    return result;
}

}  // namespace plan_manager
