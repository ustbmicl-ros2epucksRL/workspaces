#include "plan_manager/dynamic_obstacle_planning.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace plan_manager
{
namespace
{

double pointSegmentDistance(
    const Eigen::Vector2d& point,
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end)
{
    const Eigen::Vector2d segment = end - start;
    const double length_squared = segment.squaredNorm();
    if (length_squared <= 1e-12) return (point - start).norm();
    const double fraction = std::max(
        0.0, std::min(1.0, (point - start).dot(segment) / length_squared));
    return (point - (start + fraction * segment)).norm();
}

}  // namespace

std::vector<CircularMapObstacle> sweptCircularObstacles(
    const std::vector<MovingDisc>& obstacles,
    double prediction_horizon,
    double sample_spacing)
{
    std::vector<CircularMapObstacle> result;
    const double horizon = std::max(0.0, prediction_horizon);
    const double spacing = std::max(1e-6, sample_spacing);
    for (const MovingDisc& obstacle : obstacles) {
        if (!obstacle.position.allFinite() || !obstacle.velocity.allFinite() ||
            !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0) {
            continue;
        }
        const double travel = obstacle.velocity.norm() * horizon;
        const int intervals = static_cast<int>(std::ceil(travel / spacing));
        const int samples = std::max(1, intervals + 1);
        for (int index = 0; index < samples; ++index) {
            const double fraction = samples == 1
                ? 0.0 : static_cast<double>(index) / static_cast<double>(samples - 1);
            result.push_back({
                obstacle.position + fraction * horizon * obstacle.velocity,
                obstacle.radius,
            });
        }
    }
    return result;
}

bool movingObstacleSnapshotChanged(
    const std::vector<MovingDisc>& current,
    const std::vector<MovingDisc>& planned,
    double distance_threshold)
{
    if (current.size() != planned.size()) return true;
    std::unordered_map<std::string, Eigen::Vector2d> reference;
    for (const MovingDisc& obstacle : planned) {
        if (!reference.emplace(obstacle.name, obstacle.position).second) return true;
    }
    const double threshold = std::max(0.0, distance_threshold);
    for (const MovingDisc& obstacle : current) {
        const auto found = reference.find(obstacle.name);
        if (found == reference.end() ||
            (obstacle.position - found->second).norm() > threshold) {
            return true;
        }
        reference.erase(found);
    }
    return !reference.empty();
}

bool dynamicRouteNeedsRefresh(
    const std::vector<MovingDisc>& current,
    const std::vector<MovingDisc>& planned,
    double distance_threshold)
{
    return movingObstacleSnapshotChanged(current, planned, distance_threshold);
}

bool remainingPathIntersectsObstacles(
    const std::vector<Eigen::Vector2d>& path,
    const Eigen::Vector2d& current_center,
    const std::vector<CircularMapObstacle>& obstacles,
    double center_clearance)
{
    if (path.empty() || obstacles.empty() || !current_center.allFinite()) return false;
    if (path.size() == 1) {
        for (const CircularMapObstacle& obstacle : obstacles) {
            if ((path.front() - obstacle.center).norm() <=
                std::max(0.0, center_clearance) + std::max(0.0, obstacle.radius)) {
                return true;
            }
        }
        return false;
    }

    std::size_t closest_segment = 0;
    double closest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
        const double distance = pointSegmentDistance(
            current_center, path[index], path[index + 1]);
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_segment = index;
        }
    }

    const double clearance = std::max(0.0, center_clearance);
    for (const CircularMapObstacle& obstacle : obstacles) {
        if (!obstacle.center.allFinite() || !std::isfinite(obstacle.radius) ||
            obstacle.radius <= 0.0) {
            continue;
        }
        const double required = clearance + obstacle.radius;
        for (std::size_t index = closest_segment; index + 1 < path.size(); ++index) {
            if (pointSegmentDistance(
                    obstacle.center, path[index], path[index + 1]) <= required) {
                return true;
            }
        }
    }
    return false;
}

}  // namespace plan_manager
