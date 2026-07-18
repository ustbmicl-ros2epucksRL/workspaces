#include "plan_manager/clearance_grid.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>

namespace plan_manager
{
namespace
{

constexpr int kOccupiedThreshold = 50;

bool finite(double value)
{
    return std::isfinite(value);
}

}  // namespace

nav_msgs::OccupancyGrid overlayCircularObstacles(
    const nav_msgs::OccupancyGrid& map,
    const std::vector<CircularMapObstacle>& obstacles)
{
    nav_msgs::OccupancyGrid output = map;
    const ClearanceGrid coordinates(map, true);
    if (!coordinates.valid() || obstacles.empty()) {
        return output;
    }
    for (int y = 0; y < coordinates.height(); ++y) {
        for (int x = 0; x < coordinates.width(); ++x) {
            const Eigen::Vector2d world = coordinates.mapToWorld(x, y);
            bool occupied = false;
            for (const CircularMapObstacle& obstacle : obstacles) {
                if (!finite(obstacle.center.x()) || !finite(obstacle.center.y()) ||
                    !finite(obstacle.radius) || obstacle.radius <= 0.0) {
                    continue;
                }
                if ((world - obstacle.center).squaredNorm() <=
                    obstacle.radius * obstacle.radius + 1e-12) {
                    occupied = true;
                    break;
                }
            }
            if (occupied) {
                output.data[static_cast<std::size_t>(
                    y * coordinates.width() + x)] = 100;
            }
        }
    }
    return output;
}

ClearanceGrid::ClearanceGrid(
    const nav_msgs::OccupancyGrid& map,
    bool treat_unknown_as_occupied)
    : map_(map), treat_unknown_as_occupied_(treat_unknown_as_occupied)
{
    valid_ = validate();
    if (valid_) {
        buildDistanceField();
    }
}

bool ClearanceGrid::valid() const
{
    return valid_;
}

const std::string& ClearanceGrid::error() const
{
    return error_;
}

int ClearanceGrid::width() const
{
    return static_cast<int>(map_.info.width);
}

int ClearanceGrid::height() const
{
    return static_cast<int>(map_.info.height);
}

double ClearanceGrid::resolution() const
{
    return static_cast<double>(map_.info.resolution);
}

bool ClearanceGrid::inside(int map_x, int map_y) const
{
    return map_x >= 0 && map_y >= 0 && map_x < width() && map_y < height();
}

std::size_t ClearanceGrid::index(int map_x, int map_y) const
{
    return static_cast<std::size_t>(map_y * width() + map_x);
}

bool ClearanceGrid::validate()
{
    if (map_.info.width == 0 || map_.info.height == 0) {
        error_ = "map dimensions must be non-zero";
        return false;
    }
    if (!finite(map_.info.resolution) || map_.info.resolution <= 0.0) {
        error_ = "map resolution must be finite and positive";
        return false;
    }
    const std::size_t expected =
        static_cast<std::size_t>(map_.info.width) * static_cast<std::size_t>(map_.info.height);
    if (map_.data.size() != expected) {
        error_ = "map data length does not match width*height";
        return false;
    }

    const geometry_msgs::Pose& origin = map_.info.origin;
    const geometry_msgs::Quaternion& q = origin.orientation;
    if (!finite(origin.position.x) || !finite(origin.position.y) ||
        !finite(q.x) || !finite(q.y) || !finite(q.z) || !finite(q.w)) {
        error_ = "map origin contains non-finite values";
        return false;
    }
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (norm < 1e-9 || std::abs(q.x / norm) > 1e-6 || std::abs(q.y / norm) > 1e-6) {
        error_ = "map origin must contain a valid planar quaternion";
        return false;
    }
    const double z = q.z / norm;
    const double w = q.w / norm;
    origin_yaw_ = std::atan2(2.0 * w * z, 1.0 - 2.0 * z * z);
    return true;
}

void ClearanceGrid::buildDistanceField()
{
    const double infinity = std::numeric_limits<double>::infinity();
    clearance_meters_.assign(map_.data.size(), infinity);

    using QueueItem = std::pair<double, std::size_t>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    for (int y = 0; y < height(); ++y) {
        for (int x = 0; x < width(); ++x) {
            const std::size_t cell = index(x, y);
            const std::int8_t value = map_.data[cell];
            const bool occupied = value >= kOccupiedThreshold ||
                (value < 0 && treat_unknown_as_occupied_);
            if (occupied) {
                clearance_meters_[cell] = 0.0;
                queue.emplace(0.0, cell);
            }
        }
    }

    const int directions[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };
    while (!queue.empty()) {
        const QueueItem current = queue.top();
        queue.pop();
        if (current.first > clearance_meters_[current.second] + 1e-12) {
            continue;
        }
        const int x = static_cast<int>(current.second % static_cast<std::size_t>(width()));
        const int y = static_cast<int>(current.second / static_cast<std::size_t>(width()));
        for (const auto& direction : directions) {
            const int nx = x + direction[0];
            const int ny = y + direction[1];
            if (!inside(nx, ny)) {
                continue;
            }
            const bool diagonal = direction[0] != 0 && direction[1] != 0;
            const double step = resolution() * (diagonal ? std::sqrt(2.0) : 1.0);
            const double candidate = current.first + step;
            const std::size_t next = index(nx, ny);
            if (candidate + 1e-12 < clearance_meters_[next]) {
                clearance_meters_[next] = candidate;
                queue.emplace(candidate, next);
            }
        }
    }

    const double occupied_half_diagonal = 0.5 * std::sqrt(2.0) * resolution();
    for (int y = 0; y < height(); ++y) {
        for (int x = 0; x < width(); ++x) {
            const std::size_t cell = index(x, y);
            double obstacle_clearance = clearance_meters_[cell];
            if (std::isfinite(obstacle_clearance) && obstacle_clearance > 0.0) {
                obstacle_clearance = std::max(0.0, obstacle_clearance - occupied_half_diagonal);
            }
            const double boundary_clearance = resolution() * std::min(
                std::min(static_cast<double>(x) + 0.5, static_cast<double>(width() - x) - 0.5),
                std::min(static_cast<double>(y) + 0.5, static_cast<double>(height() - y) - 0.5));
            clearance_meters_[cell] = std::min(obstacle_clearance, boundary_clearance);
        }
    }
}

bool ClearanceGrid::worldToMap(const Eigen::Vector2d& world, int* map_x, int* map_y) const
{
    if (!valid_ || map_x == nullptr || map_y == nullptr ||
        !finite(world.x()) || !finite(world.y())) {
        return false;
    }
    const double dx = world.x() - map_.info.origin.position.x;
    const double dy = world.y() - map_.info.origin.position.y;
    const double c = std::cos(origin_yaw_);
    const double s = std::sin(origin_yaw_);
    const double local_x = c * dx + s * dy;
    const double local_y = -s * dx + c * dy;
    const int x = static_cast<int>(std::floor(local_x / resolution()));
    const int y = static_cast<int>(std::floor(local_y / resolution()));
    if (!inside(x, y)) {
        return false;
    }
    *map_x = x;
    *map_y = y;
    return true;
}

Eigen::Vector2d ClearanceGrid::mapToWorld(int map_x, int map_y) const
{
    if (!valid_ || !inside(map_x, map_y)) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        return Eigen::Vector2d(nan, nan);
    }
    const double local_x = (static_cast<double>(map_x) + 0.5) * resolution();
    const double local_y = (static_cast<double>(map_y) + 0.5) * resolution();
    const double c = std::cos(origin_yaw_);
    const double s = std::sin(origin_yaw_);
    return Eigen::Vector2d(
        map_.info.origin.position.x + c * local_x - s * local_y,
        map_.info.origin.position.y + s * local_x + c * local_y);
}

double ClearanceGrid::clearance(int map_x, int map_y) const
{
    if (!valid_ || !inside(map_x, map_y)) {
        return 0.0;
    }
    return clearance_meters_[index(map_x, map_y)];
}

double ClearanceGrid::clearanceAtWorld(const Eigen::Vector2d& world) const
{
    int map_x = 0;
    int map_y = 0;
    return worldToMap(world, &map_x, &map_y) ? clearance(map_x, map_y) : 0.0;
}

bool ClearanceGrid::isFeasible(int map_x, int map_y, double envelope_radius) const
{
    return valid_ && inside(map_x, map_y) &&
        clearance(map_x, map_y) + 1e-12 >= std::max(0.0, envelope_radius);
}

bool ClearanceGrid::segmentFeasible(
    const Eigen::Vector2d& start,
    const Eigen::Vector2d& end,
    double envelope_radius) const
{
    if (!valid_) {
        return false;
    }
    const double distance = (end - start).norm();
    const double sample_spacing = std::max(1e-6, 0.5 * resolution());
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / sample_spacing)));
    int previous_x = 0;
    int previous_y = 0;
    bool have_previous = false;
    for (int step = 0; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        int map_x = 0;
        int map_y = 0;
        if (!worldToMap(start + ratio * (end - start), &map_x, &map_y) ||
            !isFeasible(map_x, map_y, envelope_radius)) {
            return false;
        }
        if (have_previous && map_x != previous_x && map_y != previous_y &&
            (!isFeasible(map_x, previous_y, envelope_radius) ||
             !isFeasible(previous_x, map_y, envelope_radius))) {
            return false;
        }
        previous_x = map_x;
        previous_y = map_y;
        have_previous = true;
    }
    return true;
}

}  // namespace plan_manager
