#include "plan_manager/formation_assignment.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace plan_manager
{

Eigen::Vector2d rotateSlot(const Eigen::Vector2d& slot, double yaw)
{
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Eigen::Vector2d(c * slot.x() - s * slot.y(), s * slot.x() + c * slot.y());
}

std::vector<int> assignSlotsByCurrentGeometry(
    const std::vector<Eigen::Vector2d>& robot_positions,
    const std::vector<bool>& has_robot_position,
    const std::vector<Eigen::Vector2d>& slots,
    double yaw,
    const std::vector<int>& fallback_assignment)
{
    const std::size_t robot_count = robot_positions.size();
    if (robot_count == 0 ||
        has_robot_position.size() != robot_count ||
        slots.size() != robot_count ||
        fallback_assignment.size() != robot_count) {
        return fallback_assignment;
    }

    for (bool has_position : has_robot_position) {
        if (!has_position) {
            return fallback_assignment;
        }
    }

    Eigen::Vector2d center = Eigen::Vector2d::Zero();
    for (const Eigen::Vector2d& position : robot_positions) {
        center += position;
    }
    center /= static_cast<double>(robot_count);

    std::vector<Eigen::Vector2d> rotated_slots;
    rotated_slots.reserve(robot_count);
    for (const Eigen::Vector2d& slot : slots) {
        rotated_slots.push_back(rotateSlot(slot, yaw));
    }

    std::vector<int> candidate(robot_count);
    std::iota(candidate.begin(), candidate.end(), 0);
    std::vector<int> best = candidate;
    double best_cost = std::numeric_limits<double>::infinity();

    do {
        double cost = 0.0;
        for (std::size_t robot_index = 0; robot_index < robot_count; ++robot_index) {
            const Eigen::Vector2d target = center + rotated_slots[static_cast<std::size_t>(candidate[robot_index])];
            cost += (robot_positions[robot_index] - target).squaredNorm();
        }
        if (cost < best_cost) {
            best_cost = cost;
            best = candidate;
        }
    } while (std::next_permutation(candidate.begin(), candidate.end()));

    return best;
}

}  // namespace plan_manager
