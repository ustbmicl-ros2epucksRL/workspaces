#ifndef PLAN_MANAGER_FORMATION_ASSIGNMENT_H
#define PLAN_MANAGER_FORMATION_ASSIGNMENT_H

#include <Eigen/Eigen>

#include <vector>

namespace plan_manager
{

Eigen::Vector2d rotateSlot(const Eigen::Vector2d& slot, double yaw);

std::vector<int> assignSlotsByCurrentGeometry(
    const std::vector<Eigen::Vector2d>& robot_positions,
    const std::vector<bool>& has_robot_position,
    const std::vector<Eigen::Vector2d>& slots,
    double yaw,
    const std::vector<int>& fallback_assignment);

}  // namespace plan_manager

#endif
