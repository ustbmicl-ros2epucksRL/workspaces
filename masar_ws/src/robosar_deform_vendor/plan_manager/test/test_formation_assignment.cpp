#include "plan_manager/formation_assignment.h"

#include <iostream>
#include <string>

namespace
{

bool expectAssignment(const std::string& name,
                      const std::vector<int>& actual,
                      const std::vector<int>& expected)
{
    if (actual == expected) {
        return true;
    }
    std::cerr << name << " failed. expected:";
    for (int value : expected) {
        std::cerr << " " << value;
    }
    std::cerr << " actual:";
    for (int value : actual) {
        std::cerr << " " << value;
    }
    std::cerr << std::endl;
    return false;
}

bool responseDemoInitialSquareKeepsCurrentCorners()
{
    const std::vector<Eigen::Vector2d> slots{
        Eigen::Vector2d(0.3, 0.3),
        Eigen::Vector2d(0.3, -0.3),
        Eigen::Vector2d(-0.3, -0.3),
        Eigen::Vector2d(-0.3, 0.3),
    };
    const std::vector<Eigen::Vector2d> robots{
        Eigen::Vector2d(4.2, -4.6),
        Eigen::Vector2d(4.8, -4.6),
        Eigen::Vector2d(4.2, -5.2),
        Eigen::Vector2d(4.8, -5.2),
    };
    const std::vector<bool> has_robot_position{true, true, true, true};
    const std::vector<int> fallback{0, 1, 2, 3};

    const std::vector<int> assignment = plan_manager::assignSlotsByCurrentGeometry(
        robots,
        has_robot_position,
        slots,
        0.0,
        fallback);

    return expectAssignment(
        "responseDemoInitialSquareKeepsCurrentCorners",
        assignment,
        std::vector<int>{3, 0, 2, 1});
}

bool missingRobotPoseFallsBackToRobotIdOrder()
{
    const std::vector<Eigen::Vector2d> slots{
        Eigen::Vector2d(0.3, 0.3),
        Eigen::Vector2d(0.3, -0.3),
        Eigen::Vector2d(-0.3, -0.3),
        Eigen::Vector2d(-0.3, 0.3),
    };
    const std::vector<Eigen::Vector2d> robots(4, Eigen::Vector2d::Zero());
    const std::vector<bool> has_robot_position{true, true, false, true};
    const std::vector<int> fallback{0, 1, 2, 3};

    const std::vector<int> assignment = plan_manager::assignSlotsByCurrentGeometry(
        robots,
        has_robot_position,
        slots,
        0.0,
        fallback);

    return expectAssignment("missingRobotPoseFallsBackToRobotIdOrder", assignment, fallback);
}

}  // namespace

int main()
{
    bool ok = true;
    ok = responseDemoInitialSquareKeepsCurrentCorners() && ok;
    ok = missingRobotPoseFallsBackToRobotIdOrder() && ok;
    return ok ? 0 : 1;
}
