#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include "plan_manager/clearance_grid.h"
#include "plan_manager/formation_planner.h"

namespace
{

nav_msgs::OccupancyGrid makeMap(int width, int height, double resolution = 1.0)
{
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.width = static_cast<std::uint32_t>(width);
    map.info.height = static_cast<std::uint32_t>(height);
    map.info.resolution = resolution;
    map.info.origin.orientation.w = 1.0;
    map.data.assign(static_cast<std::size_t>(width * height), 0);
    return map;
}

void occupy(nav_msgs::OccupancyGrid* map, int x, int y)
{
    map->data[static_cast<std::size_t>(y * static_cast<int>(map->info.width) + x)] = 100;
}

plan_manager::FormationPlanner planner(double envelope = 0.2, double clearance_weight = 2.0)
{
    return plan_manager::FormationPlanner(
        plan_manager::PlannerConfig{envelope, clearance_weight, true});
}

}  // namespace

TEST(FormationPlanner, RejectsMissingMapInsteadOfReturningDirectLine)
{
    const nav_msgs::OccupancyGrid map;
    const plan_manager::CenterPlan result = planner().plan(
        map, Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(4.0, 4.0));

    EXPECT_EQ(result.failure, plan_manager::PlanFailure::MAP_INVALID);
    EXPECT_TRUE(result.points.empty());
}

TEST(FormationPlanner, RejectsOccupiedGoalWithoutMovingIt)
{
    nav_msgs::OccupancyGrid map = makeMap(10, 10);
    occupy(&map, 7, 7);

    const plan_manager::CenterPlan result = planner().plan(
        map, Eigen::Vector2d(2.5, 2.5), Eigen::Vector2d(7.5, 7.5));

    EXPECT_EQ(result.failure, plan_manager::PlanFailure::GOAL_IN_COLLISION);
    EXPECT_TRUE(result.points.empty());
}

TEST(FormationPlanner, RejectsStartDeepInsideInflatedObstacle)
{
    nav_msgs::OccupancyGrid map = makeMap(30, 14, 0.1);
    for (int y = 0; y < 14; ++y) occupy(&map, 5, y);

    const plan_manager::CenterPlan result = planner(0.35).plan(
        map, Eigen::Vector2d(0.65, 0.65), Eigen::Vector2d(2.05, 0.65));

    EXPECT_EQ(result.failure, plan_manager::PlanFailure::START_IN_COLLISION);
    EXPECT_TRUE(result.points.empty());
}

TEST(FormationPlanner, EscapesDeepEnvelopeStartOnlyWhenCallerValidatesRobotFootprints)
{
    nav_msgs::OccupancyGrid map = makeMap(30, 14, 0.1);
    for (int y = 0; y < 14; ++y) occupy(&map, 5, y);
    const Eigen::Vector2d start(0.65, 0.65);
    const Eigen::Vector2d goal(2.05, 0.65);
    const plan_manager::ClearanceGrid grid(map, true);
    ASSERT_GT(grid.clearanceAtWorld(start), 0.0);
    ASSERT_LT(grid.clearanceAtWorld(start), 0.35 - map.info.resolution);

    const plan_manager::CenterPlan result = planner(0.35).plan(
        map, start, goal, true);

    ASSERT_EQ(result.failure, plan_manager::PlanFailure::NONE);
    ASSERT_GE(result.points.size(), 2U);
    EXPECT_NEAR((result.points.front() - start).norm(), 0.0, 1e-9);
    EXPECT_GT(grid.clearanceAtWorld(result.points[1]), grid.clearanceAtWorld(start));
}

TEST(FormationPlanner, CompactReleaseRequiresPhysicalRadiusButNotRecoveredSafetyGap)
{
    nav_msgs::OccupancyGrid map = makeMap(30, 14, 0.05);
    for (int y = 0; y < 14; ++y) occupy(&map, 5, y);
    const plan_manager::ClearanceGrid grid(map, true);
    // This cell has enough clearance for a 0.11 m body but not for the body
    // plus a 0.04 m comfort gap, matching the post-transport HOME failure.
    const std::vector<Eigen::Vector2d> positions{
        Eigen::Vector2d(0.425, 0.325),
        Eigen::Vector2d(0.625, 0.325),
    };
    ASSERT_GT(grid.clearanceAtWorld(positions.front()), 0.11);
    ASSERT_LT(grid.clearanceAtWorld(positions.front()), 0.15);

    EXPECT_TRUE(plan_manager::allRobotFootprintsCollisionFree(
        grid, positions, 0.11));
    EXPECT_FALSE(plan_manager::allRobotFootprintsCollisionFree(
        grid, positions, 0.15));
}

TEST(FormationPlanner, ContactReleaseAcceptsFreeCentersButRejectsOccupiedCenters)
{
    nav_msgs::OccupancyGrid map = makeMap(30, 14, 0.05);
    for (int y = 0; y < 14; ++y) occupy(&map, 5, y);
    const plan_manager::ClearanceGrid grid(map, true);
    const std::vector<Eigen::Vector2d> near_wall{
        // The robot footprint overlaps the conservative map boundary after a
        // physical push, but its center is still in a free occupancy cell.
        Eigen::Vector2d(0.375, 0.325),
        Eigen::Vector2d(0.625, 0.325),
    };
    ASSERT_GT(grid.clearanceAtWorld(near_wall.front()), 0.0);
    ASSERT_LT(grid.clearanceAtWorld(near_wall.front()), 0.11);

    EXPECT_TRUE(plan_manager::allRobotCentersOutsideOccupiedSpace(grid, near_wall));

    std::vector<Eigen::Vector2d> occupied = near_wall;
    occupied.front() = Eigen::Vector2d(0.275, 0.325);
    EXPECT_DOUBLE_EQ(grid.clearanceAtWorld(occupied.front()), 0.0);
    EXPECT_FALSE(plan_manager::allRobotCentersOutsideOccupiedSpace(grid, occupied));
}

TEST(FormationPlanner, EscapesMarginallyInflatedStartTowardHigherClearance)
{
    nav_msgs::OccupancyGrid map = makeMap(30, 14, 0.1);
    for (int y = 0; y < 14; ++y) occupy(&map, 5, y);
    const Eigen::Vector2d start(0.95, 0.65);
    const Eigen::Vector2d goal(2.05, 0.65);
    const double envelope = 0.35;
    const plan_manager::ClearanceGrid grid(map, true);
    ASSERT_LT(grid.clearanceAtWorld(start), envelope);
    ASSERT_GT(grid.clearanceAtWorld(start), envelope - map.info.resolution);

    const plan_manager::CenterPlan result = planner(envelope).plan(map, start, goal);

    ASSERT_EQ(result.failure, plan_manager::PlanFailure::NONE);
    ASSERT_GE(result.points.size(), 2U);
    EXPECT_NEAR((result.points.front() - start).norm(), 0.0, 1e-9);
    EXPECT_GT(grid.clearanceAtWorld(result.points[1]), grid.clearanceAtWorld(start));
}

TEST(FormationPlanner, DoesNotCutOccupiedDiagonalCorner)
{
    nav_msgs::OccupancyGrid map = makeMap(8, 8);
    occupy(&map, 3, 2);
    occupy(&map, 2, 3);
    const Eigen::Vector2d start(2.5, 2.5);
    const Eigen::Vector2d goal(3.5, 3.5);

    const plan_manager::CenterPlan result = planner(0.1).plan(map, start, goal);
    ASSERT_EQ(result.failure, plan_manager::PlanFailure::NONE);

    const plan_manager::ClearanceGrid grid(map, true);
    EXPECT_FALSE(grid.segmentFeasible(start, goal, 0.1));
    ASSERT_GE(result.points.size(), 3U)
        << "first=" << result.points.front().transpose()
        << " last=" << result.points.back().transpose();
    for (std::size_t index = 1; index < result.points.size(); ++index) {
        EXPECT_TRUE(grid.segmentFeasible(result.points[index - 1], result.points[index], 0.1));
    }
}

TEST(FormationPlanner, ChoosesCenterOfWideCorridor)
{
    const nav_msgs::OccupancyGrid map = makeMap(24, 12);
    const plan_manager::CenterPlan result = planner(0.4, 8.0).plan(
        map, Eigen::Vector2d(1.5, 2.5), Eigen::Vector2d(22.5, 2.5));

    ASSERT_EQ(result.failure, plan_manager::PlanFailure::NONE);
    ASSERT_GE(result.points.size(), 3U);
    double maximum_y = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector2d& point : result.points) {
        maximum_y = std::max(maximum_y, point.y());
    }
    EXPECT_GE(maximum_y, 5.0);
}

TEST(FormationPlanner, RejectsCorridorNarrowerThanMinimumEnvelope)
{
    nav_msgs::OccupancyGrid map = makeMap(15, 9);
    for (int x = 4; x <= 10; ++x) {
        occupy(&map, x, 2);
        occupy(&map, x, 6);
    }

    const plan_manager::CenterPlan result = planner(1.4).plan(
        map, Eigen::Vector2d(2.5, 4.5), Eigen::Vector2d(12.5, 4.5));

    EXPECT_EQ(result.failure, plan_manager::PlanFailure::NO_STATIC_PATH);
    EXPECT_TRUE(result.points.empty());
}

TEST(FormationPlanner, SmoothingPreservesEnvelopeClearance)
{
    nav_msgs::OccupancyGrid map = makeMap(14, 14);
    for (int y = 1; y <= 10; ++y) {
        occupy(&map, 6, y);
    }
    const double envelope = 0.4;
    const plan_manager::CenterPlan result = planner(envelope).plan(
        map, Eigen::Vector2d(3.5, 3.5), Eigen::Vector2d(10.5, 3.5));
    ASSERT_EQ(result.failure, plan_manager::PlanFailure::NONE);

    const plan_manager::ClearanceGrid grid(map, true);
    for (std::size_t index = 1; index < result.points.size(); ++index) {
        EXPECT_TRUE(grid.segmentFeasible(
            result.points[index - 1], result.points[index], envelope));
    }
}

TEST(FormationPlanner, MovingRobotIsASoftCostRatherThanAStaticWall)
{
    nav_msgs::OccupancyGrid map = makeMap(80, 60, 0.1);
    plan_manager::FormationPlanner planner({0.25, 0.08, true});
    const std::vector<plan_manager::CircularMapObstacle> traffic{
        {{4.0, 3.0}, 0.11},
    };

    const plan_manager::CenterPlan result = planner.plan(
        map, {1.0, 3.0}, {7.0, 3.0}, false, traffic);

    ASSERT_EQ(plan_manager::PlanFailure::NONE, result.failure);
    ASSERT_GE(result.points.size(), 3u);
    double maximum_lateral_deviation = 0.0;
    for (const Eigen::Vector2d& point : result.points) {
        maximum_lateral_deviation = std::max(
            maximum_lateral_deviation, std::abs(point.y() - 3.0));
    }
    EXPECT_GT(maximum_lateral_deviation, 0.15);
}

TEST(FormationPlanner, MovingRobotAtGoalCausesWaitingLocallyNotPermanentPlanFailure)
{
    nav_msgs::OccupancyGrid map = makeMap(80, 60, 0.1);
    plan_manager::FormationPlanner planner({0.25, 0.08, true});
    const std::vector<plan_manager::CircularMapObstacle> traffic{
        {{7.0, 3.0}, 0.11},
    };

    const plan_manager::CenterPlan result = planner.plan(
        map, {1.0, 3.0}, {7.0, 3.0}, false, traffic);

    EXPECT_EQ(plan_manager::PlanFailure::NONE, result.failure);
    EXPECT_FALSE(result.points.empty());
}
