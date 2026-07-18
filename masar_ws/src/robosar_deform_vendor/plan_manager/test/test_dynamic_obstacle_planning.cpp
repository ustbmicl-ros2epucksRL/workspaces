#include <gtest/gtest.h>

#include "plan_manager/dynamic_obstacle_planning.h"

namespace
{

plan_manager::MovingDisc moving(
    const std::string& name, double x, double y, double vx, double vy,
    double radius = 0.11)
{
    plan_manager::MovingDisc result;
    result.name = name;
    result.position = Eigen::Vector2d(x, y);
    result.velocity = Eigen::Vector2d(vx, vy);
    result.radius = radius;
    return result;
}

}  // namespace

TEST(DynamicObstaclePlanning, SweptSamplesCoverPredictedMotion)
{
    const auto samples = plan_manager::sweptCircularObstacles(
        {moving("agent_2", 0.0, 0.0, 0.2, 0.0)}, 2.0, 0.1);

    ASSERT_EQ(5u, samples.size());
    EXPECT_NEAR(0.0, samples.front().center.x(), 1e-9);
    EXPECT_NEAR(0.4, samples.back().center.x(), 1e-9);
    for (const auto& sample : samples) EXPECT_NEAR(0.11, sample.radius, 1e-9);
}

TEST(DynamicObstaclePlanning, SnapshotComparisonUsesNamesAndDistance)
{
    const std::vector<plan_manager::MovingDisc> planned{
        moving("agent_0", 0.0, 0.0, 0.0, 0.0),
        moving("agent_1", 1.0, 0.0, 0.0, 0.0),
    };
    const std::vector<plan_manager::MovingDisc> reordered_small_motion{
        moving("agent_1", 1.04, 0.0, 0.0, 0.0),
        moving("agent_0", 0.0, 0.0, 0.0, 0.0),
    };
    const std::vector<plan_manager::MovingDisc> path_crossing{
        moving("agent_1", 1.20, 0.0, 0.0, 0.0),
        moving("agent_0", 0.0, 0.0, 0.0, 0.0),
    };

    EXPECT_FALSE(plan_manager::movingObstacleSnapshotChanged(
        reordered_small_motion, planned, 0.10));
    EXPECT_TRUE(plan_manager::movingObstacleSnapshotChanged(
        path_crossing, planned, 0.10));
    EXPECT_TRUE(plan_manager::movingObstacleSnapshotChanged(
        {moving("agent_0", 0.0, 0.0, 0.0, 0.0)}, planned, 0.10));
}

TEST(DynamicObstaclePlanning, OnlyRemainingPathTriggersReplan)
{
    const std::vector<Eigen::Vector2d> path{
        {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0},
    };
    const std::vector<plan_manager::CircularMapObstacle> behind{
        {{0.2, 0.0}, 0.11},
    };
    const std::vector<plan_manager::CircularMapObstacle> ahead{
        {{2.4, 0.2}, 0.11},
    };

    EXPECT_FALSE(plan_manager::remainingPathIntersectsObstacles(
        path, {1.5, 0.0}, behind, 0.30));
    EXPECT_TRUE(plan_manager::remainingPathIntersectsObstacles(
        path, {1.5, 0.0}, ahead, 0.30));
}

TEST(DynamicObstaclePlanning, ClearedTrafficRefreshesAnUnobstructedDetour)
{
    const std::vector<plan_manager::MovingDisc> planned{
        moving("agent_0", 1.0, 0.0, 0.0, 0.0),
        moving("agent_1", 2.0, 0.0, 0.0, 0.0),
    };
    const std::vector<plan_manager::MovingDisc> cleared{
        moving("agent_0", 1.0, 3.0, 0.0, 0.0),
        moving("agent_1", 2.0, 3.0, 0.0, 0.0),
    };
    const std::vector<Eigen::Vector2d> stale_detour{
        {0.0, 0.0}, {0.0, -2.0}, {3.0, -2.0}, {3.0, 0.0},
    };
    const auto current_obstacles = plan_manager::sweptCircularObstacles(
        cleared, 2.0, 0.1);

    EXPECT_FALSE(plan_manager::remainingPathIntersectsObstacles(
        stale_detour, {0.0, 0.0}, current_obstacles, 0.30));
    EXPECT_TRUE(plan_manager::dynamicRouteNeedsRefresh(cleared, planned, 0.35));

    std::vector<plan_manager::MovingDisc> jittered = planned;
    jittered.front().position.x() += 0.10;
    EXPECT_FALSE(plan_manager::dynamicRouteNeedsRefresh(jittered, planned, 0.35));
}
