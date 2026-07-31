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
