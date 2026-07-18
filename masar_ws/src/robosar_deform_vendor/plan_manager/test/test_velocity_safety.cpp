#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "plan_manager/velocity_safety.h"

namespace
{

plan_manager::VelocitySafetyConfig config()
{
    return plan_manager::VelocitySafetyConfig{0.18, 0.04, 2.0, 1.0, 0.5};
}

plan_manager::DiscObstacle obstacle(
    double x,
    double y,
    double vx = 0.0,
    double vy = 0.0,
    double stamp = 10.0)
{
    return plan_manager::DiscObstacle{
        Eigen::Vector2d(x, y), Eigen::Vector2d(vx, vy), 0.18, ros::Time(stamp)};
}

}  // namespace

TEST(VelocitySafety, MovingApartHasNoPredictedCollision)
{
    const double ttc = plan_manager::predictedTimeToCollision(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(-0.5, 0.0),
        obstacle(2.0, 0.0),
        config());

    EXPECT_TRUE(std::isinf(ttc));
}

TEST(VelocitySafety, HeadOnPreferredVelocityIsProjected)
{
    const std::vector<plan_manager::DiscObstacle> obstacles{
        obstacle(2.0, 0.0, -0.5, 0.0),
    };
    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.8, 0.0),
        obstacles,
        config(),
        ros::Time(10.1));

    EXPECT_FALSE(result.velocity.isApprox(Eigen::Vector2d(0.8, 0.0), 1e-12));
    EXPECT_TRUE(result.blocked || result.minimum_time_to_collision > config().time_horizon);
}

TEST(VelocitySafety, NearTangentMotionRemainsUnchanged)
{
    const Eigen::Vector2d preferred(0.8, 0.0);
    const std::vector<plan_manager::DiscObstacle> obstacles{
        obstacle(1.5, 0.45),
    };
    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        preferred,
        preferred,
        obstacles,
        config(),
        ros::Time(10.1));

    EXPECT_TRUE(result.velocity.isApprox(preferred, 1e-12));
    EXPECT_FALSE(result.blocked);
}

TEST(VelocitySafety, AlreadyOverlappingAllowsSeparatingVelocity)
{
    const Eigen::Vector2d preferred(-0.5, 0.0);
    const std::vector<plan_manager::DiscObstacle> obstacles{
        obstacle(0.2, 0.0),
    };
    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero(),
        preferred,
        obstacles,
        config(),
        ros::Time(10.1));

    EXPECT_TRUE(result.velocity.isApprox(preferred, 1e-12));
    EXPECT_FALSE(result.blocked);
}

TEST(VelocitySafety, StaleObstacleIsIgnored)
{
    const Eigen::Vector2d preferred(0.7, 0.0);
    const std::vector<plan_manager::DiscObstacle> obstacles{
        obstacle(0.8, 0.0, 0.0, 0.0, 8.0),
    };
    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        preferred,
        preferred,
        obstacles,
        config(),
        ros::Time(10.0));

    EXPECT_TRUE(result.velocity.isApprox(preferred, 1e-12));
    EXPECT_TRUE(std::isinf(result.minimum_time_to_collision));
}

TEST(VelocitySafety, ZeroPreferredVelocityStaysZero)
{
    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.2, 0.0),
        Eigen::Vector2d::Zero(),
        std::vector<plan_manager::DiscObstacle>{obstacle(0.5, 0.0)},
        config(),
        ros::Time(10.1));

    EXPECT_TRUE(result.velocity.isZero(1e-12));
    EXPECT_FALSE(result.blocked);
}

TEST(VelocitySafety, StationaryRobotEvadesAnApproachingDynamicObstacle)
{
    plan_manager::VelocitySafetyConfig physical{0.11, 0.04, 2.0, 0.20, 0.8};
    const std::vector<plan_manager::DiscObstacle> obstacles{
        plan_manager::DiscObstacle{
            Eigen::Vector2d(0.35, 0.0), Eigen::Vector2d(-0.15, 0.0),
            0.11, ros::Time(10.0)},
    };

    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero(),
        obstacles,
        physical,
        ros::Time(10.1));

    EXPECT_FALSE(result.blocked);
    EXPECT_NEAR(result.velocity.x(), -physical.max_speed, 1e-12);
    EXPECT_GT(result.minimum_time_to_collision, physical.time_horizon);
}

TEST(VelocitySafety, RobotCollisionThreatPrioritizesFullRadialSeparation)
{
    plan_manager::VelocitySafetyConfig physical{0.11, 0.04, 2.0, 0.20, 0.8};
    const std::vector<plan_manager::DiscObstacle> obstacles{
        plan_manager::DiscObstacle{
            Eigen::Vector2d(0.50, 0.0), Eigen::Vector2d(-0.10, 0.0),
            0.11, ros::Time(10.0), true},
    };

    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.05, 0.0),
        Eigen::Vector2d(0.20, 0.0),
        obstacles,
        physical,
        ros::Time(10.1));

    EXPECT_FALSE(result.blocked);
    EXPECT_NEAR(result.velocity.x(), -physical.max_speed, 1e-12);
    EXPECT_NEAR(result.velocity.y(), 0.0, 1e-12);
}

TEST(VelocitySafety, StationaryRobotDoesNotRetreatFromPreferredPathConflict)
{
    plan_manager::VelocitySafetyConfig physical{0.11, 0.04, 2.0, 0.20, 0.8};
    const std::vector<plan_manager::DiscObstacle> obstacles{
        plan_manager::DiscObstacle{
            Eigen::Vector2d(0.50, 0.0), Eigen::Vector2d::Zero(),
            0.11, ros::Time(10.0), true},
    };

    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.20, 0.0),
        obstacles,
        physical,
        ros::Time(10.1));

    // A hypothetical conflict in the preferred path is not an emergency.
    // The projector may slow down or steer around it, but must not create the
    // retreat/re-approach limit cycle seen around stationary teammates.
    EXPECT_FALSE(result.blocked);
    EXPECT_GE(result.velocity.x(), 0.0);
    EXPECT_GT(result.velocity.norm(), 0.0);
    EXPECT_GT(result.minimum_time_to_collision, physical.time_horizon);
}

TEST(VelocitySafety, RepeatedCallsReturnIdenticalVelocity)
{
    const std::vector<plan_manager::DiscObstacle> obstacles{
        obstacle(1.0, 0.0),
        obstacle(0.8, 0.5, 0.0, -0.1),
    };
    const plan_manager::SafeVelocityResult first = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.1, 0.0),
        Eigen::Vector2d(0.8, 0.0),
        obstacles,
        config(),
        ros::Time(10.1));

    for (int iteration = 0; iteration < 100; ++iteration) {
        const plan_manager::SafeVelocityResult next = plan_manager::projectSafeVelocity(
            Eigen::Vector2d::Zero(),
            Eigen::Vector2d(0.1, 0.0),
            Eigen::Vector2d(0.8, 0.0),
            obstacles,
            config(),
            ros::Time(10.1));
        EXPECT_DOUBLE_EQ(next.velocity.x(), first.velocity.x());
        EXPECT_DOUBLE_EQ(next.velocity.y(), first.velocity.y());
        EXPECT_EQ(next.blocked, first.blocked);
    }
}

TEST(VelocitySafety, CloseObstacleUsesSeparatingEscapeInsteadOfZeroDeadlock)
{
    plan_manager::VelocitySafetyConfig physical{0.11, 0.04, 2.0, 0.20, 0.8};
    const std::vector<plan_manager::DiscObstacle> obstacles{
        plan_manager::DiscObstacle{
            Eigen::Vector2d(0.17, 0.0), Eigen::Vector2d::Zero(),
            0.03, ros::Time(10.0)},
    };

    const plan_manager::SafeVelocityResult result = plan_manager::projectSafeVelocity(
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d::Zero(),
        Eigen::Vector2d(0.20, 0.0),
        obstacles,
        physical,
        ros::Time(10.1));

    EXPECT_FALSE(result.blocked);
    EXPECT_NEAR(result.velocity.x(), -physical.max_speed, 1e-12);
}
