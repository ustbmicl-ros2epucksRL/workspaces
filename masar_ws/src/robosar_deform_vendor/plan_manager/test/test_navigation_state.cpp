#include <gtest/gtest.h>

#include <cmath>

#include "plan_manager/formation_navigation.h"

using plan_manager::NavigationEvent;
using plan_manager::NavigationInput;
using plan_manager::NavigationState;
using plan_manager::NavigationStateMachine;

TEST(NavigationState, StartsStoppedWithoutMapOrMotionPermission)
{
    NavigationStateMachine machine(2.0, 1.0);
    EXPECT_EQ(machine.update(NavigationInput(), 0.0).state, NavigationState::STOPPED);
}

TEST(NavigationState, MissingMapDeadlineProducesMapUnavailable)
{
    NavigationStateMachine machine(2.0, 1.0);
    NavigationInput input;
    input.has_goal = true;
    input.motion_enabled = true;
    EXPECT_EQ(machine.update(input, 0.0).state, NavigationState::WAITING_FOR_MAP);
    const auto result = machine.update(input, 2.1);
    EXPECT_EQ(result.state, NavigationState::ERROR);
    EXPECT_EQ(result.detail, "MAP_UNAVAILABLE");
}

TEST(NavigationState, StaticNoPathDoesNotProduceDirectFallback)
{
    NavigationStateMachine machine(2.0, 1.0);
    NavigationInput input;
    input.has_goal = input.motion_enabled = input.map_ready = input.odom_ready = true;
    input.planning_event = NavigationEvent::STATIC_NO_PATH;
    const auto result = machine.update(input, 0.0);
    EXPECT_EQ(result.state, NavigationState::FAILED);
    EXPECT_EQ(result.detail, "NO_STATIC_PATH");
    EXPECT_FALSE(result.command_motion);
}

TEST(NavigationState, PlanningFailureKeepsExactReasonAfterMotionGateCloses)
{
    NavigationStateMachine machine(2.0, 1.0);
    NavigationInput input;
    input.has_goal = input.motion_enabled = input.map_ready = input.odom_ready = true;
    input.planning_event = NavigationEvent::START_IN_COLLISION;
    auto result = machine.update(input, 0.0);
    EXPECT_EQ(result.state, NavigationState::FAILED);
    EXPECT_EQ(result.detail, "START_IN_COLLISION");

    input.motion_enabled = false;
    result = machine.update(input, 0.1);
    EXPECT_EQ(result.state, NavigationState::FAILED);
    EXPECT_EQ(result.detail, "START_IN_COLLISION");
    EXPECT_FALSE(result.command_motion);
}

TEST(NavigationState, DynamicBlockRetriesOnlyAfterBackoffOrDataChange)
{
    NavigationStateMachine machine(2.0, 1.0);
    NavigationInput input;
    input.has_goal = input.motion_enabled = input.map_ready = input.odom_ready = true;
    input.planning_event = NavigationEvent::PLAN_READY;
    EXPECT_EQ(machine.update(input, 0.0).state, NavigationState::TRACKING);

    input.planning_event = NavigationEvent::DYNAMIC_BLOCKED;
    EXPECT_FALSE(machine.update(input, 0.2).request_plan);
    input.planning_event = NavigationEvent::NONE;
    EXPECT_FALSE(machine.update(input, 0.9).request_plan);
    EXPECT_TRUE(machine.update(input, 1.2).request_plan);
    EXPECT_FALSE(machine.update(input, 1.3).request_plan);
    input.data_revision++;
    EXPECT_TRUE(machine.update(input, 1.4).request_plan);
}

TEST(NavigationState, StopPreservesGoalAndPathProgress)
{
    NavigationStateMachine machine(2.0, 1.0);
    NavigationInput input;
    input.has_goal = input.motion_enabled = input.map_ready = input.odom_ready = true;
    input.planning_event = NavigationEvent::PLAN_READY;
    machine.update(input, 0.0);
    machine.setPathProgress(4.25);

    input.motion_enabled = false;
    const auto paused = machine.update(input, 0.1);
    EXPECT_EQ(paused.state, NavigationState::PAUSED);
    EXPECT_TRUE(machine.hasGoal());
    EXPECT_DOUBLE_EQ(machine.pathProgress(), 4.25);
    EXPECT_FALSE(paused.command_motion);
}

TEST(TransportApproach, SelectsNearestFeasiblePointInBehindBoxRegion)
{
    const plan_manager::TransportApproachSearch config{0.55, 1.60, 0.60, 0.05};
    const auto result = plan_manager::resolveTransportApproach(
        Eigen::Vector2d::Zero(), Eigen::Vector2d(0.0, 1.0),
        Eigen::Vector2d(0.0, -1.0), config,
        [](const Eigen::Vector2d& point) {
            return point.y() >= -0.90 - 1e-9 && std::abs(point.x()) < 0.01;
        });

    ASSERT_TRUE(result.found);
    EXPECT_NEAR(result.point.x(), 0.0, 1e-12);
    EXPECT_NEAR(result.point.y(), -0.90, 1e-12);
}

TEST(TransportApproach, ReportsFailureWhenBehindBoxRegionHasNoFeasiblePoint)
{
    const plan_manager::TransportApproachSearch config{0.55, 1.20, 0.30, 0.05};
    const auto result = plan_manager::resolveTransportApproach(
        Eigen::Vector2d::Zero(), Eigen::Vector2d(0.0, 1.0),
        Eigen::Vector2d(0.0, -1.0), config,
        [](const Eigen::Vector2d&) { return false; });
    EXPECT_FALSE(result.found);
}
