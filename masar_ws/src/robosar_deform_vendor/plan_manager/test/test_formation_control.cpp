#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "plan_manager/formation_control.h"

namespace
{

plan_manager::FormationLimits limits()
{
    return plan_manager::FormationLimits{0.18, 0.60, 0.04, 0.20};
}

}  // namespace

TEST(FormationControl, MinimumScaleComesFromRobotDiameterAndGap)
{
    EXPECT_NEAR(plan_manager::minimumFormationScale(limits()), 2.0 / 3.0, 1e-12);
}

TEST(FormationControl, CircularEnvelopeUsesPhysicalMinimumFormation)
{
    const plan_manager::FormationLimits burger{0.11, 0.60, 0.04, 0.20};
    const double expected =
        plan_manager::minimumFormationScale(burger) * 0.60 / std::sqrt(2.0) + 0.11 + 0.08;
    EXPECT_NEAR(
        plan_manager::minimumCircularEnvelopeRadius(burger, 0.08), expected, 1e-12);
    EXPECT_LT(expected, 0.40);
}

TEST(FormationControl, AffineTargetsKeepCentroidAtCenter)
{
    const Eigen::Vector2d center(4.0, -2.0);
    const std::vector<Eigen::Vector2d> slots = plan_manager::squareNominalSlots(0.60);
    plan_manager::DeformationState deformation;
    deformation.longitudinal_scale = 0.80;
    deformation.lateral_scale = 0.70;

    const std::vector<Eigen::Vector2d> targets = plan_manager::affineSlotTargets(
        center, 0.73, slots, deformation);

    ASSERT_EQ(targets.size(), 4U);
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const Eigen::Vector2d& target : targets) {
        mean += target;
    }
    mean /= static_cast<double>(targets.size());
    EXPECT_NEAR(mean.x(), center.x(), 1e-12);
    EXPECT_NEAR(mean.y(), center.y(), 1e-12);
}

TEST(FormationControl, ZeroMeanLocalOffsetsDoNotMoveCenter)
{
    const Eigen::Vector2d center(2.0, 3.0);
    plan_manager::DeformationState deformation;
    deformation.local_offsets = {
        Eigen::Vector2d(0.20, 0.05),
        Eigen::Vector2d(-0.10, 0.00),
        Eigen::Vector2d(0.00, -0.15),
        Eigen::Vector2d(0.15, 0.10),
    };

    const std::vector<Eigen::Vector2d> targets = plan_manager::affineSlotTargets(
        center, 0.0, plan_manager::squareNominalSlots(0.60), deformation);

    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const Eigen::Vector2d& target : targets) {
        mean += target;
    }
    mean /= static_cast<double>(targets.size());
    EXPECT_NEAR(mean.x(), center.x(), 1e-12);
    EXPECT_NEAR(mean.y(), center.y(), 1e-12);
}

TEST(FormationControl, LateralScaleCanShrinkWithoutLongitudinalScale)
{
    plan_manager::DeformationState deformation;
    deformation.longitudinal_scale = 1.0;
    deformation.lateral_scale = 0.70;
    const std::vector<Eigen::Vector2d> targets = plan_manager::affineSlotTargets(
        Eigen::Vector2d::Zero(),
        0.0,
        plan_manager::squareNominalSlots(0.60),
        deformation);

    EXPECT_NEAR(std::abs(targets[0].x() - targets[2].x()), 0.60, 1e-12);
    EXPECT_NEAR(std::abs(targets[0].y() - targets[1].y()), 0.42, 1e-12);
}

TEST(FormationControl, ScaleChangeIsRateLimited)
{
    plan_manager::DeformationState current;
    const plan_manager::DeformationState next = plan_manager::updateDeformationScales(
        current, 0.50, 0.40, limits(), 1.0);

    EXPECT_NEAR(next.longitudinal_scale, 0.80, 1e-12);
    EXPECT_NEAR(next.lateral_scale, 0.80, 1e-12);
}

TEST(FormationControl, FormationYawFollowsSharpPathTurnContinuously)
{
    const double start = -2.80;
    const double target = -1.57;
    const double updated = plan_manager::updateFormationYaw(
        start, target, 0.16, 0.30, 0.10);

    // Slot tangential speed is bounded by the center-speed budget:
    // delta <= (0.16 / 0.30) * 0.10 rad.
    EXPECT_GT(updated, start);
    EXPECT_NEAR(updated - start, (0.16 / 0.30) * 0.10, 1e-12);
    EXPECT_LT(updated, target);
}

TEST(FormationControl, FormationYawUsesShortestDirectionAcrossPi)
{
    const double updated = plan_manager::updateFormationYaw(
        3.12, -3.12, 0.20, 0.30, 0.10);
    const double delta = std::atan2(
        std::sin(updated - 3.12), std::cos(updated - 3.12));
    EXPECT_GT(delta, 0.0);
    EXPECT_LT(delta, 0.067);
}

TEST(FormationControl, QualityDescribesVisibleImperfectSquare)
{
    const std::vector<Eigen::Vector2d> targets = plan_manager::squareNominalSlots(0.60);
    std::vector<Eigen::Vector2d> actual = targets;
    actual[0] += Eigen::Vector2d(0.08, -0.04);
    actual[1] += Eigen::Vector2d(-0.05, 0.06);
    actual[2] += Eigen::Vector2d(0.03, 0.02);
    actual[3] += Eigen::Vector2d(-0.04, -0.03);

    const plan_manager::FormationQuality quality = plan_manager::evaluateFormationQuality(
        actual, targets, Eigen::Vector2d::Zero(), 0.40);

    EXPECT_LT(quality.rms_slot_error, 0.10);
    EXPECT_LT(quality.max_slot_error, 0.10);
    EXPECT_LT(quality.center_error, 0.02);
    EXPECT_GT(quality.minimum_pair_distance, 0.40);
    EXPECT_TRUE(quality.collision_safe);
}

TEST(FormationControl, CenterFeedforwardDecaysContinuouslyWithSlotError)
{
    EXPECT_DOUBLE_EQ(plan_manager::formationQualitySpeedScale(0.0, 0.60), 1.0);
    EXPECT_NEAR(plan_manager::formationQualitySpeedScale(0.60, 0.60), 0.5, 1e-12);
    EXPECT_NEAR(plan_manager::formationQualitySpeedScale(1.80, 0.60), 0.1, 1e-12);
    EXPECT_GT(
        plan_manager::formationQualitySpeedScale(0.30, 0.60),
        plan_manager::formationQualitySpeedScale(0.90, 0.60));
}

TEST(FormationControl, DifferentialTargetTurnsBeforeForwardInsteadOfReversing)
{
    const auto forward = plan_manager::differentialDriveTarget(
        Eigen::Vector2d(0.20, 0.0), 0.0, 0.0);
    EXPECT_NEAR(forward.linear_velocity, 0.20, 1e-12);
    EXPECT_NEAR(forward.heading_error, 0.0, 1e-12);

    const auto backward_target = plan_manager::differentialDriveTarget(
        Eigen::Vector2d(-0.20, 0.0), 0.0, 0.0);
    EXPECT_NEAR(backward_target.linear_velocity, 0.0, 1e-12);
    EXPECT_NEAR(std::abs(backward_target.heading_error), std::acos(-1.0), 1e-12);
}

TEST(FormationControl, DifferentialTargetIgnoresTinyDirectionJitter)
{
    const auto target = plan_manager::differentialDriveTarget(
        Eigen::Vector2d(0.0, 0.005), 0.0, -0.5 * std::acos(-1.0), 0.01);
    EXPECT_DOUBLE_EQ(target.linear_velocity, 0.0);
    EXPECT_NEAR(target.heading_error, -0.5 * std::acos(-1.0), 1e-12);

    const auto translating = plan_manager::differentialDriveTarget(
        Eigen::Vector2d(0.0, 0.02), 0.0, -0.5 * std::acos(-1.0), 0.01);
    EXPECT_NEAR(translating.heading_error, 0.5 * std::acos(-1.0), 1e-12);
}

TEST(FormationArrival, WaypointAllowsSmallNonRigidOffsets)
{
    plan_manager::ArrivalEvaluator evaluator;
    const plan_manager::FormationQuality quality{0.22, 0.38, 0.48, 0.12, true};
    const plan_manager::ArrivalConfig config{0.30, 0.35, 0.55, 0.8};
    const std::vector<Eigen::Vector2d> robots = plan_manager::squareNominalSlots(0.60);
    auto evidence = evaluator.evaluate(
        plan_manager::ArrivalProfile::WAYPOINT, config, quality, robots,
        Eigen::Vector2d::Zero(), 0.0, 0.0);
    EXPECT_FALSE(evidence.arrived);
    evidence = evaluator.evaluate(
        plan_manager::ArrivalProfile::WAYPOINT, config, quality, robots,
        Eigen::Vector2d::Zero(), 0.0, 0.81);
    EXPECT_TRUE(evidence.arrived);
}

TEST(FormationArrival, SurroundRequiresTargetInsideConvexHull)
{
    plan_manager::ArrivalEvaluator evaluator;
    const plan_manager::FormationQuality quality{0.1, 0.2, 0.5, 0.1, true};
    const plan_manager::ArrivalConfig config{0.3, 0.35, 0.55, 0.0};
    const std::vector<Eigen::Vector2d> one_sided = {
        {0.2, -0.3}, {0.2, 0.3}, {0.8, 0.3}, {0.8, -0.3}};
    const auto evidence = evaluator.evaluate(
        plan_manager::ArrivalProfile::SURROUND, config, quality, one_sided,
        Eigen::Vector2d::Zero(), 0.0, 0.0);
    EXPECT_FALSE(evidence.profile_geometry_ok);
    EXPECT_FALSE(evidence.arrived);
}

TEST(FormationArrival, TransportRequiresTwoRowsBehindBox)
{
    plan_manager::ArrivalEvaluator evaluator;
    const plan_manager::FormationQuality quality{0.1, 0.2, 0.4, 0.1, true};
    const plan_manager::ArrivalConfig config{0.4, 0.45, 0.7, 0.0};
    const std::vector<Eigen::Vector2d> good = {
        {-0.25, -0.22}, {-0.25, 0.22}, {-0.65, -0.22}, {-0.65, 0.22}};
    EXPECT_TRUE(evaluator.evaluate(
        plan_manager::ArrivalProfile::TRANSPORT, config, quality, good,
        Eigen::Vector2d::Zero(), 0.0, 0.0).profile_geometry_ok);
    const std::vector<Eigen::Vector2d> in_front = {
        {0.25, -0.22}, {0.25, 0.22}, {-0.25, -0.22}, {-0.25, 0.22}};
    EXPECT_FALSE(evaluator.evaluate(
        plan_manager::ArrivalProfile::TRANSPORT, config, quality, in_front,
        Eigen::Vector2d::Zero(), 0.0, 0.1).profile_geometry_ok);
}

TEST(FormationArrival, DoesNotRequireNominalScaleNearGoal)
{
    plan_manager::ArrivalEvaluator evaluator;
    plan_manager::DeformationState deformation;
    deformation.longitudinal_scale = 0.72;
    deformation.lateral_scale = 0.72;
    const auto robots = plan_manager::affineSlotTargets(
        Eigen::Vector2d::Zero(), 0.0, plan_manager::squareNominalSlots(0.60), deformation);
    const auto quality = plan_manager::evaluateFormationQuality(
        robots, robots, Eigen::Vector2d::Zero(), 0.30);
    const plan_manager::ArrivalConfig config{0.3, 0.35, 0.55, 0.0};
    EXPECT_TRUE(evaluator.evaluate(
        plan_manager::ArrivalProfile::WAYPOINT, config, quality, robots,
        Eigen::Vector2d::Zero(), 0.0, 0.0).arrived);
}

TEST(FormationArrival, HoldTimeRejectsTransientPassThrough)
{
    plan_manager::ArrivalEvaluator evaluator;
    const plan_manager::ArrivalConfig config{0.3, 0.35, 0.55, 1.0};
    const std::vector<Eigen::Vector2d> robots = plan_manager::squareNominalSlots(0.60);
    const plan_manager::FormationQuality good{0.1, 0.2, 0.5, 0.1, true};
    plan_manager::FormationQuality bad = good;
    bad.center_error = 1.0;
    evaluator.evaluate(plan_manager::ArrivalProfile::WAYPOINT, config, good, robots,
                       Eigen::Vector2d::Zero(), 0.0, 0.0);
    EXPECT_FALSE(evaluator.evaluate(plan_manager::ArrivalProfile::WAYPOINT, config, bad, robots,
                                    Eigen::Vector2d::Zero(), 0.0, 0.6).arrived);
    EXPECT_FALSE(evaluator.evaluate(plan_manager::ArrivalProfile::WAYPOINT, config, good, robots,
                                    Eigen::Vector2d::Zero(), 0.0, 1.1).arrived);
    EXPECT_TRUE(evaluator.evaluate(plan_manager::ArrivalProfile::WAYPOINT, config, good, robots,
                                   Eigen::Vector2d::Zero(), 0.0, 2.2).arrived);
}
