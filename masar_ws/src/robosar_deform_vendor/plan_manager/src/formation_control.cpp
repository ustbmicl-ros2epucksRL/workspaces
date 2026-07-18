#include "plan_manager/formation_control.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace plan_manager
{
namespace
{

double moveTowards(double current, double target, double maximum_delta)
{
    if (target > current) {
        return std::min(target, current + maximum_delta);
    }
    return std::max(target, current - maximum_delta);
}

std::vector<Eigen::Vector2d> normalizedOffsets(
    const std::vector<Eigen::Vector2d>& offsets,
    std::size_t expected_size)
{
    if (offsets.size() != expected_size || expected_size == 0) {
        return std::vector<Eigen::Vector2d>(expected_size, Eigen::Vector2d::Zero());
    }
    Eigen::Vector2d mean = Eigen::Vector2d::Zero();
    for (const Eigen::Vector2d& offset : offsets) {
        mean += offset;
    }
    mean /= static_cast<double>(expected_size);

    std::vector<Eigen::Vector2d> result;
    result.reserve(expected_size);
    for (const Eigen::Vector2d& offset : offsets) {
        result.push_back(offset - mean);
    }
    return result;
}

double cross(
    const Eigen::Vector2d& origin,
    const Eigen::Vector2d& a,
    const Eigen::Vector2d& b)
{
    const Eigen::Vector2d oa = a - origin;
    const Eigen::Vector2d ob = b - origin;
    return oa.x() * ob.y() - oa.y() * ob.x();
}

bool insideConvexHull(
    const std::vector<Eigen::Vector2d>& points,
    const Eigen::Vector2d& target)
{
    if (points.size() < 3) return false;
    std::vector<Eigen::Vector2d> sorted = points;
    std::sort(sorted.begin(), sorted.end(), [](const auto& a, const auto& b) {
        return a.x() < b.x() || (a.x() == b.x() && a.y() < b.y());
    });
    std::vector<Eigen::Vector2d> hull;
    for (const auto& point : sorted) {
        while (hull.size() >= 2 && cross(hull[hull.size()-2], hull.back(), point) <= 0.0)
            hull.pop_back();
        hull.push_back(point);
    }
    const std::size_t lower = hull.size();
    for (auto it = sorted.rbegin() + 1; it != sorted.rend(); ++it) {
        while (hull.size() > lower && cross(hull[hull.size()-2], hull.back(), *it) <= 0.0)
            hull.pop_back();
        hull.push_back(*it);
    }
    if (hull.size() < 4) return false;
    hull.pop_back();
    double sign = 0.0;
    for (std::size_t i = 0; i < hull.size(); ++i) {
        const double value = cross(hull[i], hull[(i + 1) % hull.size()], target);
        if (std::abs(value) < 1e-9) continue;
        if (sign == 0.0) sign = value;
        else if (value * sign < 0.0) return false;
    }
    return sign != 0.0;
}

bool transportGeometry(
    const std::vector<Eigen::Vector2d>& positions,
    const Eigen::Vector2d& box_center,
    double yaw)
{
    if (positions.size() != 4) return false;
    const Eigen::Vector2d forward(std::cos(yaw), std::sin(yaw));
    const Eigen::Vector2d lateral(-forward.y(), forward.x());
    double min_long = std::numeric_limits<double>::infinity();
    double max_long = -std::numeric_limits<double>::infinity();
    bool left = false;
    bool right = false;
    for (const auto& position : positions) {
        const Eigen::Vector2d relative = position - box_center;
        const double longitudinal = relative.dot(forward);
        const double sideways = relative.dot(lateral);
        if (longitudinal >= -0.05) return false;
        min_long = std::min(min_long, longitudinal);
        max_long = std::max(max_long, longitudinal);
        left = left || sideways > 0.05;
        right = right || sideways < -0.05;
    }
    return max_long - min_long >= 0.18 && left && right;
}

}  // namespace

double minimumFormationScale(const FormationLimits& limits)
{
    const double nominal_side = std::max(0.0, limits.nominal_side);
    if (nominal_side <= 1e-9) {
        return 1.0;
    }
    const double minimum_side =
        2.0 * std::max(0.0, limits.robot_radius) +
        std::max(0.0, limits.inter_robot_safety_gap);
    return std::max(0.0, std::min(1.0, minimum_side / nominal_side));
}

double minimumCircularEnvelopeRadius(
    const FormationLimits& limits,
    double external_clearance)
{
    const double half_diagonal = minimumFormationScale(limits) *
        std::max(0.0, limits.nominal_side) / std::sqrt(2.0);
    return half_diagonal + std::max(0.0, limits.robot_radius) +
        std::max(0.0, external_clearance);
}

std::vector<Eigen::Vector2d> squareNominalSlots(double nominal_side)
{
    const double half_side = 0.5 * std::max(0.0, nominal_side);
    return {
        Eigen::Vector2d(half_side, half_side),
        Eigen::Vector2d(half_side, -half_side),
        Eigen::Vector2d(-half_side, -half_side),
        Eigen::Vector2d(-half_side, half_side),
    };
}

DeformationState updateDeformationScales(
    const DeformationState& current,
    double target_longitudinal_scale,
    double target_lateral_scale,
    const FormationLimits& limits,
    double dt_seconds)
{
    DeformationState next = current;
    const double minimum_scale = minimumFormationScale(limits);
    const double target_longitudinal = std::max(
        minimum_scale, std::min(1.0, target_longitudinal_scale));
    const double target_lateral = std::max(
        minimum_scale, std::min(1.0, target_lateral_scale));
    const double maximum_delta =
        std::max(0.0, limits.max_scale_rate_per_second) * std::max(0.0, dt_seconds);
    next.longitudinal_scale = moveTowards(
        std::max(minimum_scale, std::min(1.0, current.longitudinal_scale)),
        target_longitudinal,
        maximum_delta);
    next.lateral_scale = moveTowards(
        std::max(minimum_scale, std::min(1.0, current.lateral_scale)),
        target_lateral,
        maximum_delta);
    return next;
}

double updateFormationYaw(
    double current_yaw,
    double target_yaw,
    double maximum_slot_speed,
    double formation_radius,
    double dt_seconds)
{
    const double normalized_current = std::atan2(
        std::sin(current_yaw), std::cos(current_yaw));
    const double delta = std::atan2(
        std::sin(target_yaw - normalized_current),
        std::cos(target_yaw - normalized_current));
    const double radius = std::max(0.0, formation_radius);
    if (radius <= 1e-9) {
        return std::atan2(std::sin(target_yaw), std::cos(target_yaw));
    }
    const double maximum_delta =
        std::max(0.0, maximum_slot_speed) / radius * std::max(0.0, dt_seconds);
    const double limited_delta = std::max(
        -maximum_delta, std::min(maximum_delta, delta));
    return std::atan2(
        std::sin(normalized_current + limited_delta),
        std::cos(normalized_current + limited_delta));
}

std::vector<Eigen::Vector2d> affineSlotTargets(
    const Eigen::Vector2d& center,
    double path_yaw,
    const std::vector<Eigen::Vector2d>& nominal_slots,
    const DeformationState& deformation)
{
    const double c = std::cos(path_yaw);
    const double s = std::sin(path_yaw);
    const std::vector<Eigen::Vector2d> offsets = normalizedOffsets(
        deformation.local_offsets, nominal_slots.size());
    std::vector<Eigen::Vector2d> targets;
    targets.reserve(nominal_slots.size());
    for (std::size_t index = 0; index < nominal_slots.size(); ++index) {
        const Eigen::Vector2d scaled(
            nominal_slots[index].x() * deformation.longitudinal_scale,
            nominal_slots[index].y() * deformation.lateral_scale);
        const Eigen::Vector2d rotated(
            c * scaled.x() - s * scaled.y(),
            s * scaled.x() + c * scaled.y());
        targets.push_back(center + rotated + offsets[index]);
    }
    return targets;
}

FormationQuality evaluateFormationQuality(
    const std::vector<Eigen::Vector2d>& robot_positions,
    const std::vector<Eigen::Vector2d>& slot_targets,
    const Eigen::Vector2d& center_reference,
    double minimum_safe_pair_distance)
{
    FormationQuality quality;
    if (robot_positions.empty() || robot_positions.size() != slot_targets.size()) {
        const double infinity = std::numeric_limits<double>::infinity();
        quality.rms_slot_error = infinity;
        quality.max_slot_error = infinity;
        quality.minimum_pair_distance = 0.0;
        quality.center_error = infinity;
        quality.collision_safe = false;
        return quality;
    }

    Eigen::Vector2d actual_center = Eigen::Vector2d::Zero();
    double squared_error_sum = 0.0;
    quality.minimum_pair_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < robot_positions.size(); ++index) {
        actual_center += robot_positions[index];
        const double error = (robot_positions[index] - slot_targets[index]).norm();
        squared_error_sum += error * error;
        quality.max_slot_error = std::max(quality.max_slot_error, error);
        for (std::size_t other = index + 1; other < robot_positions.size(); ++other) {
            quality.minimum_pair_distance = std::min(
                quality.minimum_pair_distance,
                (robot_positions[index] - robot_positions[other]).norm());
        }
    }
    actual_center /= static_cast<double>(robot_positions.size());
    quality.rms_slot_error = std::sqrt(
        squared_error_sum / static_cast<double>(robot_positions.size()));
    quality.center_error = (actual_center - center_reference).norm();
    if (!std::isfinite(quality.minimum_pair_distance)) {
        quality.minimum_pair_distance = 0.0;
    }
    quality.collision_safe = quality.minimum_pair_distance + 1e-12 >=
        std::max(0.0, minimum_safe_pair_distance);
    return quality;
}

double formationQualitySpeedScale(
    double maximum_slot_error,
    double nominal_side)
{
    const double reference_length = std::max(1e-6, nominal_side);
    const double normalized_error = std::max(0.0, maximum_slot_error) / reference_length;
    return 1.0 / (1.0 + normalized_error * normalized_error);
}

DifferentialDriveTarget differentialDriveTarget(
    const Eigen::Vector2d& map_velocity,
    double robot_yaw,
    double terminal_yaw,
    double minimum_translation_speed)
{
    DifferentialDriveTarget target;
    const double speed = map_velocity.norm();
    const bool translating = speed > std::max(0.0, minimum_translation_speed);
    const double desired_yaw = translating
        ? std::atan2(map_velocity.y(), map_velocity.x())
        : terminal_yaw;
    const double error = std::atan2(
        std::sin(desired_yaw - robot_yaw),
        std::cos(desired_yaw - robot_yaw));
    if (translating) {
        target.linear_velocity = speed * std::max(0.0, std::cos(error));
    }
    if (std::abs(error) > 1.05) target.linear_velocity = 0.0;
    target.heading_error = error;
    return target;
}

ArrivalEvidence ArrivalEvaluator::evaluate(
    ArrivalProfile profile,
    const ArrivalConfig& config,
    const FormationQuality& quality,
    const std::vector<Eigen::Vector2d>& robot_positions,
    const Eigen::Vector2d& profile_target,
    double target_yaw,
    double now_seconds)
{
    ArrivalEvidence evidence;
    evidence.center_ok = quality.center_error <= std::max(0.0, config.center_tolerance);
    evidence.formation_ok = quality.rms_slot_error <= std::max(0.0, config.rms_slot_tolerance) &&
        quality.max_slot_error <= std::max(0.0, config.max_slot_tolerance);
    evidence.safety_ok = quality.collision_safe;
    evidence.profile_geometry_ok = true;
    if (profile == ArrivalProfile::SURROUND) {
        evidence.profile_geometry_ok = insideConvexHull(robot_positions, profile_target);
    } else if (profile == ArrivalProfile::TRANSPORT) {
        evidence.profile_geometry_ok = transportGeometry(
            robot_positions, profile_target, target_yaw);
    }
    const bool valid = evidence.center_ok && evidence.formation_ok &&
        evidence.safety_ok && evidence.profile_geometry_ok;
    if (!valid) {
        valid_since_ = -1.0;
        evidence.hold_remaining = std::max(0.0, config.hold_seconds);
        return evidence;
    }
    if (valid_since_ < 0.0) valid_since_ = now_seconds;
    const double held = std::max(0.0, now_seconds - valid_since_);
    evidence.hold_remaining = std::max(0.0, config.hold_seconds - held);
    evidence.arrived = held + 1e-12 >= std::max(0.0, config.hold_seconds);
    return evidence;
}

void ArrivalEvaluator::reset() { valid_since_ = -1.0; }

}  // namespace plan_manager
