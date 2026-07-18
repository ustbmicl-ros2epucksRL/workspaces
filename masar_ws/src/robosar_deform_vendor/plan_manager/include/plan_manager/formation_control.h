#ifndef PLAN_MANAGER_FORMATION_CONTROL_H
#define PLAN_MANAGER_FORMATION_CONTROL_H

#include <Eigen/Core>

#include <vector>

namespace plan_manager
{

struct FormationLimits
{
    double robot_radius = 0.0;
    double nominal_side = 0.0;
    double inter_robot_safety_gap = 0.0;
    double max_scale_rate_per_second = 0.0;
};

struct DeformationState
{
    double longitudinal_scale = 1.0;
    double lateral_scale = 1.0;
    std::vector<Eigen::Vector2d> local_offsets;
};

struct FormationQuality
{
    double rms_slot_error = 0.0;
    double max_slot_error = 0.0;
    double minimum_pair_distance = 0.0;
    double center_error = 0.0;
    bool collision_safe = false;
};

struct DifferentialDriveTarget
{
    double linear_velocity = 0.0;
    double heading_error = 0.0;
};

enum class ArrivalProfile { WAYPOINT, SURROUND, TRANSPORT, HOME };

struct ArrivalConfig
{
    double center_tolerance = 0.0;
    double rms_slot_tolerance = 0.0;
    double max_slot_tolerance = 0.0;
    double hold_seconds = 0.0;
};

struct ArrivalEvidence
{
    bool arrived = false;
    bool center_ok = false;
    bool formation_ok = false;
    bool safety_ok = false;
    bool profile_geometry_ok = false;
    double hold_remaining = 0.0;
};

class ArrivalEvaluator
{
public:
    ArrivalEvidence evaluate(
        ArrivalProfile profile,
        const ArrivalConfig& config,
        const FormationQuality& quality,
        const std::vector<Eigen::Vector2d>& robot_positions,
        const Eigen::Vector2d& profile_target,
        double target_yaw,
        double now_seconds);
    void reset();

private:
    double valid_since_ = -1.0;
};

double minimumFormationScale(const FormationLimits& limits);

double minimumCircularEnvelopeRadius(
    const FormationLimits& limits,
    double external_clearance);

std::vector<Eigen::Vector2d> squareNominalSlots(double nominal_side);

DeformationState updateDeformationScales(
    const DeformationState& current,
    double target_longitudinal_scale,
    double target_lateral_scale,
    const FormationLimits& limits,
    double dt_seconds);

double updateFormationYaw(
    double current_yaw,
    double target_yaw,
    double maximum_slot_speed,
    double formation_radius,
    double dt_seconds);

std::vector<Eigen::Vector2d> affineSlotTargets(
    const Eigen::Vector2d& center,
    double path_yaw,
    const std::vector<Eigen::Vector2d>& nominal_slots,
    const DeformationState& deformation);

FormationQuality evaluateFormationQuality(
    const std::vector<Eigen::Vector2d>& robot_positions,
    const std::vector<Eigen::Vector2d>& slot_targets,
    const Eigen::Vector2d& center_reference,
    double minimum_safe_pair_distance);

double formationQualitySpeedScale(
    double maximum_slot_error,
    double nominal_side);

DifferentialDriveTarget differentialDriveTarget(
    const Eigen::Vector2d& map_velocity,
    double robot_yaw,
    double terminal_yaw,
    double minimum_translation_speed = 1e-4);

}  // namespace plan_manager

#endif
