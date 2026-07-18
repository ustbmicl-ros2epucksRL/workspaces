#include "plan_manager/velocity_safety.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace plan_manager
{
namespace
{

Eigen::Vector2d clampMagnitude(const Eigen::Vector2d& velocity, double maximum)
{
    const double limit = std::max(0.0, maximum);
    const double norm = velocity.norm();
    if (norm <= limit || norm <= 1e-12) {
        return velocity;
    }
    return velocity * (limit / norm);
}

Eigen::Vector2d rotate(const Eigen::Vector2d& vector, double angle)
{
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Eigen::Vector2d(c * vector.x() - s * vector.y(), s * vector.x() + c * vector.y());
}

void appendUnique(std::vector<Eigen::Vector2d>* candidates, const Eigen::Vector2d& candidate)
{
    for (const Eigen::Vector2d& existing : *candidates) {
        if (existing.isApprox(candidate, 1e-12)) {
            return;
        }
    }
    candidates->push_back(candidate);
}

bool fresh(const DiscObstacle& obstacle, const ros::Time& now, double timeout)
{
    const double age = (now - obstacle.stamp).toSec();
    return age <= std::max(0.0, timeout);
}

double candidateMinimumTtc(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& candidate,
    const std::vector<DiscObstacle>& obstacles,
    const VelocitySafetyConfig& config,
    const ros::Time& now)
{
    double minimum = std::numeric_limits<double>::infinity();
    for (const DiscObstacle& obstacle : obstacles) {
        if (!fresh(obstacle, now, config.obstacle_timeout)) {
            continue;
        }
        minimum = std::min(
            minimum,
            predictedTimeToCollision(position, candidate, obstacle, config));
    }
    return minimum;
}

}  // namespace

double predictedTimeToCollision(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& candidate_velocity,
    const DiscObstacle& obstacle,
    const VelocitySafetyConfig& config)
{
    const Eigen::Vector2d relative_position = obstacle.position - position;
    const Eigen::Vector2d relative_velocity = candidate_velocity - obstacle.velocity;
    const double combined_radius =
        std::max(0.0, config.robot_radius) +
        std::max(0.0, obstacle.radius) +
        std::max(0.0, config.safety_gap);
    const double c = relative_position.squaredNorm() - combined_radius * combined_radius;
    if (c <= 0.0) {
        return relative_position.dot(relative_velocity) < -1e-12
            ? std::numeric_limits<double>::infinity()
            : 0.0;
    }

    const double a = relative_velocity.squaredNorm();
    if (a <= 1e-12) {
        return std::numeric_limits<double>::infinity();
    }
    const double b = -2.0 * relative_position.dot(relative_velocity);
    const double discriminant = b * b - 4.0 * a * c;
    if (discriminant < 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    const double root = std::sqrt(std::max(0.0, discriminant));
    const double first = (-b - root) / (2.0 * a);
    const double second = (-b + root) / (2.0 * a);
    if (second < 0.0) {
        return std::numeric_limits<double>::infinity();
    }
    return first >= 0.0 ? first : 0.0;
}

SafeVelocityResult projectSafeVelocity(
    const Eigen::Vector2d& position,
    const Eigen::Vector2d& current_velocity,
    const Eigen::Vector2d& preferred_velocity,
    const std::vector<DiscObstacle>& obstacles,
    const VelocitySafetyConfig& config,
    const ros::Time& now)
{
    SafeVelocityResult result;
    result.minimum_time_to_collision = std::numeric_limits<double>::infinity();
    const Eigen::Vector2d preferred = clampMagnitude(preferred_velocity, config.max_speed);
    Eigen::Vector2d candidate_basis = preferred;
    const double horizon = std::max(0.0, config.time_horizon);
    std::vector<const DiscObstacle*> urgent_obstacles;
    for (const DiscObstacle& obstacle : obstacles) {
        if (!fresh(obstacle, now, config.obstacle_timeout)) continue;
        const double combined_radius = std::max(0.0, config.robot_radius) +
            std::max(0.0, obstacle.radius) + std::max(0.0, config.safety_gap);
        const bool overlapping =
            (obstacle.position - position).norm() <= combined_radius + 1e-12;
        // Reserve the radial escape priority for a collision already implied
        // by the measured velocity.  A stationary robot whose preferred path
        // crosses a teammate is not in an emergency: marking that hypothetical
        // path as urgent makes the best candidate point directly away, which
        // creates a retreat/re-approach limit cycle.  The normal candidate
        // projection below still rejects every preferred velocity inside the
        // TTC horizon and selects a slower or tangential alternative.
        const bool robot_collision_threat = obstacle.requires_separation_response &&
            predictedTimeToCollision(
                position, current_velocity, obstacle, config) <= horizon;
        if (overlapping || robot_collision_threat) {
            urgent_obstacles.push_back(&obstacle);
        }
    }
    if (candidate_basis.norm() <= 1e-12) {
        const double stopped_ttc = candidateMinimumTtc(
            position, Eigen::Vector2d::Zero(), obstacles, config, now);
        result.minimum_time_to_collision = stopped_ttc;
        if (stopped_ttc > horizon) {
            return result;
        }
        Eigen::Vector2d escape = Eigen::Vector2d::Zero();
        for (const DiscObstacle& obstacle : obstacles) {
            if (!fresh(obstacle, now, config.obstacle_timeout) ||
                predictedTimeToCollision(
                    position, Eigen::Vector2d::Zero(), obstacle, config) > horizon) {
                continue;
            }
            if (std::find(urgent_obstacles.begin(), urgent_obstacles.end(), &obstacle) ==
                urgent_obstacles.end()) {
                urgent_obstacles.push_back(&obstacle);
            }
            const Eigen::Vector2d away = position - obstacle.position;
            if (away.norm() > 1e-9) {
                escape += away.normalized();
            }
        }
        if (escape.norm() <= 1e-9 || config.max_speed <= 0.0) {
            result.blocked = true;
            return result;
        }
        candidate_basis = escape.normalized() * config.max_speed;
    }

    std::vector<Eigen::Vector2d> candidates;
    for (double scale : {1.0, 0.75, 0.50, 0.25}) {
        appendUnique(&candidates, candidate_basis * scale);
    }
    // A full-circle deterministic lattice is still small enough for four TB3s,
    // but unlike the old +/-45 degree fan it contains separating velocities
    // when a robot starts inside another obstacle's safety disc.
    const double degrees[] = {15.0, 30.0, 45.0, 60.0, 75.0, 90.0,
                              120.0, 150.0, 180.0};
    for (double degrees_value : degrees) {
        const double radians = degrees_value * std::acos(-1.0) / 180.0;
        for (double sign : {1.0, -1.0}) {
            for (double scale : {1.0, 0.75, 0.50, 0.25}) {
                appendUnique(&candidates, rotate(candidate_basis, sign * radians) * scale);
            }
        }
    }

    const Eigen::Vector2d preferred_direction = preferred.norm() > 1e-12 ?
        preferred.normalized() : Eigen::Vector2d::Zero();
    double best_score = std::numeric_limits<double>::infinity();
    double best_separation_rate = -std::numeric_limits<double>::infinity();
    for (const Eigen::Vector2d& candidate : candidates) {
        const double minimum_ttc = candidateMinimumTtc(
            position, candidate, obstacles, config, now);
        if (minimum_ttc <= horizon) {
            continue;
        }
        const double score = (candidate - preferred).squaredNorm() +
            0.2 * (candidate - current_velocity).squaredNorm() -
            0.1 * candidate.dot(preferred_direction);
        double separation_rate = std::numeric_limits<double>::infinity();
        for (const DiscObstacle* obstacle : urgent_obstacles) {
            const Eigen::Vector2d relative = obstacle->position - position;
            if (relative.norm() <= 1e-9) continue;
            separation_rate = std::min(
                separation_rate,
                relative.normalized().dot(obstacle->velocity - candidate));
        }
        const bool urgent_better = !urgent_obstacles.empty() &&
            (separation_rate > best_separation_rate + 1e-12 ||
             (std::abs(separation_rate - best_separation_rate) <= 1e-12 &&
              score + 1e-12 < best_score));
        const bool normal_better = urgent_obstacles.empty() && score + 1e-12 < best_score;
        if (urgent_better || normal_better) {
            best_score = score;
            best_separation_rate = separation_rate;
            result.velocity = candidate;
            result.minimum_time_to_collision = minimum_ttc;
            result.blocked = false;
        }
    }
    // Zero is the last resort only. Keeping it outside the scoring loop avoids
    // a safe escape candidate being replaced by a lower-scoring stop command.
    if (!std::isfinite(best_score)) {
        result.velocity = Eigen::Vector2d::Zero();
        result.minimum_time_to_collision = candidateMinimumTtc(
            position, result.velocity, obstacles, config, now);
        result.blocked = true;
    }
    return result;
}

}  // namespace plan_manager
