#include "plan_manager/dynamic_obstacle_planning.h"

#include <algorithm>
#include <cmath>

namespace plan_manager
{
std::vector<CircularMapObstacle> sweptCircularObstacles(
    const std::vector<MovingDisc>& obstacles,
    double prediction_horizon,
    double sample_spacing)
{
    std::vector<CircularMapObstacle> result;
    const double horizon = std::max(0.0, prediction_horizon);
    const double spacing = std::max(1e-6, sample_spacing);
    for (const MovingDisc& obstacle : obstacles) {
        if (!obstacle.position.allFinite() || !obstacle.velocity.allFinite() ||
            !std::isfinite(obstacle.radius) || obstacle.radius <= 0.0) {
            continue;
        }
        const double travel = obstacle.velocity.norm() * horizon;
        const int intervals = static_cast<int>(std::ceil(travel / spacing));
        const int samples = std::max(1, intervals + 1);
        for (int index = 0; index < samples; ++index) {
            const double fraction = samples == 1
                ? 0.0 : static_cast<double>(index) / static_cast<double>(samples - 1);
            result.push_back({
                obstacle.position + fraction * horizon * obstacle.velocity,
                obstacle.radius,
            });
        }
    }
    return result;
}

}  // namespace plan_manager
