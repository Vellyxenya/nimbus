#include "nimbus/planning/motion_model.hpp"

#include <algorithm>

#include "nimbus/occupancy_grid.hpp"

namespace nimbus
{

namespace
{

// At 30 Hz and the autonomy speed cap, five frames are enough to exit the
// conservative inflation that can temporarily cover the current bird pose.
constexpr size_t kInitialInflationEscapeSteps = 5;

double limitAxisAcceleration(double velocity, double requested_acceleration,
                             double minimum_velocity, double maximum_velocity,
                             double maximum_acceleration)
{
    const double minimum_allowed = (minimum_velocity - velocity) / MotionModel::kDt;
    const double maximum_allowed = (maximum_velocity - velocity) / MotionModel::kDt;
    const double lower_bound = std::max(-maximum_acceleration, minimum_allowed);
    const double upper_bound = std::min(maximum_acceleration, maximum_allowed);
    if (lower_bound > upper_bound)
    {
        return velocity > maximum_velocity ? -maximum_acceleration : maximum_acceleration;
    }
    return std::clamp(requested_acceleration, lower_bound, upper_bound);
}

PlannedTrajectory simulateInternal(const MotionState& initial,
                                   const std::vector<AccelerationCommand>& controls,
                                   const RollingOccupancyGrid& map,
                                   MotionModelPhaseTimings* timing)
{
    PlannedTrajectory trajectory;
    trajectory.x.reserve(controls.size() + 1);
    trajectory.y.reserve(controls.size() + 1);
    trajectory.vx.reserve(controls.size() + 1);
    trajectory.vy.reserve(controls.size() + 1);
    trajectory.controls.reserve(controls.size());
    trajectory.x.push_back(initial.x);
    trajectory.y.push_back(initial.y);
    trajectory.vx.push_back(initial.vx);
    trajectory.vy.push_back(initial.vy);
    const auto initial_collision =
            map.segmentCollisionReason(initial.x, initial.y, initial.x, initial.y);
    bool escaping_initial_inflation =
            initial_collision == CollisionReason::kInflatedObstacle &&
            map.segmentCollisionReason(initial.x, initial.y, initial.x, initial.y,
                                       true) == CollisionReason::kNone;
    for (const auto& requested_control : controls)
    {
        const auto propagation_started = timing != nullptr ?
                                                 std::chrono::steady_clock::now() :
                                                 std::chrono::steady_clock::time_point{};
        const double previous_x = trajectory.x.back();
        const double previous_y = trajectory.y.back();
        const double previous_vx = trajectory.vx.back();
        const double previous_vy = trajectory.vy.back();
        const AccelerationCommand control = MotionModel::limitControlForVelocity(
                {previous_x, previous_y, previous_vx, previous_vy}, requested_control);
        const double next_vx = std::clamp(previous_vx + control.ax * MotionModel::kDt,
                                          0.0, MotionModel::kGameMaxVelocity);
        const double next_vy =
                std::clamp(previous_vy + control.ay * MotionModel::kDt,
                           -MotionModel::kGameMaxVelocity, MotionModel::kGameMaxVelocity);
        // The game applies acceleration, clamps velocity, then advances position.
        const double next_x = previous_x + next_vx * MotionModel::kDt;
        const double next_y = previous_y + next_vy * MotionModel::kDt;
        trajectory.controls.push_back(control);
        trajectory.x.push_back(next_x);
        trajectory.y.push_back(next_y);
        trajectory.vx.push_back(next_vx);
        trajectory.vy.push_back(next_vy);
        if (timing != nullptr)
        {
            timing->state_propagation +=
                    std::chrono::steady_clock::now() - propagation_started;
        }

        const auto collision_started = timing != nullptr ?
                                               std::chrono::steady_clock::now() :
                                               std::chrono::steady_clock::time_point{};
        auto collision_reason =
                map.segmentCollisionReason(previous_x, previous_y, next_x, next_y);
        if (escaping_initial_inflation &&
            trajectory.controls.size() <= kInitialInflationEscapeSteps)
        {
            // Relax only the conservative layer while exiting the initial
            // overlap. Raw obstacles and map boundaries remain collision checks.
            collision_reason = map.segmentCollisionReason(previous_x, previous_y, next_x,
                                                          next_y, true);
            const auto endpoint_reason =
                    map.segmentCollisionReason(next_x, next_y, next_x, next_y);
            escaping_initial_inflation =
                    endpoint_reason == CollisionReason::kInflatedObstacle;
        }
        else
        {
            escaping_initial_inflation = false;
        }
        if (timing != nullptr)
        {
            timing->collision_checking +=
                    std::chrono::steady_clock::now() - collision_started;
        }
        if (!trajectory.collided && collision_reason != CollisionReason::kNone)
        {
            trajectory.collided = true;
            trajectory.first_collision_step = trajectory.controls.size();
            trajectory.collision_reason = collision_reason;
        }
    }
    return trajectory;
}

}  // namespace

AccelerationCommand MotionModel::limitControlForVelocity(
        const MotionState& state, const AccelerationCommand& requested)
{
    return {limitAxisAcceleration(state.vx, requested.ax, 0.0, kAutonomyMaxVelocity,
                                  kMaxAccelerationX),
            limitAxisAcceleration(state.vy, requested.ay, -kAutonomyMaxVelocity,
                                  kAutonomyMaxVelocity, kMaxAccelerationY)};
}

PlannedTrajectory MotionModel::simulate(const MotionState& initial,
                                        const std::vector<AccelerationCommand>& controls,
                                        const RollingOccupancyGrid& map)
{
    return simulateInternal(initial, controls, map, nullptr);
}

PlannedTrajectory MotionModel::simulateWithTiming(
        const MotionState& initial, const std::vector<AccelerationCommand>& controls,
        const RollingOccupancyGrid& map, MotionModelPhaseTimings& timing)
{
    timing = {};
    return simulateInternal(initial, controls, map, &timing);
}

}  // namespace nimbus
