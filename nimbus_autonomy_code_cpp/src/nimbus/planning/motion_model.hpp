#pragma once

#include <chrono>
#include <vector>

#include "nimbus/planning/planner.hpp"

namespace nimbus
{

class RollingOccupancyGrid;

/** @brief Per-rollout timings exposed only for local performance benchmarking. */
struct MotionModelPhaseTimings
{
    /// Time spent governing controls and propagating state vectors.
    std::chrono::nanoseconds state_propagation{};
    /// Time spent rasterising predicted segments through the inflated map.
    std::chrono::nanoseconds collision_checking{};
};

/** @brief Discrete, acceleration-limited model shared by every planner. */
class MotionModel
{
  public:
    /// Game update period in seconds.
    static constexpr double kDt = 1.0 / 30.0;
    /// Documented maximum forward acceleration in metres per second squared.
    static constexpr double kMaxAccelerationX = 3.0;
    /// Documented maximum vertical acceleration in metres per second squared.
    static constexpr double kMaxAccelerationY = 35.0;
    /// Physical game velocity limit in metres per second.
    static constexpr double kGameMaxVelocity = 10.0;
    /// Maximum allowed velocity for autonomy planning in metres per second.
    /// This is to account for braking distance and empirical safety margin.
    static constexpr double kAutonomyMaxVelocity = 2.0;

    /**
     * @brief Converts a requested acceleration into one that respects the autonomy speed
     * cap.
     *
     * The returned command is constrained so the next 30 Hz game update remains
     * within the autonomy speed range whenever that is physically achievable.
     */
    static AccelerationCommand limitControlForVelocity(
            const MotionState& state, const AccelerationCommand& requested);

    /**
     * @brief Simulates a command sequence using the common planner dynamics.
     *
     * Acceleration updates and clamps velocity before position is advanced, matching
     * the game's update order. Commands and velocities are clamped to game limits.
     * The returned trajectory contains the initial state followed by every prediction.
     * When that initial state lies in inflated occupancy but not outside the map, its
     * rollout may leave the conservative region; all later collisions remain hard
     * constraints.
     */
    static PlannedTrajectory simulate(const MotionState& initial,
                                      const std::vector<AccelerationCommand>& controls,
                                      const RollingOccupancyGrid& map);

    /** @brief Simulates a command sequence while measuring propagation and collision
     * work. */
    static PlannedTrajectory simulateWithTiming(
            const MotionState& initial, const std::vector<AccelerationCommand>& controls,
            const RollingOccupancyGrid& map, MotionModelPhaseTimings& timing);
};

}  // namespace nimbus
