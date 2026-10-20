#pragma once

#include <cstddef>
#include <cstdint>
#include <random>

#include "nimbus/occupancy_grid.hpp"
#include "nimbus/planning/motion_model.hpp"
#include "nimbus/planning/planner.hpp"
#include "nimbus/planning/trajectory_objective.hpp"

namespace nimbus
{

/**
 * @brief Random-shooting baseline planner for validation before CEM MPC.
 *
 * Candidate acceleration sequences are sampled within the game's documented
 * limits, simulated at 30 Hz, and scored for progress, clearance, control
 * effort, and collision. The caller decides whether to execute or only
 * visualise the first control of the selected trajectory.
 */
class RandomShootingPlanner : public Planner
{
  public:
    /**
     * @brief Creates a deterministic random-shooting planner.
     * @param horizon Number of 30 Hz simulation steps per candidate.
     * @param sample_count Number of random sequences assessed per plan call.
     * @param seed Random seed, fixed by default to make visual debugging repeatable.
     * @param objective_weights Objective used to rank sampled trajectories.
     */
    explicit RandomShootingPlanner(size_t horizon = 12, size_t sample_count = 400,
                                   uint32_t seed = 42,
                                   const ObjectiveWeights& objective_weights = {});

    /** @brief Simulates one sequence with game-consistent velocity limits. */
    PlannedTrajectory simulate(const MotionState& initial,
                               const std::vector<AccelerationCommand>& controls,
                               const RollingOccupancyGrid& map) const;

    /** @brief Samples and returns the lowest-cost candidate for the current local map. */
    PlannedTrajectory plan(const MotionState& initial,
                           const RollingOccupancyGrid& map) override;

  private:
    /** @brief Samples one piecewise-constant acceleration sequence. */
    std::vector<AccelerationCommand> sampleControls();

    /// Number of model steps simulated for every candidate.
    size_t horizon_;
    /// Number of sampled candidates evaluated on each plan call.
    size_t sample_count_;
    /// Seeded pseudo-random source used to make candidate generation reproducible.
    std::mt19937 random_engine_;
    /// Shared objective used to rank sampled trajectories.
    TrajectoryObjective objective_;
};

}  // namespace nimbus
