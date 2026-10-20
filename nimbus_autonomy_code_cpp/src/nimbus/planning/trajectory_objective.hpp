#pragma once

#include "nimbus/planning/planner.hpp"

namespace nimbus
{

class RollingOccupancyGrid;

/** @brief Independently tunable weights for the trajectory-selection objective. */
struct ObjectiveWeights
{
    /// Reward per metre of forward progress; represented as a negative cost.
    double progress{175.0};
    /// Penalty scaling for inverse-squared obstacle clearance.
    double clearance{0.08};
    /// Penalty for squared vertical speed, discouraging unnecessary climbs/dives.
    double vertical_velocity{0.10};
    /// Penalty for normalized squared acceleration magnitude.
    double control_effort{0.09};
    /// Penalty for a normalized squared change between consecutive controls.
    double control_rate{4.0};
    /// Dominant penalty applied to every trajectory that collides.
    double collision{1.0e6};
    /// Additional penalty for colliding earlier in the horizon.
    double early_collision{500.0};
    /// Lower bound used to keep the inverse-squared clearance cost finite.
    double minimum_clearance{0.05};
};

/**
 * @brief Single source of truth for trajectory cost calculation.
 *
 * Planners own their sampling strategies, but every candidate is ranked with
 * this objective. Tuning cost behavior therefore requires editing or injecting
 * only ObjectiveWeights rather than modifying the planner implementation.
 */
class TrajectoryObjective
{
  public:
    /** @brief Creates an objective with the supplied, documented weights. */
    explicit TrajectoryObjective(const ObjectiveWeights& weights = {});

    /** @brief Returns the total cost; lower values are preferred. */
    double evaluate(const PlannedTrajectory& trajectory, const RollingOccupancyGrid& map,
                    AccelerationCommand previous_control = {}) const;

    /** @brief Exposes the active weights for logging and future parameterisation. */
    const ObjectiveWeights& weights() const { return weights_; }

  private:
    /// All objective terms configured for this evaluator.
    ObjectiveWeights weights_;
};

}  // namespace nimbus
