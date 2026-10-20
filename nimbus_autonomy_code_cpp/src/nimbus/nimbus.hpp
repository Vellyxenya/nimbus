#pragma once

#include <memory>

#include "nimbus/planning/planner.hpp"
#include "nimbus/planning/trajectory_objective.hpp"

namespace nimbus
{

/**
 * @brief ROS-independent autonomy state and algorithms.
 *
 * It owns the active planner and provides the ROS-independent entry point for
 * trajectory selection. The ROS adapter supplies sensing and visualisation only;
 * it does not depend on any concrete planner implementation.
 */
class Nimbus
{
  public:
    /** @brief Creates Nimbus with a planner selected by the factory. */
    explicit Nimbus(PlannerType planner_type = PlannerType::kCemMpc);

    /** @brief Creates Nimbus with the supplied trajectory-selection objective. */
    Nimbus(const ObjectiveWeights& objective_weights,
           PlannerType planner_type = PlannerType::kCemMpc);

    /** @brief Delegates trajectory selection to the active planner. */
    PlannedTrajectory plan(const MotionState& state, const RollingOccupancyGrid& map);

  private:
    /// Active planner selected at construction time.
    std::unique_ptr<Planner> planner_;
};

}  // namespace nimbus
