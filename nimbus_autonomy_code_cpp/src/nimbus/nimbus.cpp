#include "nimbus/nimbus.hpp"

namespace nimbus
{

Nimbus::Nimbus(PlannerType planner_type) : planner_(PlannerFactory::create(planner_type))
{
}

Nimbus::Nimbus(const ObjectiveWeights& objective_weights, PlannerType planner_type)
    : planner_(PlannerFactory::create(planner_type, objective_weights))
{
}

PlannedTrajectory Nimbus::plan(const MotionState& state, const RollingOccupancyGrid& map)
{
    return planner_->plan(state, map);
}

}  // namespace nimbus
