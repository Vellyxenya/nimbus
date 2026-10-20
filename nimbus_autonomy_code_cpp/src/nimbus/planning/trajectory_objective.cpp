#include "nimbus/planning/trajectory_objective.hpp"

#include <algorithm>
#include <cmath>

#include "nimbus/occupancy_grid.hpp"
#include "nimbus/planning/motion_model.hpp"

namespace nimbus
{

TrajectoryObjective::TrajectoryObjective(const ObjectiveWeights& weights)
    : weights_(weights)
{
}

double TrajectoryObjective::evaluate(const PlannedTrajectory& trajectory,
                                     const RollingOccupancyGrid& map,
                                     AccelerationCommand previous_control) const
{
    if (trajectory.x.empty())
    {
        return weights_.collision;
    }

    double cost = -weights_.progress * (trajectory.x.back() - trajectory.x.front());
    for (size_t index = 1; index < trajectory.x.size(); ++index)
    {
        const double clearance = map.clearance(trajectory.x[index], trajectory.y[index]);
        if (std::isfinite(clearance))
        {
            cost += weights_.clearance /
                    std::pow(std::max(clearance, weights_.minimum_clearance), 2.0);
        }
        cost += weights_.vertical_velocity * std::pow(trajectory.vy[index], 2.0);
    }
    for (const auto& control : trajectory.controls)
    {
        cost += weights_.control_effort *
                (std::pow(control.ax / MotionModel::kMaxAccelerationX, 2.0) +
                 std::pow(control.ay / MotionModel::kMaxAccelerationY, 2.0));
        cost += weights_.control_rate * (std::pow((control.ax - previous_control.ax) /
                                                          MotionModel::kMaxAccelerationX,
                                                  2.0) +
                                         std::pow((control.ay - previous_control.ay) /
                                                          MotionModel::kMaxAccelerationY,
                                                  2.0));
        previous_control = control;
    }
    if (trajectory.collided)
    {
        const size_t steps_after_collision =
                trajectory.controls.size() - trajectory.first_collision_step;
        cost += weights_.collision + weights_.early_collision * steps_after_collision;
    }
    return cost;
}

}  // namespace nimbus
