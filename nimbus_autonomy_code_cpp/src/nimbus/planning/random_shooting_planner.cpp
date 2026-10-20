#include "nimbus/planning/random_shooting_planner.hpp"

#include <limits>
#include <stdexcept>

namespace nimbus
{

RandomShootingPlanner::RandomShootingPlanner(size_t horizon, size_t sample_count,
                                             uint32_t seed,
                                             const ObjectiveWeights& objective_weights)
    : horizon_(horizon),
      sample_count_(sample_count),
      random_engine_(seed),
      objective_(objective_weights)
{
    if (horizon_ == 0 || sample_count_ == 0)
    {
        throw std::invalid_argument("planner horizon and sample count must be positive");
    }
}

PlannedTrajectory RandomShootingPlanner::simulate(
        const MotionState& initial, const std::vector<AccelerationCommand>& controls,
        const RollingOccupancyGrid& map) const
{
    auto trajectory = MotionModel::simulate(initial, controls, map);
    trajectory.cost = objective_.evaluate(trajectory, map);
    return trajectory;
}

PlannedTrajectory RandomShootingPlanner::plan(const MotionState& initial,
                                              const RollingOccupancyGrid& map)
{
    PlannedTrajectory best;
    best.cost = std::numeric_limits<double>::infinity();
    for (size_t sample = 0; sample < sample_count_; ++sample)
    {
        auto candidate = simulate(initial, sampleControls(), map);
        if (candidate.cost < best.cost)
        {
            best = std::move(candidate);
        }
    }
    return best;
}

std::vector<AccelerationCommand> RandomShootingPlanner::sampleControls()
{
    std::uniform_real_distribution<double> sample_x(-MotionModel::kMaxAccelerationX,
                                                    MotionModel::kMaxAccelerationX);
    std::uniform_real_distribution<double> sample_y(-MotionModel::kMaxAccelerationY,
                                                    MotionModel::kMaxAccelerationY);
    std::vector<AccelerationCommand> controls;
    controls.reserve(horizon_);
    constexpr size_t hold_steps = 3;
    AccelerationCommand command;
    for (size_t step = 0; step < horizon_; ++step)
    {
        if (step % hold_steps == 0)
        {
            command = {sample_x(random_engine_), sample_y(random_engine_)};
        }
        controls.push_back(command);
    }
    return controls;
}

}  // namespace nimbus
