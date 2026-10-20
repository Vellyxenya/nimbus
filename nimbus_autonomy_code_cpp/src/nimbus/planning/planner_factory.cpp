#include "nimbus/planning/planner.hpp"

#include <stdexcept>

#include "nimbus/planning/cem_mpc_planner.hpp"
#include "nimbus/planning/random_shooting_planner.hpp"
#include "nimbus/planning/trajectory_objective.hpp"

namespace nimbus
{

std::unique_ptr<Planner> PlannerFactory::create(PlannerType type)
{
    return create(type, {});
}

std::unique_ptr<Planner> PlannerFactory::create(PlannerType type,
                                                const ObjectiveWeights& objective_weights)
{
    switch (type)
    {
        case PlannerType::kRandomShooting:
            return std::make_unique<RandomShootingPlanner>(12, 400, 42,
                                                           objective_weights);
        case PlannerType::kCemMpc:
        {
            CemMpcParameters parameters;
            parameters.objective_weights = objective_weights;
            return std::make_unique<CemMpcPlanner>(parameters);
        }
    }
    throw std::invalid_argument("unknown planner type");
}

}  // namespace nimbus
