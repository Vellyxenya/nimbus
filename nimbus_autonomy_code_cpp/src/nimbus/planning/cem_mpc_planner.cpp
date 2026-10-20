#include "nimbus/planning/cem_mpc_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "nimbus/occupancy_grid.hpp"

namespace nimbus
{

namespace
{

// The ROS adapter replans at the game's 30 Hz scan rate.
constexpr size_t kControlsExecutedPerPlan = 1;

}  // namespace

CemMpcPlanner::CemMpcPlanner(CemMpcParameters parameters)
    : parameters_(parameters),
      random_engine_(parameters_.random_seed),
      objective_(parameters_.objective_weights)
{
    // Fail early because the sample/refit loop relies on non-empty, compatible
    // dimensions and never needs to defend against invalid configuration later.
    if (parameters_.horizon == 0 || parameters_.sample_count == 0 ||
        parameters_.elite_count == 0 ||
        parameters_.elite_count > parameters_.sample_count || parameters_.iterations == 0)
    {
        throw std::invalid_argument(
                "CEM dimensions must be positive and elite count must not exceed "
                "samples");
    }
}

PlannedTrajectory CemMpcPlanner::plan(const MotionState& initial,
                                      const RollingOccupancyGrid& map)
{
    // The normal runtime path deliberately avoids benchmark instrumentation.
    return planInternal(initial, map, nullptr);
}

PlannedTrajectory CemMpcPlanner::planWithTiming(const MotionState& initial,
                                                const RollingOccupancyGrid& map)
{
    // Reset rather than accumulate timings so a benchmark measures one plan.
    last_timing_ = {};
    return planInternal(initial, map, &last_timing_);
}

PlannedTrajectory CemMpcPlanner::planInternal(const MotionState& initial,
                                              const RollingOccupancyGrid& map,
                                              CemMpcPhaseTimings* timing)
{
    // The time-indexed Gaussian persists across calls, providing CEM's MPC
    // warm start after the distribution is shifted by the applied control.
    if (means_.empty())
    {
        initialiseDistribution();
    }

    PlannedTrajectory best;
    best.cost = std::numeric_limits<double>::infinity();
    const bool initial_state_is_illegal = map.isCollision(initial.x, initial.y);
    PlannedTrajectory best_initial_state_recovery;
    best_initial_state_recovery.cost = std::numeric_limits<double>::infinity();
    const auto planning_started = std::chrono::steady_clock::now();
    bool planning_time_exhausted = false;
    size_t candidates_evaluated = 0;

    // Every iteration samples from the current Gaussian, scores the resulting
    // rollouts, and refits to elites before the next iteration explores around
    // that improved distribution.
    for (size_t iteration = 0; iteration < parameters_.iterations; ++iteration)
    {
        auto candidates = sampleAndEvaluateCandidates(initial, map, planning_started,
                                                      planning_time_exhausted,
                                                      candidates_evaluated, timing);
        if (candidates.empty())
        {
            break;
        }
        selectElitesAndRefit(candidates, initial_state_is_illegal, best,
                             best_initial_state_recovery, timing);
        if (planning_time_exhausted)
        {
            break;
        }
    }
    if (initial_state_is_illegal && best_initial_state_recovery.x.empty())
    {
        // A budget exhausted before even one candidate was evaluated still
        // needs a command that attempts to leave conservative inflation.
        const auto controls = sampleControls();
        best_initial_state_recovery = MotionModel::simulate(initial, controls, map);
        best_initial_state_recovery.cost = objective_.evaluate(
                best_initial_state_recovery, map, previous_applied_control_);
        ++candidates_evaluated;
    }

    // MPC executes only the first control, so align the fitted Gaussian with
    // the remaining horizon before the next scan arrives.
    shiftDistribution(kControlsExecutedPerPlan);
    if (timing != nullptr)
    {
        timing->candidates_evaluated = candidates_evaluated;
    }
    if (!best.x.empty())
    {
        best.planning_time_exhausted = planning_time_exhausted;
        best.candidates_evaluated = candidates_evaluated;
        previous_applied_control_ = best.controls.front();
        return best;
    }
    if (initial_state_is_illegal)
    {
        best_initial_state_recovery.is_initial_state_recovery = true;
        best_initial_state_recovery.planning_time_exhausted = planning_time_exhausted;
        best_initial_state_recovery.candidates_evaluated = candidates_evaluated;
        previous_applied_control_ = best_initial_state_recovery.controls.front();
        return best_initial_state_recovery;
    }

    // A legal initial state with no feasible rollout uses a deterministic,
    // conservative fallback rather than sending an arbitrary sampled command.
    auto fallback = MotionModel::simulate(initial, emergencyBrakeControls(initial), map);
    fallback.cost = objective_.evaluate(fallback, map, previous_applied_control_);
    fallback.is_emergency_fallback = true;
    fallback.planning_time_exhausted = planning_time_exhausted;
    fallback.candidates_evaluated = candidates_evaluated;
    previous_applied_control_ = fallback.controls.front();
    return fallback;
}

std::vector<PlannedTrajectory> CemMpcPlanner::sampleAndEvaluateCandidates(
        const MotionState& initial, const RollingOccupancyGrid& map,
        std::chrono::steady_clock::time_point planning_started,
        bool& planning_time_exhausted, size_t& candidates_evaluated,
        CemMpcPhaseTimings* timing)
{
    std::vector<PlannedTrajectory> candidates;
    candidates.reserve(parameters_.sample_count);
    for (size_t sample = 0; sample < parameters_.sample_count; ++sample)
    {
        // Stop between candidates so every sampled sequence is complete and
        // timing remains meaningful even when the MPC budget is exhausted.
        if (std::chrono::steady_clock::now() - planning_started >= kMpcPlanningBudget)
        {
            planning_time_exhausted = true;
            break;
        }

        std::vector<AccelerationCommand> controls;
        if (timing != nullptr)
        {
            const auto phase_started = std::chrono::steady_clock::now();
            controls = sampleControls();
            timing->control_sampling += std::chrono::steady_clock::now() - phase_started;
        }
        else
        {
            controls = sampleControls();
        }

        PlannedTrajectory candidate;
        if (timing != nullptr)
        {
            MotionModelPhaseTimings rollout_timing;
            candidate = MotionModel::simulateWithTiming(initial, controls, map,
                                                        rollout_timing);
            timing->state_propagation += rollout_timing.state_propagation;
            timing->collision_checking += rollout_timing.collision_checking;
        }
        else
        {
            candidate = MotionModel::simulate(initial, controls, map);
        }

        if (timing != nullptr)
        {
            const auto phase_started = std::chrono::steady_clock::now();
            candidate.cost =
                    objective_.evaluate(candidate, map, previous_applied_control_);
            timing->objective_evaluation +=
                    std::chrono::steady_clock::now() - phase_started;
        }
        else
        {
            candidate.cost =
                    objective_.evaluate(candidate, map, previous_applied_control_);
        }
        candidates.push_back(std::move(candidate));
        ++candidates_evaluated;
    }
    return candidates;
}

void CemMpcPlanner::selectElitesAndRefit(std::vector<PlannedTrajectory>& candidates,
                                         bool initial_state_is_illegal,
                                         PlannedTrajectory& best_feasible,
                                         PlannedTrajectory& best_initial_state_recovery,
                                         CemMpcPhaseTimings* timing)
{
    const auto selection_started = timing != nullptr ?
                                           std::chrono::steady_clock::now() :
                                           std::chrono::steady_clock::time_point{};

    // Feasibility is a hard requirement during normal operation; colliding
    // samples cannot pull the Gaussian toward a shortcut through a pipe.
    std::vector<PlannedTrajectory> feasible_candidates;
    feasible_candidates.reserve(candidates.size());
    std::copy_if(candidates.cbegin(), candidates.cend(),
                 std::back_inserter(feasible_candidates),
                 [](const PlannedTrajectory& candidate) { return !candidate.collided; });
    std::sort(candidates.begin(), candidates.end(),
              [](const PlannedTrajectory& left, const PlannedTrajectory& right)
              { return left.cost < right.cost; });

    // When already inside conservative inflation, retain the least-cost
    // candidate as a recovery plan but do not relax normal feasibility rules.
    if (initial_state_is_illegal &&
        candidates.front().cost < best_initial_state_recovery.cost)
    {
        best_initial_state_recovery = candidates.front();
    }

    if (!feasible_candidates.empty())
    {
        std::sort(feasible_candidates.begin(), feasible_candidates.end(),
                  [](const PlannedTrajectory& left, const PlannedTrajectory& right)
                  { return left.cost < right.cost; });
        if (feasible_candidates.front().cost < best_feasible.cost)
        {
            best_feasible = feasible_candidates.front();
        }
        feasible_candidates.resize(
                std::min(parameters_.elite_count, feasible_candidates.size()));
        refitDistribution(feasible_candidates);
    }
    else
    {
        // No safe sample exists yet. Refitting the least-bad candidates may
        // steer the next iteration toward a still-unobserved or narrow gap.
        candidates.resize(std::min(parameters_.elite_count, candidates.size()));
        refitDistribution(candidates);
    }

    if (timing != nullptr)
    {
        timing->selection_and_refit +=
                std::chrono::steady_clock::now() - selection_started;
    }
}

void CemMpcPlanner::initialiseDistribution()
{
    // Forward bias makes first-time plans progress, while vertical variance
    // remains broad enough to discover either side of a pipe gap.
    means_.assign(parameters_.horizon, {1.0, 0.0});
    standard_deviations_.assign(parameters_.horizon, {1.0, 8.0});
}

std::vector<AccelerationCommand> CemMpcPlanner::sampleControls()
{
    // Each horizon step has an independent Gaussian. Clamping keeps samples
    // within the actual game command limits before the dynamics rollout.
    std::vector<AccelerationCommand> controls;
    controls.reserve(parameters_.horizon);
    for (size_t step = 0; step < parameters_.horizon; ++step)
    {
        const double sample_x =
                means_[step].ax + standard_deviations_[step].ax *
                                          standard_normal_distribution_(random_engine_);
        const double sample_y =
                means_[step].ay + standard_deviations_[step].ay *
                                          standard_normal_distribution_(random_engine_);
        controls.push_back({std::clamp(sample_x, -MotionModel::kMaxAccelerationX,
                                       MotionModel::kMaxAccelerationX),
                            std::clamp(sample_y, -MotionModel::kMaxAccelerationY,
                                       MotionModel::kMaxAccelerationY)});
    }
    return controls;
}

void CemMpcPlanner::refitDistribution(const std::vector<PlannedTrajectory>& elites)
{
    // CEM uses the maximum-likelihood Gaussian fit of the elite controls at
    // every timestep. Exploration floors prevent premature collapse.
    for (size_t step = 0; step < parameters_.horizon; ++step)
    {
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (const auto& elite : elites)
        {
            mean_x += elite.controls[step].ax;
            mean_y += elite.controls[step].ay;
        }
        mean_x /= static_cast<double>(elites.size());
        mean_y /= static_cast<double>(elites.size());

        double variance_x = 0.0;
        double variance_y = 0.0;
        for (const auto& elite : elites)
        {
            variance_x += std::pow(elite.controls[step].ax - mean_x, 2.0);
            variance_y += std::pow(elite.controls[step].ay - mean_y, 2.0);
        }
        means_[step] = {mean_x, mean_y};
        standard_deviations_[step] = {
                std::max(0.10,
                         std::sqrt(variance_x / static_cast<double>(elites.size()))),
                std::max(4.0,
                         std::sqrt(variance_y / static_cast<double>(elites.size())))};
    }
}

void CemMpcPlanner::shiftDistribution(size_t executed_controls)
{
    // Retain the unexecuted suffix as the next warm start, then use neutral
    // controls at the newly exposed tail of the horizon.
    const size_t shift = std::min(executed_controls, parameters_.horizon);
    for (size_t step = 0; step + shift < parameters_.horizon; ++step)
    {
        means_[step] = means_[step + shift];
        standard_deviations_[step] = standard_deviations_[step + shift];
    }
    for (size_t step = parameters_.horizon - shift; step < parameters_.horizon; ++step)
    {
        means_[step] = {1.0, 0.0};
        standard_deviations_[step] = {1.0, 8.0};
    }
}

std::vector<AccelerationCommand> CemMpcPlanner::emergencyBrakeControls(
        const MotionState& initial) const
{
    // Braking is only a last resort. Horizontal deceleration is maximal while
    // vertical damping avoids amplifying an already large climb or dive.
    std::vector<AccelerationCommand> controls;
    controls.reserve(parameters_.horizon);
    double vertical_velocity = initial.vy;
    constexpr double vertical_damping_gain = 15.0;
    for (size_t step = 0; step < parameters_.horizon; ++step)
    {
        const double ay = std::clamp(-vertical_damping_gain * vertical_velocity,
                                     -MotionModel::kMaxAccelerationY,
                                     MotionModel::kMaxAccelerationY);
        controls.push_back({-MotionModel::kMaxAccelerationX, ay});
        vertical_velocity =
                std::clamp(vertical_velocity + ay * MotionModel::kDt,
                           -MotionModel::kGameMaxVelocity, MotionModel::kGameMaxVelocity);
    }
    return controls;
}

}  // namespace nimbus
