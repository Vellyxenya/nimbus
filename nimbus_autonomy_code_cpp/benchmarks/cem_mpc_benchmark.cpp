#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "nimbus/occupancy_grid.hpp"
#include "nimbus/planning/cem_mpc_planner.hpp"

namespace
{

constexpr size_t kWarmupRuns = 10;
constexpr size_t kMeasuredRuns = 100;

double milliseconds(std::chrono::nanoseconds duration)
{
    return std::chrono::duration<double, std::milli>(duration).count();
}

double percentile(std::vector<double> values, double fraction)
{
    std::sort(values.begin(), values.end());
    const auto index = static_cast<size_t>(std::ceil(fraction * values.size())) - 1;
    return values[index];
}

nimbus::RollingOccupancyGrid makeRepresentativeMap()
{
    nimbus::RollingOccupancyGrid map;
    constexpr std::array<double, 9> angles{-0.7853981634, -0.5890486225, -0.3926990817,
                                           -0.1963495408, 0.0,           0.1963495408,
                                           0.3926990817,  0.5890486225,  0.7853981634};
    constexpr std::array<float, 9> ranges{2.6F, 2.3F, 2.1F, 1.9F, 1.8F,
                                          1.9F, 2.1F, 2.3F, 2.6F};
    constexpr std::array<bool, 9> hits{true, true, true, true, true,
                                       true, true, true, true};
    const std::vector<double> angle_vector(angles.begin(), angles.end());
    const std::vector<float> range_vector(ranges.begin(), ranges.end());
    const std::vector<bool> hit_vector(hits.begin(), hits.end());
    for (size_t scan = 0; scan < 16; ++scan)
    {
        map.updateScan(0.035 * static_cast<double>(scan), 0.0, angle_vector, range_vector,
                       hit_vector);
    }
    return map;
}

}  // namespace

int main()
{
    const auto map = makeRepresentativeMap();
    const nimbus::MotionState state{0.525, 0.0, 1.5, 0.0};

    nimbus::CemMpcPlanner planner;
    for (size_t run = 0; run < kWarmupRuns; ++run)
    {
        planner.plan(state, map);
    }

    std::vector<double> durations_ms;
    std::vector<double> candidate_counts;
    std::vector<double> sampling_ms;
    std::vector<double> propagation_ms;
    std::vector<double> collision_ms;
    std::vector<double> objective_ms;
    std::vector<double> selection_ms;
    durations_ms.reserve(kMeasuredRuns);
    candidate_counts.reserve(kMeasuredRuns);
    sampling_ms.reserve(kMeasuredRuns);
    propagation_ms.reserve(kMeasuredRuns);
    collision_ms.reserve(kMeasuredRuns);
    objective_ms.reserve(kMeasuredRuns);
    selection_ms.reserve(kMeasuredRuns);
    double checksum = 0.0;
    size_t timeouts = 0;

    for (size_t run = 0; run < kMeasuredRuns; ++run)
    {
        const auto started = std::chrono::steady_clock::now();
        const auto trajectory = planner.plan(state, map);
        const auto elapsed = std::chrono::steady_clock::now() - started;
        durations_ms.push_back(
                std::chrono::duration<double, std::milli>(elapsed).count());
        candidate_counts.push_back(static_cast<double>(trajectory.candidates_evaluated));
        checksum += trajectory.cost;
        timeouts += trajectory.planning_time_exhausted ? 1U : 0U;
    }

    nimbus::CemMpcPlanner instrumented_planner;
    for (size_t run = 0; run < kWarmupRuns; ++run)
    {
        instrumented_planner.planWithTiming(state, map);
    }

    for (size_t run = 0; run < kMeasuredRuns; ++run)
    {
        instrumented_planner.planWithTiming(state, map);
        const auto& timing = instrumented_planner.lastTiming();
        sampling_ms.push_back(milliseconds(timing.control_sampling));
        propagation_ms.push_back(milliseconds(timing.state_propagation));
        collision_ms.push_back(milliseconds(timing.collision_checking));
        objective_ms.push_back(milliseconds(timing.objective_evaluation));
        selection_ms.push_back(milliseconds(timing.selection_and_refit));
    }

    const double mean_candidates =
            std::accumulate(candidate_counts.begin(), candidate_counts.end(), 0.0) /
            static_cast<double>(candidate_counts.size());
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "scenario=representative_clutter\n";
    std::cout << "runs=" << kMeasuredRuns << " warmup_runs=" << kWarmupRuns << "\n";
    std::cout << "latency_ms p50=" << percentile(durations_ms, 0.50)
              << " p95=" << percentile(durations_ms, 0.95)
              << " p99=" << percentile(durations_ms, 0.99) << "\n";
    std::cout << "rollouts mean=" << mean_candidates
              << " p50=" << percentile(candidate_counts, 0.50)
              << " p95=" << percentile(candidate_counts, 0.95) << "\n";
    std::cout << "budget_timeouts=" << timeouts << '/' << kMeasuredRuns << "\n";
    std::cout << "phase_mean_ms sampling="
              << std::accumulate(sampling_ms.begin(), sampling_ms.end(), 0.0) /
                         kMeasuredRuns
              << " propagation="
              << std::accumulate(propagation_ms.begin(), propagation_ms.end(), 0.0) /
                         kMeasuredRuns
              << " collision="
              << std::accumulate(collision_ms.begin(), collision_ms.end(), 0.0) /
                         kMeasuredRuns
              << " objective="
              << std::accumulate(objective_ms.begin(), objective_ms.end(), 0.0) /
                         kMeasuredRuns
              << " selection_refit="
              << std::accumulate(selection_ms.begin(), selection_ms.end(), 0.0) /
                         kMeasuredRuns
              << "\n";
    std::cout << "checksum=" << checksum << "\n";
    return 0;
}
