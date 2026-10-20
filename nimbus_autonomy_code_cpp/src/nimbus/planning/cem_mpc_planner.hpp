#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

#include "nimbus/planning/motion_model.hpp"
#include "nimbus/planning/planner.hpp"
#include "nimbus/planning/trajectory_objective.hpp"

namespace nimbus
{

/** @brief Per-plan phase timings exposed only for local performance benchmarking. */
struct CemMpcPhaseTimings
{
    /// Time spent drawing Gaussian control sequences.
    std::chrono::nanoseconds control_sampling{};
    /// Time spent governing controls and propagating candidate state vectors.
    std::chrono::nanoseconds state_propagation{};
    /// Time spent checking candidate segments against the inflated map.
    std::chrono::nanoseconds collision_checking{};
    /// Time spent evaluating objective terms, including map-clearance queries.
    std::chrono::nanoseconds objective_evaluation{};
    /// Time spent filtering, sorting, selecting elites, and refitting CEM.
    std::chrono::nanoseconds selection_and_refit{};
    /// Number of candidate trajectories measured in these phases.
    size_t candidates_evaluated{0};
};

/** @brief Tunable dimensions and random seed for the CEM MPC optimiser. */
struct CemMpcParameters
{
    /// Number of 30 Hz controls optimised in one MPC update.
    size_t horizon{40};
    /// Gaussian candidates evaluated in each CEM iteration.
    size_t sample_count{400};
    /// Lowest-cost candidates retained to refit the Gaussian distribution.
    size_t elite_count{50};
    /// Number of sample/refit iterations performed per MPC update.
    size_t iterations{3};
    /// Random seed for repeatable visual debugging and benchmarks.
    uint32_t random_seed{42};
    /// Objective used to score candidate trajectories.
    ObjectiveWeights objective_weights{};
};

/**
 * @brief Cross-Entropy Method optimiser executed in a receding-horizon MPC loop.
 *
 * Each call samples acceleration sequences from a time-indexed Gaussian,
 * evaluates them with the shared motion model and objective, refits the Gaussian
 * to elite candidates, then shifts that distribution forward for the next call.
 */
class CemMpcPlanner : public Planner
{
  public:
    /**
     * @brief Creates a deterministic CEM MPC planner.
     * @param parameters Documented CEM dimensions and deterministic random seed.
     */
    explicit CemMpcPlanner(CemMpcParameters parameters = {});

    /** @brief Optimises a trajectory, then warm-starts the next MPC update. */
    PlannedTrajectory plan(const MotionState& initial,
                           const RollingOccupancyGrid& map) override;

    /** @brief Optimises once while collecting per-phase timings for the benchmark
     * harness. */
    PlannedTrajectory planWithTiming(const MotionState& initial,
                                     const RollingOccupancyGrid& map);

    /** @brief Returns timing collected by the most recent `planWithTiming` call. */
    const CemMpcPhaseTimings& lastTiming() const { return last_timing_; }

  private:
    /** @brief Initialises a neutral forward-moving Gaussian for the first plan call. */
    void initialiseDistribution();

    /** @brief Draws one bounded acceleration sequence from the current Gaussian. */
    std::vector<AccelerationCommand> sampleControls();

    /** @brief Fits the mean and standard deviation to the current elite set. */
    void refitDistribution(const std::vector<PlannedTrajectory>& elites);

    /** @brief Shifts the fitted distribution past executed controls for warm-starting. */
    void shiftDistribution(size_t executed_controls);

    /** @brief Creates a deterministic braking and vertical-velocity damping sequence. */
    std::vector<AccelerationCommand> emergencyBrakeControls(
            const MotionState& initial) const;

    /**
     * @brief Draws and scores one CEM batch until the batch or planning budget is
     * exhausted.
     */
    std::vector<PlannedTrajectory> sampleAndEvaluateCandidates(
            const MotionState& initial, const RollingOccupancyGrid& map,
            std::chrono::steady_clock::time_point planning_started,
            bool& planning_time_exhausted, size_t& candidates_evaluated,
            CemMpcPhaseTimings* timing);

    /**
     * @brief Retains the best feasible plan and refits CEM from the chosen elite set.
     *
     * Feasible samples always take precedence. If the current state is already
     * inflated, the least-cost sample is also retained as a recovery candidate.
     */
    void selectElitesAndRefit(std::vector<PlannedTrajectory>& candidates,
                              bool initial_state_is_illegal,
                              PlannedTrajectory& best_feasible,
                              PlannedTrajectory& best_initial_state_recovery,
                              CemMpcPhaseTimings* timing);

    /** @brief Implements a plan with optional benchmark-only phase instrumentation. */
    PlannedTrajectory planInternal(const MotionState& initial,
                                   const RollingOccupancyGrid& map,
                                   CemMpcPhaseTimings* timing);

    /// Immutable optimiser dimensions and seed selected at construction.
    CemMpcParameters parameters_;
    /// Seeded random source used for Gaussian samples.
    std::mt19937 random_engine_;
    /// Reused unit-normal distribution; scaling produces each step's configured Gaussian.
    std::normal_distribution<double> standard_normal_distribution_{0.0, 1.0};
    /// Shared objective used for every candidate.
    TrajectoryObjective objective_;
    /// Per-step Gaussian control means, retained between MPC updates.
    std::vector<AccelerationCommand> means_;
    /// Per-step Gaussian control standard deviations, retained between updates.
    std::vector<AccelerationCommand> standard_deviations_;
    /// Last governed control selected for the real game, used to suppress inter-plan
    /// wobble.
    AccelerationCommand previous_applied_control_{};
    /// Benchmark-only timing from the most recent instrumented plan.
    CemMpcPhaseTimings last_timing_{};
};

}  // namespace nimbus
