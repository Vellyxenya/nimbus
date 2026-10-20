#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <vector>

namespace nimbus
{

class RollingOccupancyGrid;
struct ObjectiveWeights;

/** @brief Maximum wall-clock time allocated to one real-time MPC optimisation. */
inline constexpr std::chrono::milliseconds kMpcPlanningBudget{20};

/** @brief Reason why a predicted trajectory first became unsafe. */
enum class CollisionReason
{
    /// No collision was detected.
    kNone,
    /// The trajectory crossed a raw obstacle or conservative inflated obstacle cell.
    kInflatedObstacle,
    /// The trajectory left the current local-map window.
    kMapBoundary,
};

/** @brief Planar state used by the discrete Nimbus motion model. */
struct MotionState
{
    /// Map-frame x position in metres.
    double x{0.0};
    /// Map-frame y position in metres.
    double y{0.0};
    /// Map-frame x velocity in metres per second, physically constrained to [0, 10].
    double vx{0.0};
    /// Map-frame y velocity in metres per second, physically constrained to [-10, 10].
    double vy{0.0};
};

/** @brief Acceleration command in metres per second squared. */
struct AccelerationCommand
{
    /// Forward acceleration.
    double ax{0.0};
    /// Vertical acceleration.
    double ay{0.0};
};

/**
 * @brief One simulated candidate and the score used to compare candidates.
 *
 * State history intentionally uses Structure-of-Arrays (SoA): `x[k]`, `y[k]`,
 * `vx[k]`, and `vy[k]`, rather than the former `MotionState states[k]` AoS
 * layout. This makes field-wise trajectory passes and later SIMD experiments
 * possible without changing the planner interface again.
 */
struct PlannedTrajectory
{
    /// Predicted map-frame x positions, including the initial state at index zero.
    std::vector<double> x;
    /// Predicted map-frame y positions, including the initial state at index zero.
    std::vector<double> y;
    /// Predicted x velocities, including the initial state at index zero.
    std::vector<double> vx;
    /// Predicted y velocities, including the initial state at index zero.
    std::vector<double> vy;
    /// Commands applied between successive states.
    std::vector<AccelerationCommand> controls;
    /// Lower scores are preferred.
    double cost{0.0};
    /// True when any predicted state enters the inflated map or leaves the map.
    bool collided{false};
    /// One-based control index of the first collision; zero when collision-free.
    size_t first_collision_step{0};
    /// Cause of the first collision; `kNone` for a feasible trajectory.
    CollisionReason collision_reason{CollisionReason::kNone};
    /// True when this trajectory is the deterministic safety-braking fallback.
    bool is_emergency_fallback{false};
    /// True when planning began inside inflated occupancy and returns the least-cost
    /// sample.
    bool is_initial_state_recovery{false};
    /// True when CEM stopped sampling because its real-time planning budget elapsed.
    bool planning_time_exhausted{false};
    /// Number of CEM candidate rollouts evaluated while producing this trajectory.
    size_t candidates_evaluated{0};
};

/** @brief Available planning implementations. */
enum class PlannerType
{
    /// Sampling baseline used to validate the model and map integration.
    kRandomShooting,
    /// Cross-Entropy Method optimiser used in the receding-horizon MPC loop.
    kCemMpc,
};

/** @brief Common interface implemented by every trajectory planner. */
class Planner
{
  public:
    virtual ~Planner() = default;

    /** @brief Produces the best trajectory for the supplied state and local map. */
    virtual PlannedTrajectory plan(const MotionState& initial,
                                   const RollingOccupancyGrid& map) = 0;
};

/** @brief Constructs planners without exposing concrete planner types to callers. */
class PlannerFactory
{
  public:
    /** @brief Returns a newly constructed planner for the requested implementation. */
    static std::unique_ptr<Planner> create(PlannerType type);

    /** @brief Returns a planner configured with the supplied trajectory objective. */
    static std::unique_ptr<Planner> create(PlannerType type,
                                           const ObjectiveWeights& objective_weights);
};

}  // namespace nimbus
