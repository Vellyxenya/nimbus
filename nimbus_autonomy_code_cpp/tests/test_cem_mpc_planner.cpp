#include <cmath>

#include <gtest/gtest.h>

#include "nimbus/occupancy_grid.hpp"
#include "nimbus/planning/cem_mpc_planner.hpp"
#include "nimbus/planning/motion_model.hpp"

TEST(CemMpcPlanner, FindsSafeBoundedTrajectoryInEmptyMap)
{
    nimbus::RollingOccupancyGrid map(0.1, 3.0, 2.0, 0.5, 0.16);
    map.updateScan(0.0, 0.0, {}, {}, {});
    nimbus::CemMpcParameters parameters;
    parameters.horizon = 12;
    parameters.sample_count = 100;
    parameters.elite_count = 10;
    parameters.iterations = 3;
    parameters.random_seed = 7;
    nimbus::CemMpcPlanner planner(parameters);

    const auto trajectory = planner.plan({0.0, 0.0, 1.0, 0.0}, map);

    EXPECT_FALSE(trajectory.collided);
    ASSERT_EQ(trajectory.controls.size(), 12U);
    for (const auto& control : trajectory.controls)
    {
        EXPECT_LE(std::abs(control.ax), nimbus::MotionModel::kMaxAccelerationX);
        EXPECT_LE(std::abs(control.ay), nimbus::MotionModel::kMaxAccelerationY);
    }
}

TEST(CemMpcPlanner, ReturnsBestEffortCandidateWhenInitialStateIsAlreadyUnsafe)
{
    nimbus::RollingOccupancyGrid map(0.1, 1.0, 0.2, 0.5, 0.16);
    map.updateScan(0.0, 0.0, {0.0}, {0.2F}, {true});
    nimbus::CemMpcParameters parameters;
    parameters.horizon = 6;
    parameters.sample_count = 80;
    parameters.elite_count = 8;
    parameters.iterations = 2;
    parameters.random_seed = 7;
    nimbus::CemMpcPlanner planner(parameters);

    const auto trajectory = planner.plan({0.0, 0.0, 2.0, 0.0}, map);

    EXPECT_TRUE(trajectory.collided);
    EXPECT_TRUE(map.isCollision(0.0, 0.0));
    EXPECT_FALSE(trajectory.is_emergency_fallback);
    EXPECT_TRUE(trajectory.is_initial_state_recovery);
    EXPECT_GT(trajectory.first_collision_step, 0U);
}

TEST(CemMpcPlanner, RetainsEmergencyBrakeForFutureOnlyCollision)
{
    nimbus::RollingOccupancyGrid map(0.1, 1.0, 2.0, 0.5, 0.0);
    map.updateScan(0.0, 0.0, {}, {}, {});
    ASSERT_FALSE(map.isCollision(0.0, 0.0));
    nimbus::CemMpcParameters parameters;
    parameters.horizon = 6;
    parameters.sample_count = 80;
    parameters.elite_count = 8;
    parameters.iterations = 2;
    parameters.random_seed = 7;
    nimbus::CemMpcPlanner planner(parameters);

    const auto trajectory = planner.plan({0.0, 0.0, 10.0, 0.0}, map);

    EXPECT_TRUE(trajectory.collided);
    EXPECT_TRUE(trajectory.is_emergency_fallback);
    EXPECT_FALSE(trajectory.is_initial_state_recovery);
    EXPECT_GT(trajectory.first_collision_step, 1U);
}
