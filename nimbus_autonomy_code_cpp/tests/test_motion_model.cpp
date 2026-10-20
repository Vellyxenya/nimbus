#include <gtest/gtest.h>

#include "nimbus/occupancy_grid.hpp"
#include "nimbus/planning/motion_model.hpp"

TEST(MotionModel, PredictivelyLimitsTheNextVelocity)
{
    const double velocity_headroom =
            nimbus::MotionModel::kMaxAccelerationX * nimbus::MotionModel::kDt;
    const nimbus::MotionState initial{
            0.0, 0.0, nimbus::MotionModel::kAutonomyMaxVelocity - velocity_headroom,
            -nimbus::MotionModel::kAutonomyMaxVelocity + velocity_headroom};
    const auto command = nimbus::MotionModel::limitControlForVelocity(
            initial, {nimbus::MotionModel::kMaxAccelerationX,
                      -nimbus::MotionModel::kMaxAccelerationY});

    EXPECT_NEAR(command.ax, nimbus::MotionModel::kMaxAccelerationX, 1e-12);
    EXPECT_NEAR(command.ay, -velocity_headroom / nimbus::MotionModel::kDt, 1e-12);

    nimbus::RollingOccupancyGrid map(0.1, 3.0, 2.0, 0.5, 0.0);
    map.updateScan(0.0, 0.0, {}, {}, {});
    const auto trajectory =
            nimbus::MotionModel::simulate(initial,
                                          {{nimbus::MotionModel::kMaxAccelerationX,
                                            -nimbus::MotionModel::kMaxAccelerationY}},
                                          map);
    EXPECT_NEAR(trajectory.vx[1], nimbus::MotionModel::kAutonomyMaxVelocity, 1e-12);
    EXPECT_NEAR(trajectory.vy[1], -nimbus::MotionModel::kAutonomyMaxVelocity, 1e-12);
}

TEST(MotionModel, AllowsAPlanToLeaveInitialInflatedOccupancy)
{
    nimbus::RollingOccupancyGrid map(0.1, 3.0, 2.0, 0.5, 0.16);
    map.updateScan(0.0, 0.0, {0.0}, {0.3F}, {true});
    ASSERT_TRUE(map.isCollision(0.0, 0.0));

    const auto trajectory = nimbus::MotionModel::simulate(
            {0.0, 0.0, 0.0, 2.0}, std::vector<nimbus::AccelerationCommand>(5), map);

    EXPECT_FALSE(trajectory.collided);
    EXPECT_FALSE(map.isCollision(trajectory.x.back(), trajectory.y.back()));
}
