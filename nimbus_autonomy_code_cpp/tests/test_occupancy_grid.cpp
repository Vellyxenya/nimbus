#include <cmath>

#include <gtest/gtest.h>

#include "nimbus/occupancy_grid.hpp"

TEST(RollingOccupancyGrid, DefaultMapMatchesThePlayableVerticalBounds)
{
    nimbus::RollingOccupancyGrid grid;

    EXPECT_DOUBLE_EQ(grid.originY(), nimbus::RollingOccupancyGrid::kGameFloorY);
    const double map_ceiling = grid.originY() + grid.height() * grid.resolution();
    EXPECT_GE(map_ceiling, nimbus::RollingOccupancyGrid::kGameCeilingY);
    EXPECT_LT(map_ceiling,
              nimbus::RollingOccupancyGrid::kGameCeilingY + grid.resolution());
}

TEST(RollingOccupancyGrid, HitMarksEndpointOccupiedAndRayFree)
{
    nimbus::RollingOccupancyGrid grid(1.0, 10.0, 6.0, 1.0, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {3.0F}, {true});

    const auto occupancy = grid.occupancy();
    const auto cell_index = [&grid](double x, double y)
    {
        const int cell_x =
                static_cast<int>(std::floor((x - grid.originX()) / grid.resolution()));
        const int cell_y =
                static_cast<int>(std::floor((y - grid.originY()) / grid.resolution()));
        return static_cast<size_t>(cell_y) * grid.width() + cell_x;
    };
    EXPECT_EQ(occupancy[cell_index(0.0, 0.0)], 0);
    for (int offset = 0;
         offset <= nimbus::RollingOccupancyGrid::kForwardHitExtensionCells; ++offset)
    {
        EXPECT_EQ(occupancy[cell_index(3.0 + offset * grid.resolution(), 0.0)], 100);
    }
    for (int offset = 1;
         offset <= nimbus::RollingOccupancyGrid::kVerticalHitExtensionCells; ++offset)
    {
        EXPECT_EQ(occupancy[cell_index(3.0, offset * grid.resolution())], 100);
        EXPECT_EQ(occupancy[cell_index(3.0, -offset * grid.resolution())], 100);
    }
}

TEST(RollingOccupancyGrid, MaxRangeRayDoesNotCreateObstacle)
{
    nimbus::RollingOccupancyGrid grid(1.0, 10.0, 6.0, 1.0, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {3.0F}, {false});

    const int endpoint_x =
            static_cast<int>(std::floor((3.0 - grid.originX()) / grid.resolution()));
    const int endpoint_y =
            static_cast<int>(std::floor(-grid.originY() / grid.resolution()));
    EXPECT_EQ(
            grid.occupancy()[static_cast<size_t>(endpoint_y) * grid.width() + endpoint_x],
            0);
}

TEST(RollingOccupancyGrid, ConnectsNewHitToNearbyRawObstacleAboveOrBelowAndLeft)
{
    const auto expect_connection = [](double existing_y, double new_y)
    {
        nimbus::RollingOccupancyGrid grid(0.05, 4.0, 2.0, 0.5, 0.0);
        constexpr double existing_x = 0.925;
        const double new_x =
                existing_x + nimbus::RollingOccupancyGrid::kVerticalConnectionLeftCells *
                                     grid.resolution();
        grid.updateScan(0.0, 0.0, {std::atan2(existing_y, existing_x)},
                        {static_cast<float>(std::hypot(existing_x, existing_y))}, {true});
        grid.updateScan(0.0, 0.0, {std::atan2(new_y, new_x)},
                        {static_cast<float>(std::hypot(new_x, new_y))}, {true});

        const int bridge_x = static_cast<int>(
                std::floor((existing_x - grid.originX()) / grid.resolution()));
        const int bridge_y = static_cast<int>(std::floor(
                ((existing_y + new_y) * 0.5 - grid.originY()) / grid.resolution()));
        EXPECT_EQ(
                grid.occupancy()[static_cast<size_t>(bridge_y) * grid.width() + bridge_x],
                100);
    };

    expect_connection(0.225, 0.025);
    expect_connection(0.025, 0.225);
}

TEST(RollingOccupancyGrid, SegmentCollisionDetectsOccupiedCellBetweenEndpoints)
{
    nimbus::RollingOccupancyGrid grid(0.1, 3.0, 2.0, 0.5, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {0.5F}, {true});

    const double clear_x =
            0.5 + (nimbus::RollingOccupancyGrid::kForwardHitExtensionCells + 4) *
                          grid.resolution();
    EXPECT_FALSE(grid.isCollision(clear_x, 0.0));
    EXPECT_EQ(grid.segmentCollisionReason(0.0, 0.0, clear_x, 0.0),
              nimbus::CollisionReason::kInflatedObstacle);
    EXPECT_EQ(grid.segmentCollisionReason(0.0, 0.0, clear_x, 0.0, true),
              nimbus::CollisionReason::kInflatedObstacle);
}

TEST(RollingOccupancyGrid, SegmentCollisionAllowsClearPath)
{
    nimbus::RollingOccupancyGrid grid(0.1, 3.0, 2.0, 0.5, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {0.5F}, {true});

    EXPECT_EQ(grid.segmentCollisionReason(0.0, 0.5, 1.0, 0.5),
              nimbus::CollisionReason::kNone);
    EXPECT_EQ(grid.segmentCollisionReason(-1.0, 0.0, 0.0, 0.0),
              nimbus::CollisionReason::kMapBoundary);
}

TEST(RollingOccupancyGrid, WindowTracksRobotAndKeepsForwardHistory)
{
    nimbus::RollingOccupancyGrid grid(1.0, 10.0, 6.0, 1.0, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {3.0F}, {true});
    grid.updateScan(1.0, 0.0, {}, {}, {});

    EXPECT_DOUBLE_EQ(grid.originX(), 0.0);
    EXPECT_EQ(grid.occupancy()[3 * grid.width() + 3], 100);
    EXPECT_LT(grid.clearance(3.0, 0.0), grid.resolution());
}

TEST(RollingOccupancyGrid, KeepsVerticalExtentFixedWhileTheRobotMoves)
{
    nimbus::RollingOccupancyGrid grid(1.0, 10.0, 6.0, 1.0, 0.0);
    const double initial_origin_y = grid.originY();
    grid.updateScan(0.0, 0.0, {1.5707963267948966}, {2.0F}, {true});
    grid.updateScan(0.0, 1.0, {}, {}, {});

    EXPECT_DOUBLE_EQ(grid.originY(), initial_origin_y);
    EXPECT_EQ(grid.occupancy()[5 * grid.width() + 1], 100);
}

TEST(RollingOccupancyGrid, AlwaysTreatsTheTopAndBottomMapRowsAsObstacles)
{
    nimbus::RollingOccupancyGrid grid(0.1, 3.0, 2.0, 0.5, 0.0);
    grid.updateScan(0.0, 0.0, {}, {}, {});

    const double x = grid.originX() + 0.5 * grid.resolution();
    const double floor_y = grid.originY() + 0.5 * grid.resolution();
    const double ceiling_y = grid.originY() + (grid.height() - 0.5) * grid.resolution();
    EXPECT_TRUE(grid.isCollision(x, floor_y));
    EXPECT_TRUE(grid.isCollision(x, ceiling_y));
}

TEST(RollingOccupancyGrid, InflatedLayerProvidesCollisionAndClearanceQueries)
{
    nimbus::RollingOccupancyGrid grid(0.1, 2.0, 2.0, 0.2, 0.3);
    grid.updateScan(0.0, 0.0, {0.0}, {0.5F}, {true});

    EXPECT_TRUE(grid.isCollision(0.5, 0.0));
    EXPECT_FALSE(grid.isCollision(-0.15, 0.0));
    EXPECT_LT(grid.clearance(0.5, 0.0), grid.resolution());
    const double farther_x =
            0.5 + (nimbus::RollingOccupancyGrid::kForwardHitExtensionCells + 4) *
                          grid.resolution();
    EXPECT_LT(grid.clearance(0.5, 0.0), grid.clearance(farther_x, 0.0));
    EXPECT_EQ(grid.clearance(-1.0, 0.0), 0.0);
}

TEST(RollingOccupancyGrid, RawObstacleDistanceReportsMetres)
{
    nimbus::RollingOccupancyGrid grid(0.1, 3.0, 2.0, 0.5, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {0.5F}, {true});

    const int obstacle_x =
            static_cast<int>(std::floor((0.5 - grid.originX()) / grid.resolution()));
    const int obstacle_y =
            static_cast<int>(std::floor(-grid.originY() / grid.resolution()));
    const int farther_x =
            static_cast<int>(std::floor((1.4 - grid.originX()) / grid.resolution()));

    EXPECT_DOUBLE_EQ(grid.rawObstacleDistanceAt(obstacle_x, obstacle_y), 0.0);
    EXPECT_LT(grid.rawObstacleDistanceAt(obstacle_x, obstacle_y),
              grid.rawObstacleDistanceAt(farther_x, obstacle_y));
}

TEST(RollingOccupancyGrid, RawObstacleDistanceStaysWorldAlignedAfterRolling)
{
    nimbus::RollingOccupancyGrid grid(0.1, 3.0, 2.0, 0.5, 0.0);
    grid.updateScan(0.0, 0.0, {0.0}, {0.5F}, {true});
    grid.updateScan(0.2, 0.0, {}, {}, {});

    const int obstacle_x =
            static_cast<int>(std::floor((0.5 - grid.originX()) / grid.resolution()));
    const int obstacle_y =
            static_cast<int>(std::floor(-grid.originY() / grid.resolution()));
    EXPECT_DOUBLE_EQ(grid.rawObstacleDistanceAt(obstacle_x, obstacle_y), 0.0);
}
