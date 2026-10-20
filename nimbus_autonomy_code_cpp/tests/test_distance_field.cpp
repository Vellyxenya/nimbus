#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "nimbus/distance_field.hpp"

namespace
{

double bruteForceDistance(int x, int y, const std::vector<std::pair<int, int>>& obstacles)
{
    double nearest_distance_squared = std::numeric_limits<double>::infinity();
    for (const auto& [obstacle_x, obstacle_y] : obstacles)
    {
        const double delta_x = x - obstacle_x;
        const double delta_y = y - obstacle_y;
        nearest_distance_squared =
                std::min(nearest_distance_squared, delta_x * delta_x + delta_y * delta_y);
    }
    return std::sqrt(nearest_distance_squared);
}

void expectExactEuclideanDistances(uint32_t width, uint32_t height,
                                   const std::vector<std::pair<int, int>>& obstacles)
{
    nimbus::DistanceField field(width, height);
    field.rebuild(obstacles);

    for (int y = 0; y < static_cast<int>(height); ++y)
    {
        for (int x = 0; x < static_cast<int>(width); ++x)
        {
            SCOPED_TRACE(::testing::Message() << "cell=(" << x << ", " << y << ')');
            EXPECT_DOUBLE_EQ(field.at(x, y), bruteForceDistance(x, y, obstacles));
        }
    }
}

}  // namespace

TEST(DistanceField, MatchesBruteForceEuclideanDistanceForEveryCell)
{
    const std::vector<std::pair<int, int>> obstacles{{0, 0}, {3, 2}, {6, 5}};
    expectExactEuclideanDistances(7, 6, obstacles);
}

TEST(DistanceField, MatchesBruteForceAcrossEdgeAndInteriorLayouts)
{
    expectExactEuclideanDistances(6, 5, {{5, 2}});
    expectExactEuclideanDistances(6, 5, {{0, 4}, {3, 0}});
    expectExactEuclideanDistances(6, 5, {{1, 1}, {1, 3}, {4, 2}});
}

TEST(DistanceField, RebuildReplacesRatherThanAccumulatesObstacleSites)
{
    nimbus::DistanceField field(5, 5);
    field.rebuild({{0, 0}});
    field.rebuild({{4, 4}});

    EXPECT_DOUBLE_EQ(field.at(4, 4), 0.0);
    EXPECT_DOUBLE_EQ(field.at(0, 0), std::sqrt(32.0));
}

TEST(DistanceField, EmptyRebuildHasInfiniteClearance)
{
    nimbus::DistanceField field(3, 2);
    field.rebuild({{1, 1}});
    field.rebuild({});

    for (int y = 0; y < 2; ++y)
    {
        for (int x = 0; x < 3; ++x)
        {
            EXPECT_TRUE(std::isinf(field.at(x, y)));
        }
    }
}
