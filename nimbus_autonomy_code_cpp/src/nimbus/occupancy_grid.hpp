#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "nimbus/distance_field.hpp"
#include "nimbus/planning/planner.hpp"

namespace nimbus
{

/** @brief Per-update map timings exposed only for local performance benchmarking. */
struct MapUpdateTimings
{
    /// Time spent moving the horizontal circular rolling window.
    std::chrono::nanoseconds recentering{};
    /// Time spent ray tracing scan cells into the direct occupancy map.
    std::chrono::nanoseconds ray_integration{};
    /// Time spent finding raw occupied cells.
    std::chrono::nanoseconds occupied_extraction{};
    /// Time spent rebuilding the exact Euclidean distance field.
    std::chrono::nanoseconds distance_field{};
    /// Time spent rebuilding obstacle inflation.
    std::chrono::nanoseconds inflation{};
};

/**
 * @brief Occupancy evidence in a forward-looking, world-aligned local window.
 *
 * The map uses Nimbus's coordinates: x points forward/right and y points up.
 * Each scan shifts only the x window to keep `rear_extent_m` behind the robot
 * and the remaining width ahead. The y extent stays fixed in world coordinates.
 * Evidence inside the overlapping x area is retained; columns that leave the
 * rear or front of the window are cleared. Each scan directly marks traversed
 * cells free and measured hit cells occupied. A second layer
 * inflates obstacles by the bird footprint and safety margin for planning. To
 * keep plans inside the playable world, the fixed top and bottom map rows are
 * always treated as occupied before this inflation. To
 * protect against partially observed pipe edges, each obstacle hit also marks
 * the configured cells farther right and the adjacent cells above and below as
 * occupied.
 * For a newly observed hit, the map also looks three cells left and 0.40 m in
 * either vertical direction; finding another raw obstacle fills that segment.
 */
class RollingOccupancyGrid
{
  public:
    /// Default square-cell resolution in metres for the runtime local map.
    static constexpr double kDefaultResolution = 0.035;
    /// Default horizontal map width in metres.
    static constexpr double kDefaultWidthMeters = 4.5;
    /// Default map width retained behind the bird in metres.
    static constexpr double kDefaultRearExtentMeters = 0.5;
    /// Default bird footprint plus safety margin in metres.
    static constexpr double kDefaultSafetyRadiusMeters = 0.115;
    /// World y coordinate of Nimbus's collision floor in metres.
    static constexpr double kGameFloorY = -1.3548;
    /// World y coordinate of Nimbus's collision ceiling in metres.
    static constexpr double kGameCeilingY = 2.44;
    /// Number of cells conservatively marked ahead of a measured obstacle hit.
    static constexpr int kForwardHitExtensionCells = 4;
    /// Number of cells conservatively marked above and below a measured hit.
    static constexpr int kVerticalHitExtensionCells = 1;
    /// Horizontal offset used when looking for a nearby vertical raw-obstacle link.
    static constexpr int kVerticalConnectionLeftCells = 3;
    /// Maximum vertical raw-obstacle-link search distance in metres.
    static constexpr double kVerticalConnectionSearchMeters = 0.4;

    /**
     * @brief Creates an initially unknown map spanning Nimbus's playable vertical
     * extent.
     *
     * The default map is anchored at the game's floor and ends at its ceiling.
     * It rolls only horizontally as the bird moves forward.
     */
    RollingOccupancyGrid();

    /**
     * @brief Creates an initially unknown generic local map.
     * @param resolution Cell width and height in metres.
     * @param width_m Total x extent of the local window in metres.
     * @param height_m Fixed y extent of the local window in metres.
     * @param rear_extent_m Portion of the x extent retained behind the robot.
     * @param safety_radius_m Bird collision radius plus planning safety margin.
     * @throws std::invalid_argument When dimensions are invalid or the rear
     * extent does not lie inside the map.
     */
    explicit RollingOccupancyGrid(double resolution, double width_m, double height_m,
                                  double rear_extent_m, double safety_radius_m);

    /**
     * @brief Fuses one laser scan into the rolling window.
     *
     * Every input vector has one item per ray. `angles` are in radians in the
     * world-aligned laser frame, `ranges` are in metres, and `hits` distinguishes
     * a measured obstacle from a max-range free-space ray. All vector sizes must
     * match. Invalid or non-positive ranges are ignored.
     */
    void updateScan(double robot_x, double robot_y, const std::vector<double>& angles,
                    const std::vector<float>& ranges, const std::vector<bool>& hits);

    /** @brief Fuses one scan while collecting benchmark-only phase timings. */
    void updateScanWithTiming(double robot_x, double robot_y,
                              const std::vector<double>& angles,
                              const std::vector<float>& ranges,
                              const std::vector<bool>& hits);

    /** @brief Returns timing collected by the most recent `updateScanWithTiming` call. */
    const MapUpdateTimings& lastUpdateTiming() const { return last_update_timing_; }

    /** @brief Returns the cell resolution in metres. */
    double resolution() const { return resolution_; }

    /** @brief Returns the number of cells along x. */
    uint32_t width() const { return width_; }

    /** @brief Returns the number of cells along y. */
    uint32_t height() const { return height_; }

    /** @brief Returns the world x coordinate of the map's lower-left cell edge. */
    double originX() const { return origin_x_; }

    /** @brief Returns the world y coordinate of the map's lower-left cell edge. */
    double originY() const { return origin_y_; }

    /**
     * @brief Returns row-major ROS OccupancyGrid values.
     *
     * Values are -1 for unknown, 0 for free, and 100 for occupied.
     */
    std::vector<int8_t> occupancy() const;

    /**
     * @brief Returns the safety-inflated occupancy layer in ROS map format.
     *
     * The layer contains -1 for unknown, 0 for free, and 100 for cells that are
     * occupied or inside the configured safety radius of an occupied cell.
     */
    std::vector<int8_t> inflatedOccupancy() const;

    /**
     * @brief Returns exact distance from a logical cell to a raw obstacle in metres.
     * @pre `cell_x` and `cell_y` lie within the map dimensions.
     */
    double rawObstacleDistanceAt(int cell_x, int cell_y) const;

    /**
     * @brief Returns whether a world position is unsafe for the bird centre.
     *
     * Positions outside the current local map are treated as collisions because
     * their clearance is unknown.
     */
    bool isCollision(double x, double y) const;

    /**
     * @brief Returns the first safety condition encountered along a rasterised segment.
     *
     * @param ignore_inflation When true, detects raw obstacle cells and map boundaries
     * but ignores conservative inflation-only cells.
     */
    CollisionReason segmentCollisionReason(double start_x, double start_y, double end_x,
                                           double end_y,
                                           bool ignore_inflation = false) const;

    /**
     * @brief Returns conservative distance from a world position to raw obstacles.
     *
     * Returns infinity when no obstacle has been observed in the current window.
     * Positions outside the map return zero because they are not safe to plan into.
     */
    double clearance(double x, double y) const;

  private:
    /// Direct occupancy values, deliberately matching ROS OccupancyGrid semantics.
    static constexpr int8_t kUnknown = -1;
    static constexpr int8_t kFree = 0;
    static constexpr int8_t kOccupied = 100;

    /** @brief Serializes either the raw or inflated layer for ROS visualisation. */
    std::vector<int8_t> serializeOccupancy(bool inflated) const;

    /** @brief Creates a map with an explicit fixed world y origin. */
    RollingOccupancyGrid(double resolution, double width_m, double height_m,
                         double rear_extent_m, double safety_radius_m, double origin_y);

    /** @brief Moves only the horizontal window around the robot while preserving overlap.
     */
    void recenterAround(double x);

    /** @brief Advances horizontal circular storage and clears entering columns. */
    void shift(int shift_x);

    /** @brief Converts world coordinates to integer grid-cell coordinates. */
    std::pair<int, int> worldToCell(double x, double y) const;

    /** @brief Maps a logical grid coordinate to its wrapped storage index. */
    size_t storageIndex(int x, int y) const;

    /** @brief Wraps one integer coordinate into a circular storage extent. */
    static int wrapIndex(int coordinate, int extent);

    /** @brief Marks an in-bounds raw map cell free. */
    void setFree(int x, int y);

    /** @brief Returns whether a mapped cell is a raw obstacle rather than inflated
     * occupancy. */
    bool isRawObstacle(int x, int y) const;

    /** @brief Marks an in-bounds raw map cell occupied. */
    void setOccupied(int x, int y);

    /** @brief Fills raw-obstacle connections found above or below a new hit. */
    void connectNearbyObstacleVertically(int obstacle_x, int obstacle_y);

    /** @brief Fuses a scan with optional benchmark-only phase instrumentation. */
    void updateScanInternal(double robot_x, double robot_y,
                            const std::vector<double>& angles,
                            const std::vector<float>& ranges,
                            const std::vector<bool>& hits, MapUpdateTimings* timing);

    /** @brief Marks the fixed game floor and ceiling as raw obstacles. */
    void markWorldBoundaries();

    /** @brief Extracts raw obstacle cells for distance and inflation rebuilds. */
    void extractOccupiedCells();

    /** @brief Rebuilds collision flags from raw obstacles and the safety radius. */
    void rebuildInflatedCollision();

    /** @brief Rebuilds the conservative planning layer after a map update. */
    void updatePlanningLayer(MapUpdateTimings* timing);

    /// Cell size in metres.
    double resolution_;
    /// Number of columns in the map.
    uint32_t width_;
    /// Number of rows in the map.
    uint32_t height_;
    /// World x coordinate at the lower-left cell edge.
    double origin_x_;
    /// World y coordinate at the lower-left cell edge.
    double origin_y_;
    /// Local window length retained behind the robot in metres.
    double rear_extent_m_;
    /// Bird radius plus safety margin used to inflate obstacles in metres.
    double safety_radius_m_;
    /// Exact raw-obstacle clearance field in logical grid order.
    DistanceField distance_field_;
    /// Circular row-major direct cell states: unknown, free, or occupied.
    std::vector<int8_t> cells_;
    /// Circular row-major collision layer derived from occupied evidence.
    std::vector<bool> inflated_collision_;
    /// Physical storage column corresponding to logical map column zero.
    int storage_offset_x_{0};
    /// Grid coordinates of raw obstacles used by distance and inflation rebuilds.
    std::vector<std::pair<int, int>> occupied_cells_;
    /// Benchmark-only timing from the most recent instrumented map update.
    MapUpdateTimings last_update_timing_{};
};

}  // namespace nimbus
