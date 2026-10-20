#include "nimbus/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace nimbus
{

namespace
{

uint32_t cellCount(double length, double resolution)
{
    if (length <= 0.0 || resolution <= 0.0)
    {
        throw std::invalid_argument("map dimensions and resolution must be positive");
    }
    return static_cast<uint32_t>(std::ceil(length / resolution));
}

}  // namespace

RollingOccupancyGrid::RollingOccupancyGrid()
    : RollingOccupancyGrid(kDefaultResolution, kDefaultWidthMeters,
                           kGameCeilingY - kGameFloorY, kDefaultRearExtentMeters,
                           kDefaultSafetyRadiusMeters, kGameFloorY)
{
}

RollingOccupancyGrid::RollingOccupancyGrid(double resolution, double width_m,
                                           double height_m, double rear_extent_m,
                                           double safety_radius_m)
    : RollingOccupancyGrid(resolution, width_m, height_m, rear_extent_m, safety_radius_m,
                           -0.5 * height_m)
{
}

RollingOccupancyGrid::RollingOccupancyGrid(double resolution, double width_m,
                                           double height_m, double rear_extent_m,
                                           double safety_radius_m, double origin_y)
    : resolution_(resolution),
      width_(cellCount(width_m, resolution)),
      height_(cellCount(height_m, resolution)),
      origin_x_(-0.5 * width_ * resolution_),
      origin_y_(origin_y),
      rear_extent_m_(rear_extent_m),
      safety_radius_m_(safety_radius_m),
      distance_field_(width_, height_),
      cells_(width_ * height_, kUnknown),
      inflated_collision_(width_ * height_, false)
{
    if (rear_extent_m < 0.0 || rear_extent_m >= width_ * resolution_ ||
        safety_radius_m < 0.0)
    {
        throw std::invalid_argument("rear extent and safety radius must be valid");
    }
}

void RollingOccupancyGrid::updateScan(double robot_x, double robot_y,
                                      const std::vector<double>& angles,
                                      const std::vector<float>& ranges,
                                      const std::vector<bool>& hits)
{
    updateScanInternal(robot_x, robot_y, angles, ranges, hits, nullptr);
}

void RollingOccupancyGrid::updateScanWithTiming(double robot_x, double robot_y,
                                                const std::vector<double>& angles,
                                                const std::vector<float>& ranges,
                                                const std::vector<bool>& hits)
{
    last_update_timing_ = {};
    updateScanInternal(robot_x, robot_y, angles, ranges, hits, &last_update_timing_);
}

void RollingOccupancyGrid::updateScanInternal(double robot_x, double robot_y,
                                              const std::vector<double>& angles,
                                              const std::vector<float>& ranges,
                                              const std::vector<bool>& hits,
                                              MapUpdateTimings* timing)
{
    if (angles.size() != ranges.size() || ranges.size() != hits.size())
    {
        throw std::invalid_argument("scan fields must have equal lengths");
    }
    if (timing != nullptr)
    {
        const auto phase_started = std::chrono::steady_clock::now();
        recenterAround(robot_x);
        timing->recentering += std::chrono::steady_clock::now() - phase_started;
    }
    else
    {
        recenterAround(robot_x);
    }

    // The grid rolls only along x, so the robot's y position is interpreted in
    // the fixed game-coordinate vertical extent.
    const auto [start_x, start_y] = worldToCell(robot_x, robot_y);
    const auto ray_integration_started = timing != nullptr ?
                                                 std::chrono::steady_clock::now() :
                                                 std::chrono::steady_clock::time_point{};
    for (size_t index = 0; index < ranges.size(); ++index)
    {
        const double distance = ranges[index];
        if (!std::isfinite(distance) || distance <= 0.0)
        {
            continue;
        }
        const auto [end_x, end_y] =
                worldToCell(robot_x + distance * std::cos(angles[index]),
                            robot_y + distance * std::sin(angles[index]));

        // Bresenham enumerates the discrete cells crossed by the continuous
        // laser ray. Every traversed cell is marked free; only the endpoint
        // of a true hit is withheld from that free update.
        int cell_x = start_x;
        int cell_y = start_y;
        const int delta_x = std::abs(end_x - cell_x);
        const int delta_y = std::abs(end_y - cell_y);
        const int step_x = cell_x < end_x ? 1 : -1;
        const int step_y = cell_y < end_y ? 1 : -1;
        int error = delta_x - delta_y;
        while (true)
        {
            const bool endpoint = cell_x == end_x && cell_y == end_y;
            if (!hits[index] || !endpoint)
            {
                setFree(cell_x, cell_y);
            }
            if (endpoint)
            {
                break;
            }
            const int twice_error = 2 * error;
            if (twice_error > -delta_y)
            {
                error -= delta_y;
                cell_x += step_x;
            }
            if (twice_error < delta_x)
            {
                error += delta_x;
                cell_y += step_y;
            }
        }
        if (hits[index])
        {
            const bool is_new_obstacle = !isRawObstacle(end_x, end_y);

            // A hit may land on a partially observed pipe edge. Extend the
            // raw obstacle slightly right and vertically before inflating it
            // so planning does not cut through that incomplete observation.
            for (int forward_cell = 0; forward_cell <= kForwardHitExtensionCells;
                 ++forward_cell)
            {
                setOccupied(end_x + forward_cell, end_y);
            }
            for (int vertical_cell = 1; vertical_cell <= kVerticalHitExtensionCells;
                 ++vertical_cell)
            {
                setOccupied(end_x, end_y - vertical_cell);
                setOccupied(end_x, end_y + vertical_cell);
                setOccupied(end_x + kForwardHitExtensionCells, end_y - vertical_cell);
                setOccupied(end_x + kForwardHitExtensionCells, end_y + vertical_cell);
            }
            if (is_new_obstacle)
            {
                connectNearbyObstacleVertically(end_x, end_y);
            }
        }
    }

    if (timing != nullptr)
    {
        timing->ray_integration +=
                std::chrono::steady_clock::now() - ray_integration_started;
    }
    updatePlanningLayer(timing);
}

std::vector<int8_t> RollingOccupancyGrid::occupancy() const
{
    return serializeOccupancy(false);
}

std::vector<int8_t> RollingOccupancyGrid::inflatedOccupancy() const
{
    return serializeOccupancy(true);
}

double RollingOccupancyGrid::rawObstacleDistanceAt(int cell_x, int cell_y) const
{
    return distance_field_.at(cell_x, cell_y) * resolution_;
}

std::vector<int8_t> RollingOccupancyGrid::serializeOccupancy(bool inflated) const
{
    std::vector<int8_t> result(cells_.size(), kUnknown);
    for (int y = 0; y < static_cast<int>(height_); ++y)
    {
        for (int x = 0; x < static_cast<int>(width_); ++x)
        {
            const size_t logical_index = static_cast<size_t>(y) * width_ + x;
            result[logical_index] = cells_[storageIndex(x, y)];
            if (inflated && inflated_collision_[storageIndex(x, y)])
            {
                result[logical_index] = kOccupied;
            }
        }
    }
    return result;
}

bool RollingOccupancyGrid::isCollision(double x, double y) const
{
    const auto [cell_x, cell_y] = worldToCell(x, y);
    if (cell_x < 0 || cell_x >= static_cast<int>(width_) || cell_y < 0 ||
        cell_y >= static_cast<int>(height_))
    {
        return true;
    }
    return inflated_collision_[storageIndex(cell_x, cell_y)];
}

CollisionReason RollingOccupancyGrid::segmentCollisionReason(double start_x,
                                                             double start_y, double end_x,
                                                             double end_y,
                                                             bool ignore_inflation) const
{
    auto [cell_x, cell_y] = worldToCell(start_x, start_y);
    const auto [end_cell_x, end_cell_y] = worldToCell(end_x, end_y);
    const int delta_x = std::abs(end_cell_x - cell_x);
    const int delta_y = std::abs(end_cell_y - cell_y);
    const int step_x = cell_x < end_cell_x ? 1 : -1;
    const int step_y = cell_y < end_cell_y ? 1 : -1;
    int error = delta_x - delta_y;
    while (true)
    {
        if (cell_x < 0 || cell_x >= static_cast<int>(width_) || cell_y < 0 ||
            cell_y >= static_cast<int>(height_))
        {
            return CollisionReason::kMapBoundary;
        }
        if (isRawObstacle(cell_x, cell_y) ||
            (!ignore_inflation && inflated_collision_[storageIndex(cell_x, cell_y)]))
        {
            return CollisionReason::kInflatedObstacle;
        }
        if (cell_x == end_cell_x && cell_y == end_cell_y)
        {
            return CollisionReason::kNone;
        }
        const int twice_error = 2 * error;
        if (twice_error > -delta_y)
        {
            error -= delta_y;
            cell_x += step_x;
        }
        if (twice_error < delta_x)
        {
            error += delta_x;
            cell_y += step_y;
        }
    }
}

double RollingOccupancyGrid::clearance(double x, double y) const
{
    const auto [cell_x, cell_y] = worldToCell(x, y);
    if (cell_x < 0 || cell_x >= static_cast<int>(width_) || cell_y < 0 ||
        cell_y >= static_cast<int>(height_))
    {
        return 0.0;
    }
    const double cell_radius = 0.5 * std::sqrt(2.0) * resolution_;
    return std::max(0.0, distance_field_.at(cell_x, cell_y) * resolution_ - cell_radius);
}

void RollingOccupancyGrid::recenterAround(double x)
{
    const double desired_origin_x = x - rear_extent_m_;
    const int shift_x =
            static_cast<int>(std::round((desired_origin_x - origin_x_) / resolution_));
    if (shift_x != 0)
    {
        shift(shift_x);
    }
}

void RollingOccupancyGrid::shift(int shift_x)
{
    origin_x_ += shift_x * resolution_;
    // Advance circular storage instead of copying every overlapping column.
    // storage_offset_x_ maps logical map x=0 to its current physical column.
    storage_offset_x_ = wrapIndex(storage_offset_x_ + shift_x, static_cast<int>(width_));

    if (std::abs(shift_x) >= static_cast<int>(width_))
    {
        std::fill(cells_.begin(), cells_.end(), kUnknown);
        return;
    }

    const int first_new_x = shift_x > 0 ? static_cast<int>(width_) - shift_x : 0;
    const int last_new_x = shift_x > 0 ? static_cast<int>(width_) : -shift_x;
    for (int y = 0; y < static_cast<int>(height_); ++y)
    {
        for (int x = first_new_x; x < last_new_x; ++x)
        {
            cells_[storageIndex(x, y)] = kUnknown;
        }
    }
}

std::pair<int, int> RollingOccupancyGrid::worldToCell(double x, double y) const
{
    return {static_cast<int>(std::floor((x - origin_x_) / resolution_)),
            static_cast<int>(std::floor((y - origin_y_) / resolution_))};
}

size_t RollingOccupancyGrid::storageIndex(int x, int y) const
{
    // Logical coordinates stay world-aligned while only physical x storage
    // wraps as the map window moves forward.
    const auto physical_x = wrapIndex(x + storage_offset_x_, static_cast<int>(width_));
    return static_cast<size_t>(y) * width_ + physical_x;
}

int RollingOccupancyGrid::wrapIndex(int coordinate, int extent)
{
    const int remainder = coordinate % extent;
    return remainder < 0 ? remainder + extent : remainder;
}

void RollingOccupancyGrid::setFree(int x, int y)
{
    if (x < 0 || x >= static_cast<int>(width_) || y < 0 || y >= static_cast<int>(height_))
    {
        return;
    }
    cells_[storageIndex(x, y)] = kFree;
}

bool RollingOccupancyGrid::isRawObstacle(int x, int y) const
{
    return x >= 0 && x < static_cast<int>(width_) && y >= 0 &&
           y < static_cast<int>(height_) && cells_[storageIndex(x, y)] == kOccupied;
}

void RollingOccupancyGrid::setOccupied(int x, int y)
{
    if (x < 0 || x >= static_cast<int>(width_) || y < 0 || y >= static_cast<int>(height_))
    {
        return;
    }
    cells_[storageIndex(x, y)] = kOccupied;
}

void RollingOccupancyGrid::connectNearbyObstacleVertically(int obstacle_x, int obstacle_y)
{
    // Hits from one pipe can be vertically separated by a short unobserved
    // section. Inspect the column just left of the hit and close such a small
    // break, without ever filling the actual pipe gap.
    const int scan_x = obstacle_x - kVerticalConnectionLeftCells;
    if (scan_x < 0 || scan_x >= static_cast<int>(width_) || obstacle_y < 0 ||
        obstacle_y >= static_cast<int>(height_))
    {
        return;
    }

    const int scan_cells =
            static_cast<int>(std::ceil(kVerticalConnectionSearchMeters / resolution_));
    const auto connect_direction = [this, scan_x, obstacle_y, scan_cells](int direction)
    {
        const int last_scan_y = direction > 0 ? std::min(obstacle_y + scan_cells,
                                                         static_cast<int>(height_) - 1) :
                                                std::max(obstacle_y - scan_cells, 0);
        for (int scan_y = obstacle_y + direction;
             direction > 0 ? scan_y <= last_scan_y : scan_y >= last_scan_y;
             scan_y += direction)
        {
            if (!isRawObstacle(scan_x, scan_y))
            {
                continue;
            }
            const int first_fill_y = std::min(obstacle_y, scan_y);
            const int last_fill_y = std::max(obstacle_y, scan_y);
            for (int fill_y = first_fill_y; fill_y <= last_fill_y; ++fill_y)
            {
                setOccupied(scan_x, fill_y);
            }
            return;
        }
    };
    connect_direction(1);
    connect_direction(-1);
}

void RollingOccupancyGrid::markWorldBoundaries()
{
    // The grid's fixed vertical extent coincides with the playable world, so
    // its floor and ceiling must remain obstacles even without laser returns.
    for (int x = 0; x < static_cast<int>(width_); ++x)
    {
        setOccupied(x, 0);
        setOccupied(x, static_cast<int>(height_) - 1);
    }
}

void RollingOccupancyGrid::extractOccupiedCells()
{
    // Keep the source list in logical coordinates: DistanceField and inflation
    // should not need to know that the underlying map columns are circular.
    occupied_cells_.clear();
    for (int y = 0; y < static_cast<int>(height_); ++y)
    {
        for (int x = 0; x < static_cast<int>(width_); ++x)
        {
            if (cells_[storageIndex(x, y)] == kOccupied)
            {
                occupied_cells_.emplace_back(x, y);
            }
        }
    }
}

void RollingOccupancyGrid::rebuildInflatedCollision()
{
    std::fill(inflated_collision_.begin(), inflated_collision_.end(), false);
    const double cell_radius = 0.5 * std::sqrt(2.0) * resolution_;
    // Add one full grid cell beyond the configured footprint and safety margin
    // to remain conservative when a circular bird overlaps square map cells.
    const double inflation_radius = safety_radius_m_ + cell_radius + resolution_;
    const int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));
    for (const auto& [occupied_x, occupied_y] : occupied_cells_)
    {
        for (int offset_y = -radius_cells; offset_y <= radius_cells; ++offset_y)
        {
            for (int offset_x = -radius_cells; offset_x <= radius_cells; ++offset_x)
            {
                if (std::hypot(offset_x * resolution_, offset_y * resolution_) >
                    inflation_radius)
                {
                    continue;
                }
                const int x = occupied_x + offset_x;
                const int y = occupied_y + offset_y;
                if (x >= 0 && x < static_cast<int>(width_) && y >= 0 &&
                    y < static_cast<int>(height_))
                {
                    inflated_collision_[storageIndex(x, y)] = true;
                }
            }
        }
    }
}

void RollingOccupancyGrid::updatePlanningLayer(MapUpdateTimings* timing)
{
    // Derived planning data is rebuilt after all scan evidence has been fused:
    // raw obstacles -> exact clearance field -> conservative collision layer.
    markWorldBoundaries();

    const auto occupied_extraction_started =
            timing != nullptr ? std::chrono::steady_clock::now() :
                                std::chrono::steady_clock::time_point{};
    extractOccupiedCells();
    if (timing != nullptr)
    {
        timing->occupied_extraction +=
                std::chrono::steady_clock::now() - occupied_extraction_started;
    }

    if (timing != nullptr)
    {
        const auto phase_started = std::chrono::steady_clock::now();
        distance_field_.rebuild(occupied_cells_);
        timing->distance_field += std::chrono::steady_clock::now() - phase_started;
    }
    else
    {
        distance_field_.rebuild(occupied_cells_);
    }

    const auto inflation_started = timing != nullptr ?
                                           std::chrono::steady_clock::now() :
                                           std::chrono::steady_clock::time_point{};
    rebuildInflatedCollision();
    if (timing != nullptr)
    {
        timing->inflation += std::chrono::steady_clock::now() - inflation_started;
    }
}

}  // namespace nimbus
