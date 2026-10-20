#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace nimbus
{

/**
 * @brief Reusable exact Euclidean distance field over a fixed logical grid.
 *
 * The field owns its transform workspaces so rebuilding it from a set of raw
 * obstacle cells does not allocate during map updates.
 */
class DistanceField
{
  public:
    /** @brief Creates an empty field with the specified logical grid dimensions. */
    DistanceField(uint32_t width, uint32_t height);

    /**
     * @brief Rebuilds distances in cell units from the supplied obstacle cells.
     * @pre Every supplied cell lies within the field dimensions.
     */
    void rebuild(const std::vector<std::pair<int, int>>& occupied_cells);

    /**
     * @brief Returns the distance in cell units at a logical cell.
     * @pre `x` and `y` lie within the field dimensions.
     */
    double at(int x, int y) const;

  private:
    /** @brief Applies a one-dimensional squared Euclidean distance transform. */
    void squaredDistanceTransform1D(size_t count);

    /// Logical grid width in cells.
    uint32_t width_;
    /// Logical grid height in cells.
    uint32_t height_;
    /// Raw-obstacle mask in logical row-major order.
    std::vector<uint8_t> occupied_;
    /// Final distances in logical row-major order, measured in cells.
    std::vector<double> distances_;
    /// Row-pass output retained for the column pass.
    std::vector<double> intermediate_;
    /// Reused input workspace for one one-dimensional transform.
    std::vector<double> transform_input_;
    /// Reused output workspace for one one-dimensional transform.
    std::vector<double> transform_output_;
    /// Reused lower-envelope site indices for one one-dimensional transform.
    std::vector<int> transform_sites_;
    /// Reused lower-envelope breakpoints for one one-dimensional transform.
    std::vector<double> transform_breakpoints_;
};

}  // namespace nimbus
