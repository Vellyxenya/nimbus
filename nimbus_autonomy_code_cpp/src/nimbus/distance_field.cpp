#include "nimbus/distance_field.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nimbus
{

DistanceField::DistanceField(uint32_t width, uint32_t height)
    : width_(width),
      height_(height),
      occupied_(static_cast<size_t>(width_) * height_, 0),
      distances_(static_cast<size_t>(width_) * height_,
                 std::numeric_limits<double>::infinity()),
      intermediate_(static_cast<size_t>(width_) * height_),
      transform_input_(std::max(width_, height_)),
      transform_output_(std::max(width_, height_)),
      transform_sites_(std::max(width_, height_)),
      transform_breakpoints_(std::max(width_, height_) + 1)
{
}

void DistanceField::rebuild(const std::vector<std::pair<int, int>>& occupied_cells)
{
    std::fill(occupied_.begin(), occupied_.end(), 0);
    if (occupied_cells.empty())
    {
        std::fill(distances_.begin(), distances_.end(),
                  std::numeric_limits<double>::infinity());
        return;
    }

    for (const auto& [x, y] : occupied_cells)
    {
        occupied_[static_cast<size_t>(y) * width_ + x] = 1;
    }

    const auto infinity = std::numeric_limits<double>::infinity();
    for (int y = 0; y < static_cast<int>(height_); ++y)
    {
        for (int x = 0; x < static_cast<int>(width_); ++x)
        {
            transform_input_[x] =
                    occupied_[static_cast<size_t>(y) * width_ + x] ? 0.0 : infinity;
        }
        squaredDistanceTransform1D(width_);
        for (int x = 0; x < static_cast<int>(width_); ++x)
        {
            intermediate_[static_cast<size_t>(y) * width_ + x] = transform_output_[x];
        }
    }

    for (int x = 0; x < static_cast<int>(width_); ++x)
    {
        for (int y = 0; y < static_cast<int>(height_); ++y)
        {
            transform_input_[y] = intermediate_[static_cast<size_t>(y) * width_ + x];
        }
        squaredDistanceTransform1D(height_);
        for (int y = 0; y < static_cast<int>(height_); ++y)
        {
            distances_[static_cast<size_t>(y) * width_ + x] =
                    std::sqrt(transform_output_[y]);
        }
    }
}

double DistanceField::at(int x, int y) const
{
    return distances_[static_cast<size_t>(y) * width_ + x];
}

void DistanceField::squaredDistanceTransform1D(size_t count)
{
    const auto infinity = std::numeric_limits<double>::infinity();
    size_t first_site = 0;
    while (first_site < count && !std::isfinite(transform_input_[first_site]))
    {
        ++first_site;
    }
    if (first_site == count)
    {
        std::fill_n(transform_output_.begin(), count, infinity);
        return;
    }

    size_t envelope_size = 0;
    transform_sites_[0] = static_cast<int>(first_site);
    transform_breakpoints_[0] = -infinity;
    transform_breakpoints_[1] = infinity;
    for (size_t site = first_site + 1; site < count; ++site)
    {
        if (!std::isfinite(transform_input_[site]))
        {
            continue;
        }
        double breakpoint = 0.0;
        while (true)
        {
            const int previous_site = transform_sites_[envelope_size];
            const double site_coordinate = static_cast<double>(site);
            const double previous_coordinate = static_cast<double>(previous_site);
            breakpoint = ((transform_input_[site] + site_coordinate * site_coordinate) -
                          (transform_input_[previous_site] +
                           previous_coordinate * previous_coordinate)) /
                         (2.0 * (site_coordinate - previous_coordinate));
            if (breakpoint > transform_breakpoints_[envelope_size])
            {
                break;
            }
            --envelope_size;
        }
        ++envelope_size;
        transform_sites_[envelope_size] = static_cast<int>(site);
        transform_breakpoints_[envelope_size] = breakpoint;
        transform_breakpoints_[envelope_size + 1] = infinity;
    }

    size_t envelope_index = 0;
    for (size_t position = 0; position < count; ++position)
    {
        while (transform_breakpoints_[envelope_index + 1] < position)
        {
            ++envelope_index;
        }
        // `position` is unsigned while a parabola site is signed. Convert
        // before subtraction: cells left of a site need a negative offset,
        // not an unsigned wraparound distance.
        const double offset = static_cast<double>(position) -
                              static_cast<double>(transform_sites_[envelope_index]);
        transform_output_[position] =
                offset * offset + transform_input_[transform_sites_[envelope_index]];
    }
}

}  // namespace nimbus
