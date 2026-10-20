#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <vector>

#include "nimbus/occupancy_grid.hpp"

namespace
{

constexpr size_t kWarmupRuns = 10;
constexpr size_t kMeasuredRuns = 100;

double milliseconds(std::chrono::nanoseconds duration)
{
    return std::chrono::duration<double, std::milli>(duration).count();
}

double percentile(std::vector<double> values, double fraction)
{
    std::sort(values.begin(), values.end());
    const auto index = static_cast<size_t>(std::ceil(fraction * values.size())) - 1;
    return values[index];
}

struct ScanData
{
    std::vector<double> angles;
    std::vector<float> ranges;
    std::vector<bool> hits;
};

ScanData representativeScan()
{
    return {{-0.7853981634, -0.5890486225, -0.3926990817, -0.1963495408, 0.0,
             0.1963495408, 0.3926990817, 0.5890486225, 0.7853981634},
            {2.6F, 2.3F, 2.1F, 1.9F, 1.8F, 1.9F, 2.1F, 2.3F, 2.6F},
            {true, true, true, true, true, true, true, true, true}};
}

double mean(const std::vector<double>& values)
{
    return std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
}

}  // namespace

int main()
{
    const auto scan = representativeScan();
    nimbus::RollingOccupancyGrid map;
    for (size_t run = 0; run < kWarmupRuns; ++run)
    {
        map.updateScan(0.035 * static_cast<double>(run), 0.05 * std::sin(0.1 * run),
                       scan.angles, scan.ranges, scan.hits);
    }

    std::vector<double> total_ms;
    total_ms.reserve(kMeasuredRuns);
    for (size_t run = 0; run < kMeasuredRuns; ++run)
    {
        const auto started = std::chrono::steady_clock::now();
        map.updateScan(0.035 * static_cast<double>(run + kWarmupRuns),
                       0.05 * std::sin(0.1 * (run + kWarmupRuns)), scan.angles,
                       scan.ranges, scan.hits);
        total_ms.push_back(milliseconds(std::chrono::steady_clock::now() - started));
    }

    nimbus::RollingOccupancyGrid instrumented_map;
    std::vector<double> recentering_ms;
    std::vector<double> ray_integration_ms;
    std::vector<double> extraction_ms;
    std::vector<double> distance_field_ms;
    std::vector<double> inflation_ms;
    for (size_t run = 0; run < kWarmupRuns; ++run)
    {
        instrumented_map.updateScanWithTiming(0.035 * static_cast<double>(run),
                                              0.05 * std::sin(0.1 * run), scan.angles,
                                              scan.ranges, scan.hits);
    }
    for (size_t run = 0; run < kMeasuredRuns; ++run)
    {
        const double x = 0.035 * static_cast<double>(run + kWarmupRuns);
        const double y = 0.05 * std::sin(0.1 * (run + kWarmupRuns));
        instrumented_map.updateScanWithTiming(x, y, scan.angles, scan.ranges, scan.hits);
        const auto& timing = instrumented_map.lastUpdateTiming();
        recentering_ms.push_back(milliseconds(timing.recentering));
        ray_integration_ms.push_back(milliseconds(timing.ray_integration));
        extraction_ms.push_back(milliseconds(timing.occupied_extraction));
        distance_field_ms.push_back(milliseconds(timing.distance_field));
        inflation_ms.push_back(milliseconds(timing.inflation));
    }

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "scenario=representative_scan\n";
    std::cout << "runs=" << kMeasuredRuns << " warmup_runs=" << kWarmupRuns << "\n";
    std::cout << "update_ms p50=" << percentile(total_ms, 0.50)
              << " p95=" << percentile(total_ms, 0.95)
              << " p99=" << percentile(total_ms, 0.99) << "\n";
    std::cout << "phase_mean_ms recenter=" << mean(recentering_ms)
              << " ray_integration=" << mean(ray_integration_ms)
              << " occupied_extraction=" << mean(extraction_ms)
              << " distance_field=" << mean(distance_field_ms)
              << " inflation=" << mean(inflation_ms) << "\n";
    return 0;
}
