# Shipping benchmark snapshot

This file intentionally records only the current configuration to ship. Older
implementation comparisons and rejected experiments were removed because their
parameters and map representation no longer match this code.

## Reproduce

Run from `nimbus_autonomy_code_cpp` after closing the game and autonomy node:

```bash
cmake --preset release
cmake --build --preset release
./build/release/cem_mpc_benchmark
./build/release/mapping_benchmark
```

Both benchmarks use 10 warm-up runs followed by 100 measured runs. The CEM
benchmark retains its normal warm-start state between calls and uses a
deterministic populated local map. The mapping benchmark integrates a
deterministic nine-ray hit scan while rolling the window one cell per update.

Compare results only on the same machine and Release preset. The instrumented
phase timings add measurement overhead, so use the uninstrumented percentiles
for performance comparisons.

## Configuration

| Component | Shipping value |
| --- | --- |
| Build | CMake `release` preset: GCC 13.3, `-O3 -DNDEBUG`, C++17, `-Wall -Wextra` |
| MPC budget | 20 ms |
| CEM | horizon 40; 400 samples × 3 iterations; 50 elites; seed 42 |
| Map | 3.5 cm cells; 4.5 m width; 0.5 m rear extent; fixed game vertical extent; direct unknown/free/occupied cells |
| Obstacle safety radius | 0.115 m, plus one conservative grid cell during inflation |
| Objective | progress 175; clearance 0.08; vertical velocity 0.10; control effort 0.09; control rate 4.0; collision `1e6`; early collision 500; minimum clearance 0.05 |

## Results

Recorded on an 11th Gen Intel Core i7-11700 with GCC 13.3.
These values are a final release snapshot, not a cross-machine 
performance guarantee.

### CEM MPC: `representative_clutter`

| Latency p50 | Latency p95 | Latency p99 | Mean rollouts | Budget timeouts |
| ---: | ---: | ---: | ---: | ---: |
| 4.11 ms | 4.40 ms | 4.53 ms | 1200 | 0 / 100 |

Instrumented mean phase timings: sampling 1.64 ms; state propagation 1.36 ms;
collision checking 1.83 ms; objective evaluation 0.64 ms; selection/refit
0.09 ms.

### Map update: `representative_scan`

| Update p50 | Update p95 | Update p99 |
| ---: | ---: | ---: |
| 0.530 ms | 0.606 ms | 0.676 ms |

Instrumented mean phase timings: recentering 0.000 ms; ray integration
0.002 ms; occupied extraction 0.022 ms; distance field 0.095 ms; inflation
0.445 ms.
