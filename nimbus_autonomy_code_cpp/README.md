# Nimbus autonomy

## Build

From this directory:

```bash
cmake --preset release
cmake --build --preset release
```

## Run

Use three terminals after building:

```bash
# RViz
bash launch_mapping_rviz.sh

# Game
nimbus_main_game

# Autonomy node
./build/release/nimbus_autonomy_node
```

To inspect mapping and planning while retaining keyboard control, inhibit
acceleration-command publication:

```bash
./build/release/nimbus_autonomy_node --ros-args -p inhibit_control:=true
```

## RViz

The supplied launch script opens RViz with the bird centred in view. It shows:

- `/nimbus/local_map`: raw local occupancy map;
- `/nimbus/inflated_map`: conservative collision layer used by planning;
- `/nimbus/distance_field`: full-map green-to-red distance field; red is at a
  raw obstacle and green is the greatest clearance currently visible in the map;
- `/nimbus/integrated_position`: integrated bird position;
- `/nimbus/planned_trajectory`: the current feasible trajectory.

## Objective experiments

The experiment harness explores the CEM-MPC objective-weight parameter space.
It uses the unmodified game; start each game with an arrow key when prompted.

Install the experiment dependency once:

```bash
python3 -m pip install -r tools/requirements.txt
```

Run an objective-weight sweep, a one-episode smoke test, or the saved candidate
confirmation run:

```bash
python3 tools/run_objective_experiments.py experiments/objective_sweep.json
python3 tools/run_objective_experiments.py experiments/objective_sweep.json --trials 1 --repeats 1
python3 tools/run_objective_experiments.py experiments/objective_confirmation.json
```

See [`tools/README.md`](tools/README.md) for the specification and
result formats.

## Project structure

```text
src/nimbus/             Core C++ autonomy, mapping, and planning code
src/nimbus/planning/    Motion model, CEM MPC, objective, and planner factory
tests/                   Unit and regression tests
benchmarks/              CEM and map-update benchmarks
docs/                    Algorithmic design overview
experiments/             Objective-weight experiment specifications and notes
tools/                   Experiment harness and its Python dependencies
nimbus_mapping.rviz     RViz configuration loaded by launch_mapping_rviz.sh
```

## Formatting

Install the pre-commit hook once from this directory:

```bash
pre-commit install
```

It applies the repository's `.clang-format` style to staged C++ source and
header files. To format every tracked C++ file manually, run:

```bash
pre-commit run --all-files
```
