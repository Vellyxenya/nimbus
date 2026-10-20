# Objective-weight experiments

This directory contains JSON specifications for tuning the CEM-MPC objective
without recompiling the autonomy node. The harness launches a fresh game and
node for every episode, applies one generated ROS 2 parameter file, and scores
the episode by the furthest integrated forward position.

The shipped game uses manual start. When the harness reports that a game is
ready, focus its window and press an arrow key to start that episode. This keeps
the experiment workflow independent of game-specific test hooks.

Build the C++ node and install the Python dependency once:

```bash
cd nimbus_autonomy_code_cpp
cmake --preset release
cmake --build --preset release
python3 -m pip install -r tools/requirements.txt
```

Close manually launched instances of the game and autonomy node, then run a
TPE search:

```bash
python3 tools/run_objective_experiments.py experiments/objective_sweep.json
```

For a one-episode smoke test, use:

```bash
python3 tools/run_objective_experiments.py experiments/objective_sweep.json --trials 1 --repeats 1
```

## Sweep specifications

[`objective_sweep.json`](objective_sweep.json) uses Optuna's TPE sampler.
`fixed_parameters` holds values that do not vary; each entry in `ranges` names
one ROS parameter and defines `lower`, `upper`, and either `linear` or `log`
sampling. Parameters absent from both sections retain the baseline value in
the harness.

`trial_count` is the number of configurations, not a list of evenly spaced
values. The first `tpe_startup_trials` configurations are random; TPE selects
later configurations from the results observed so far. `runs_per_configuration`
repeats a configuration and scores it by mean furthest x. Use
`episode_timeout_seconds` as the maximum duration of one run.

## Confirmation specifications

[`objective_confirmation.json`](objective_confirmation.json) compares saved
full configurations rather than sampling new values. Each candidate needs a
label and all objective weights. Run it with:

```bash
python3 tools/run_objective_experiments.py experiments/objective_confirmation.json
```

Confirmation ranks candidates first by completed time-limit runs, then by mean
furthest x position. This makes the final selection favour repeatable success.

## Results

Every invocation creates a timestamped directory under `experiment_results/`:

- `specification.json`: exact input specification used for the run;
- `trial_*/parameters.yaml`: ROS parameters applied to that configuration;
- `trial_*/run_*/node.log` and `game.log`: diagnostics for each episode;
- `results.csv`: one row per episode, including outcome and furthest x;
- `trials.csv`: mean score and minimum distance for each configuration;
- `best_parameters.json`: best configuration according to the run mode's
  ranking.

Use `--output-dir <path>` to choose a location, `--trials` and `--repeats` to
override a sweep specification, and `--dry-run` to validate configuration
generation without launching ROS or the game.
