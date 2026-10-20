#!/usr/bin/env python3
"""Search or confirm CEM objective weights over repeatable game episodes.

Each episode launches a fresh game and autonomy node, records the furthest
integrated x position, and lets Optuna maximise mean distance across repeats.
The game is started manually with an arrow key after its window appears.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import re
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any

try:
    import optuna
except ModuleNotFoundError as exception:
    raise SystemExit(
        "Optuna is required. Install it with "
        "'python3 -m pip install -r tools/requirements.txt'."
    ) from exception


PROJECT_DIR = Path(__file__).resolve().parents[1]
NODE_NAME = "nimbus_autonomy_code_cpp"
GAME_NODE_NAME = "main_game"
GAME_ENDED_TOPIC = "/nimbus_game_ended"
POSITION_PATTERN = re.compile(r"pose=\(([-+0-9.]+), ([-+0-9.]+)\)")

# Start every trial from the checked-in configuration. A sweep specification
# changes only its fixed or ranged parameters; the remaining weights stay here.
DEFAULT_WEIGHTS = {
    "objective.progress": 176.83785006361197,
    "objective.clearance": 0.08092761655183967,
    "objective.vertical_velocity": 0.1,
    "objective.control_effort": 0.09092936002368128,
    "objective.control_rate": 3.8630302359403776,
    "objective.collision": 1.0e6,
    "objective.early_collision": 500.0,
    "objective.minimum_clearance": 0.05,
}


class LoggedProcess:
    """A managed subprocess whose output supplies readiness and distance metrics.

    Output is both saved verbatim for post-run diagnosis and read live. The
    autonomy node's readiness line gates game launch, while its mapping lines
    provide the furthest integrated x position used as the episode score.
    """

    def __init__(self, command: list[str], log_path: Path) -> None:
        self._log_file = log_path.open("w", encoding="utf-8")
        self.process = subprocess.Popen(
            command,
            cwd=PROJECT_DIR,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.ready = threading.Event()
        self.max_position_x = 0.0
        self._reader = threading.Thread(target=self._read_output, daemon=True)
        self._reader.start()

    def _read_output(self) -> None:
        """Persist output while extracting the node-ready signal and max x metric."""
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self._log_file.write(line)
            self._log_file.flush()
            if "Nimbus autonomy node ready." in line:
                self.ready.set()
            position = POSITION_PATTERN.search(line)
            if position is not None:
                self.max_position_x = max(self.max_position_x, float(position.group(1)))

    def stop(self) -> None:
        """Terminate the complete process group and close its diagnostic log."""
        if self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self._reader.join(timeout=1.0)
        self._log_file.close()


def parse_arguments() -> argparse.Namespace:
    """Parse the sweep/confirmation specification and optional runtime overrides."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path, help="JSON sweep specification")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for generated parameter files, logs, and results.csv",
    )
    parser.add_argument(
        "--node-command",
        default=str(PROJECT_DIR / "build/release/nimbus_autonomy_node"),
        help="Autonomy-node executable",
    )
    parser.add_argument(
        "--game-command",
        default="nimbus_main_game",
        help="Command that launches the unmodified game",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Generate configurations and results.csv headers without launching trials",
    )
    parser.add_argument("--trials", type=int, default=None,
                        help="Override the specification's trial_count")
    parser.add_argument("--repeats", type=int, default=None,
                        help="Override the specification's runs_per_configuration")
    return parser.parse_args()


def read_spec(path: Path) -> dict[str, Any]:
    """Read one JSON experiment specification and require an object at its root."""
    with path.open(encoding="utf-8") as stream:
        spec = json.load(stream)
    if not isinstance(spec, dict):
        raise ValueError("the experiment specification must be a JSON object")
    return spec


def validate_weight_name(name: str) -> None:
    """Reject parameter names that are not exposed by the autonomy node."""
    if name not in DEFAULT_WEIGHTS:
        raise ValueError(f"unknown objective parameter '{name}'")


def suggest_configuration(spec: dict[str, Any], trial: Any) -> dict[str, float]:
    """Merge defaults and fixed weights, then ask Optuna for every swept weight."""
    configuration = dict(DEFAULT_WEIGHTS)
    fixed_parameters = spec.get("fixed_parameters", {})
    if not isinstance(fixed_parameters, dict):
        raise ValueError("'fixed_parameters' must be an object")
    for name, value in fixed_parameters.items():
        validate_weight_name(name)
        configuration[name] = float(value)

    ranges = spec.get("ranges", {})
    if not isinstance(ranges, dict):
        raise ValueError("'ranges' must be an object")
    for name, range_specification in ranges.items():
        validate_weight_name(name)
        if name in fixed_parameters:
            raise ValueError(f"'{name}' cannot be both fixed and swept")
        if not isinstance(range_specification, dict):
            raise ValueError(f"range for '{name}' must be an object")
        lower = float(range_specification["lower"])
        upper = float(range_specification["upper"])
        scale = range_specification.get("scale", "linear")
        if not math.isfinite(lower) or not math.isfinite(upper) or lower > upper:
            raise ValueError(f"range for '{name}' must be finite with lower <= upper")
        if scale == "linear":
            configuration[name] = trial.suggest_float(name, lower, upper)
        elif scale == "log" and lower > 0.0:
            configuration[name] = trial.suggest_float(name, lower, upper, log=True)
        else:
            raise ValueError(f"range for '{name}' has invalid scale '{scale}'")
    return configuration


def confirmation_configurations(spec: dict[str, Any]) -> list[tuple[str, dict[str, float]]] | None:
    """Return saved full-weight candidates, or ``None`` for an Optuna search."""
    candidates = spec.get("candidate_configurations")
    if candidates is None:
        return None
    if not isinstance(candidates, list) or not candidates:
        raise ValueError("'candidate_configurations' must be a non-empty list")

    configurations: list[tuple[str, dict[str, float]]] = []
    for candidate in candidates:
        if not isinstance(candidate, dict):
            raise ValueError("every candidate configuration must be an object")
        label = candidate.get("label")
        weights = candidate.get("weights")
        if not isinstance(label, str) or not label:
            raise ValueError("every candidate configuration needs a non-empty 'label'")
        if not isinstance(weights, dict) or set(weights) != set(DEFAULT_WEIGHTS):
            raise ValueError(f"candidate '{label}' must define every objective weight")
        configurations.append((label, {name: float(value) for name, value in weights.items()}))
    return configurations


def write_ros_parameter_file(path: Path, weights: dict[str, float]) -> None:
    """Materialise one configuration as a ROS 2 parameter file for a fresh node."""
    lines = [f"{NODE_NAME}:", "  ros__parameters:"]
    for name, value in weights.items():
        rendered = f"{value:.17g}"
        if "." not in rendered and "e" not in rendered.lower():
            rendered += ".0"
        lines.append(f"    {name}: {rendered}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def ensure_no_active_session() -> None:
    """Avoid mixing trial traffic with a manually launched game or autonomy node."""
    node_names = subprocess.run(
        ["ros2", "node", "list"],
        check=True,
        capture_output=True,
        text=True,
    ).stdout.splitlines()
    if f"/{NODE_NAME}" in node_names:
        raise RuntimeError("stop the existing autonomy node before starting an experiment")
    if f"/{GAME_NODE_NAME}" in node_names:
        raise RuntimeError("close the existing Nimbus game before starting an experiment")


def start_game_end_listener() -> subprocess.Popen[str]:
    """Create the one-shot subscriber that classifies each episode's outcome."""
    return subprocess.Popen(
        ["ros2", "topic", "echo", "--once", GAME_ENDED_TOPIC],
        cwd=PROJECT_DIR,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def wait_for_game_end(listener: subprocess.Popen[str], timeout_seconds: float) -> tuple[str, str]:
    """Wait for crash/time-limit status, treating a silent game as a runner timeout."""
    try:
        output, _ = listener.communicate(timeout=timeout_seconds)
    except subprocess.TimeoutExpired:
        listener.terminate()
        output, _ = listener.communicate(timeout=3.0)
        return "runner_timeout", output
    if listener.returncode != 0:
        return "listener_error", output
    if re.search(r"data:\s*true", output):
        return "crash", output
    if re.search(r"data:\s*false", output):
        return "game_time_limit", output
    return "listener_error", output


def run_episode(
    trial_directory: Path,
    parameter_file: Path,
    node_command: str,
    game_command: str,
    timeout_seconds: float,
) -> dict[str, Any]:
    """Run one isolated game episode and return the score plus its terminal outcome.

    The node starts first so it cannot miss the first scan. The game-end
    listener subscribes before manual start, ensuring quick crashes are still
    recorded. ``finally`` always tears down all spawned processes.
    """
    trial_directory.mkdir()
    node = LoggedProcess(
        shlex.split(node_command) + ["--ros-args", "--params-file", str(parameter_file)],
        trial_directory / "node.log",
    )
    game: LoggedProcess | None = None
    listener: subprocess.Popen[str] | None = None
    started = time.monotonic()
    try:
        if not node.ready.wait(timeout=10.0):
            raise RuntimeError("autonomy node did not report readiness within 10 seconds")
        game = LoggedProcess(shlex.split(game_command), trial_directory / "game.log")
        listener = start_game_end_listener()
        time.sleep(0.5)  # Let the game-end subscriber join before manual start.
        print("Game ready. Press an arrow key in the game window to start the episode.",
              flush=True)
        outcome, listener_output = wait_for_game_end(listener, timeout_seconds)
        (trial_directory / "game_end_listener.log").write_text(listener_output,
                                                                  encoding="utf-8")
        return {
            "outcome": outcome,
            "elapsed_seconds": time.monotonic() - started,
            "max_position_x": node.max_position_x,
        }
    finally:
        if listener is not None and listener.poll() is None:
            listener.terminate()
            listener.wait(timeout=3.0)
        if game is not None:
            game.stop()
        node.stop()


def write_results_header(path: Path) -> None:
    """Create the per-episode results table."""
    with path.open("w", newline="", encoding="utf-8") as stream:
        csv.DictWriter(
                stream,
                fieldnames=[
                        "trial",
                        "candidate",
                        "repeat",
                        "outcome",
                        "elapsed_seconds",
                        "max_position_x",
                        "weights_json",
                ],
        ).writeheader()


def write_trial_results_header(path: Path) -> None:
    """Create the per-configuration aggregate-score table."""
    with path.open("w", newline="", encoding="utf-8") as stream:
        csv.DictWriter(
                stream,
                fieldnames=[
                        "trial", "candidate", "completed_runs", "objective_score",
                        "mean_max_position_x",
                        "minimum_max_position_x", "weights_json",
                ],
        ).writeheader()


def verify_command(command: str, description: str) -> None:
    """Fail early when a requested node or game command cannot be launched."""
    arguments = shlex.split(command)
    if not arguments:
        raise RuntimeError(f"{description} command is empty")
    executable = Path(arguments[0])
    if shutil.which(arguments[0]) is None and not executable.is_file():
        raise RuntimeError(f"{description} executable '{arguments[0]}' is not on PATH")


def main() -> int:
    """Execute either a TPE sweep or a fixed-candidate confirmation run."""
    arguments = parse_arguments()
    if not arguments.dry_run:
        if shutil.which("ros2") is None:
            raise RuntimeError("the harness requires 'ros2' on PATH")
        verify_command(arguments.node_command, "autonomy node")
        verify_command(arguments.game_command, "game")
        ensure_no_active_session()
    specification = read_spec(arguments.spec)
    candidates = confirmation_configurations(specification)
    if candidates is not None and arguments.trials is not None:
        raise ValueError("--trials is not supported for fixed candidate configurations")
    trial_count = (len(candidates) if candidates is not None else
                   (arguments.trials if arguments.trials is not None
                    else int(specification.get("trial_count", 1))))
    repeats = (arguments.repeats if arguments.repeats is not None
               else int(specification.get("runs_per_configuration", 1)))
    timeout_seconds = float(specification.get("episode_timeout_seconds", 75.0))
    if trial_count <= 0 or repeats <= 0 or timeout_seconds <= 0.0:
        raise ValueError("trial count, repeats, and episode timeout must be positive")

    output_directory = arguments.output_dir or (
            PROJECT_DIR / "experiment_results" /
            time.strftime("%Y%m%d_%H%M%S"))
    output_directory.mkdir(parents=True, exist_ok=False)
    (output_directory / "specification.json").write_text(
            json.dumps(specification, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    results_path = output_directory / "results.csv"
    write_results_header(results_path)
    trial_results_path = output_directory / "trials.csv"
    write_trial_results_header(trial_results_path)

    # A candidate list is a deterministic confirmation run. Otherwise Optuna's
    # TPE sampler uses early random trials before exploiting observed scores.
    study = None
    if candidates is None:
        startup_trials = int(specification.get("tpe_startup_trials", 10))
        if startup_trials < 0:
            raise ValueError("'tpe_startup_trials' must be non-negative")
        optuna.logging.set_verbosity(optuna.logging.WARNING)
        study = optuna.create_study(
                direction="maximize",
                sampler=optuna.samplers.TPESampler(
                        seed=int(specification.get("seed", 42)),
                        n_startup_trials=startup_trials,
                ),
        )

    best_result: dict[str, Any] | None = None
    for trial_index in range(trial_count):
        optuna_trial = None
        if candidates is not None:
            candidate_label, weights = candidates[trial_index]
            trial_number = trial_index
        else:
            assert study is not None
            optuna_trial = study.ask()
            weights = suggest_configuration(specification, optuna_trial)
            optuna_trial.set_user_attr("weights", weights)
            candidate_label = "tpe"
            trial_number = optuna_trial.number
        trial_directory = output_directory / f"trial_{trial_number:03d}"
        trial_directory.mkdir()
        parameter_file = trial_directory / "parameters.yaml"
        write_ros_parameter_file(parameter_file, weights)

        # Repeat an identical parameter file to reduce sensitivity to different
        # game layouts; their mean furthest distance is the search objective.
        episode_results: list[dict[str, Any]] = []
        for repeat in range(repeats):
            if arguments.dry_run:
                result = {
                    "outcome": "dry_run",
                    "elapsed_seconds": 0.0,
                    "max_position_x": 0.0,
                }
            else:
                result = run_episode(
                        trial_directory / f"run_{repeat:02d}",
                        parameter_file,
                        arguments.node_command,
                        arguments.game_command,
                        timeout_seconds,
                )
            episode_results.append(result)
            with results_path.open("a", newline="", encoding="utf-8") as stream:
                csv.DictWriter(stream, fieldnames=[
                        "trial", "candidate", "repeat", "outcome", "elapsed_seconds",
                        "max_position_x", "weights_json",
                ]).writerow({
                        "trial": trial_number,
                        "candidate": candidate_label,
                        "repeat": repeat,
                        **result,
                        "weights_json": json.dumps(weights, sort_keys=True),
                })
            print(f"trial={trial_number} repeat={repeat} outcome={result['outcome']} "
                  f"max_x={result['max_position_x']:.2f} m")
        objective_score = sum(result["max_position_x"] for result in episode_results) / repeats
        completed_runs = sum(result["outcome"] == "game_time_limit"
                             for result in episode_results)
        if optuna_trial is not None:
            assert study is not None
            study.tell(optuna_trial, objective_score)
        with trial_results_path.open("a", newline="", encoding="utf-8") as stream:
            csv.DictWriter(stream, fieldnames=[
                    "trial", "candidate", "completed_runs", "objective_score",
                    "mean_max_position_x",
                    "minimum_max_position_x", "weights_json",
            ]).writerow({
                    "trial": trial_number,
                    "candidate": candidate_label,
                    "completed_runs": completed_runs,
                    "objective_score": objective_score,
                    "mean_max_position_x": objective_score,
                    "minimum_max_position_x": min(
                            result["max_position_x"] for result in episode_results),
                    "weights_json": json.dumps(weights, sort_keys=True),
            })
        print(f"trial={trial_number} objective_score={objective_score:.2f} m")

        # Confirmation favours configurations that finish more complete runs;
        # an exploratory sweep optimises its mean-distance objective directly.
        rank = ((completed_runs, objective_score) if candidates is not None
                else (objective_score,))
        if best_result is None or rank > best_result["rank"]:
            best_result = {
                    "rank": rank,
                    "trial": trial_number,
                    "candidate": candidate_label,
                    "completed_runs": completed_runs,
                    "objective_score": objective_score,
                    "weights": weights,
            }
    assert best_result is not None
    (output_directory / "best_parameters.json").write_text(
            json.dumps({
                    key: value for key, value in best_result.items() if key != "rank"
            }, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
    )
    print(f"Results: {results_path}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, ValueError, subprocess.CalledProcessError) as exception:
        print(f"error: {exception}", file=sys.stderr)
        raise SystemExit(1) from exception
