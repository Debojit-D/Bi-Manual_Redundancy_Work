"""Configuration-driven orchestration of the authoritative paper runners."""

from __future__ import annotations

from contextlib import contextmanager
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
from typing import Callable

import numpy as np

from bimanual_redundancy import paths
from bimanual_redundancy.core import ManipulabilityObjective
from bimanual_redundancy.paper_config import (
    PAPER_CONFIG_NAMES,
    PaperConfig,
    load_named_paper_config,
    load_paper_config,
    smoke_overlay,
    toml_dumps,
)


OUTPUT_SUBDIRECTORIES = {
    "static": "static",
    "translational": "translational",
    "six_d": "six_d",
    "directional_direct_vs_indirect": "directional_comparison",
}


@dataclass(frozen=True)
class RunSelection:
    experiment: str
    objective: str
    initial_configuration: dict

    @property
    def identifier(self) -> str:
        return self.initial_configuration["id"]


def plan_runs(config: PaperConfig) -> tuple[RunSelection, ...]:
    """Expand the configured sweep without duplicating experiment logic."""
    return tuple(
        RunSelection(config.experiment, objective, initial)
        for initial in config.data["initial_configurations"]
        for objective in config.data["paper"]["objectives"]
    )


def _git_state() -> tuple[str, bool]:
    try:
        sha = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=paths.REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"],
                cwd=paths.REPO_ROOT,
                check=True,
                capture_output=True,
                text=True,
            ).stdout.strip()
        )
        return sha, dirty
    except (OSError, subprocess.CalledProcessError):
        return "unknown", True


def _package_version(distribution: str) -> str:
    try:
        return importlib.metadata.version(distribution)
    except importlib.metadata.PackageNotFoundError:
        return "not-installed"


def build_run_metadata(
    config: PaperConfig,
    selection: RunSelection,
    *,
    timestamp: str | None = None,
    status: str = "planned",
) -> dict:
    """Build the complete, JSON-safe provenance record for one run."""
    sha, dirty = _git_state()
    return {
        "timestamp": timestamp or datetime.now(timezone.utc).isoformat(),
        "git_commit_sha": sha,
        "git_working_tree_dirty": dirty,
        "paper_experiment": selection.experiment,
        "objective": selection.objective,
        "initial_configuration_identifier": selection.identifier,
        "python_version": platform.python_version(),
        "mujoco_version": _package_version("mujoco"),
        "mink_version": _package_version("mink"),
        "numpy_version": _package_version("numpy"),
        "scipy_version": _package_version("scipy"),
        "platform": platform.platform(),
        "operating_system": platform.system(),
        "status": status,
        "source_config": str(config.source),
        "resolved_config": deepcopy(config.data),
    }


def _write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _resolved_for_run(config: PaperConfig, selection: RunSelection) -> dict:
    resolved = deepcopy(config.data)
    resolved["run_selection"] = {
        "experiment": selection.experiment,
        "objective": selection.objective,
        "initial_configuration_identifier": selection.identifier,
    }
    return resolved


def _unique_batch_directory(root: Path, run_id: str | None = None) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    stem = run_id or datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S_%fZ")
    candidate = root / stem
    suffix = 2
    while candidate.exists():
        candidate = root / f"{stem}_{suffix}"
        suffix += 1
    candidate.mkdir(parents=False, exist_ok=False)
    return candidate


@contextmanager
def _configured_runner(module, data):
    """Inject runner constants for one call, restoring compatibility defaults."""
    controller = data["controller"]
    optimization = data["optimization"]
    bases = data["robot_bases"]
    trajectory = data.get("trajectory", {})
    replacements = {
        "CONTROL_HZ": float(controller["frequency_hz"]),
        "K_P": np.diag(controller["feedback_gains"]),
        "GRASP_K_P": np.diag(controller["grasp_feedback_gains"]),
        "ENABLE_ARM_BIAS_COMPENSATION": controller["arm_bias_compensation"],
        "LEFT_ARM_SPAWN_POSITION": np.asarray(bases["left_position"], dtype=float),
        "RIGHT_ARM_SPAWN_POSITION": np.asarray(bases["right_position"], dtype=float),
        "LEFT_ARM_SPAWN_EULER_XYZ_DEGREES": np.asarray(bases["left_euler_xyz_degrees"], dtype=float),
        "RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES": np.asarray(bases["right_euler_xyz_degrees"], dtype=float),
        "FINITE_DIFFERENCE_STEP": float(optimization["finite_difference_step"]),
        "DESIRED_WRENCH_DIRECTION": np.asarray(optimization["desired_wrench_direction"], dtype=float),
    }
    for name, key in (
        ("LIFT_HEIGHT", "lift_height"),
        ("LIFT_DURATION", "lift_duration"),
        ("LOWER_DURATION", "lower_duration"),
        ("FINAL_HOLD_DURATION", "final_hold_duration"),
        ("START_TO_INTERMEDIATE_DURATION", "start_to_intermediate_duration"),
        ("INTERMEDIATE_TO_GOAL_DURATION", "intermediate_to_goal_duration"),
    ):
        if key in trajectory and hasattr(module, name):
            replacements[name] = float(trajectory[key])
    originals = {name: getattr(module, name) for name in replacements if hasattr(module, name)}
    try:
        for name, value in replacements.items():
            if hasattr(module, name):
                setattr(module, name, value)
        yield
    finally:
        for name, value in originals.items():
            setattr(module, name, value)


def _common_runner_arguments(config: PaperConfig, run_dir: Path) -> dict:
    data = config.data
    collision = data["collision"]
    optimization = data["optimization"]
    limits = data["joint_limits"]
    recording = data["recording"]
    characteristic_length = (
        None
        if optimization["characteristic_length_mode"] == "automatic"
        else optimization["characteristic_length"]
    )
    sphere_model = Path(collision["sphere_model"])
    if not sphere_model.is_absolute():
        sphere_model = paths.REPO_ROOT / sphere_model
    return {
        "record_data": recording["data"],
        "output_csv": run_dir / "data.csv" if recording["data"] else None,
        "collision_weight": collision["inter_arm"]["weight"],
        "collision_safety_margin": collision["inter_arm"]["safety_margin"],
        "collision_proximity_scale": collision["inter_arm"]["proximity_scale"],
        "collision_sphere_model_path": sphere_model,
        "enable_collision_penalty": collision["inter_arm"]["enabled"],
        "enable_table_collision_penalty": collision["table"]["enabled"],
        "table_collision_weight": collision["table"]["weight"],
        "table_collision_safety_margin": collision["table"]["safety_margin"],
        "table_collision_proximity_scale": collision["table"]["proximity_scale"],
        "table_collision_geom_name": collision["table"].get("geom_name") or None,
        "enable_self_collision_penalty": collision["self"]["enabled"],
        "self_collision_weight": collision["self"]["weight"],
        "self_collision_safety_margin": collision["self"]["safety_margin"],
        "self_collision_proximity_scale": collision["self"]["proximity_scale"],
        "optimization_gain": optimization["gain"],
        "maximum_joint_speed": optimization["maximum_joint_velocity"],
        "joint_limit_margin": limits["margin"],
        "joint_limit_stop_distance": limits["stop_distance"],
        "joint_limit_slow_distance": limits["slow_distance"],
        "characteristic_length": characteristic_length,
        "video_output_dir": run_dir / "video" if recording["video"] else None,
        "video_width": recording["video_width"],
        "video_height": recording["video_height"],
        "video_fps": recording["video_fps"],
        "video_views": tuple(recording["video_views"]),
        "video_encoder": recording["video_encoder"],
        "video_nvenc_view_limit": len(recording["video_views"]),
        "headless": bool(data.get("smoke", False)),
    }


def execute_run(selection: RunSelection, run_dir: Path, config: PaperConfig) -> None:
    """Call the existing authoritative runner with only resolved config values."""
    data = config.data
    common = _common_runner_arguments(config, run_dir)
    baseline = selection.objective == "baseline"
    objective = (
        ManipulabilityObjective.FORCE
        if baseline
        else ManipulabilityObjective(selection.objective)
    )
    common.update(
        objective=objective,
        enable_redundancy_optimization=not baseline,
    )

    if selection.experiment in ("static", "directional_direct_vs_indirect"):
        from bimanual_redundancy.experiments import dual_franka_eq8_static_optimization as runner

        stopping = data["stopping"]
        with _configured_runner(runner, data):
            runner.main(
                **common,
                table_spawn_position=selection.initial_configuration["position"],
                duration=stopping.get("fixed_duration_seconds"),
                convergence_speed_threshold=(
                    None if "fixed_duration_seconds" in stopping else stopping["convergence_speed"]
                ),
                convergence_hold_duration=stopping["convergence_hold_seconds"],
                minimum_convergence_time=stopping["minimum_run_seconds"],
            )
        return

    if selection.experiment == "translational":
        from bimanual_redundancy.experiments import dual_franka_eq8_optimized_pick_place as runner

        with _configured_runner(runner, data):
            runner.main(
                **common,
                table_spawn_position=selection.initial_configuration["position"],
                hold_duration=data["trajectory"]["final_hold_duration"],
            )
        return

    from bimanual_redundancy.experiments import dual_franka_eq8_optimized_6d_pick_place as runner

    pose = selection.initial_configuration
    trajectory = data["trajectory"]
    with _configured_runner(runner, data):
        runner.main(
            **common,
            start_position=pose["position"],
            start_euler_xyz=pose["euler_xyz"],
            intermediate_position=pose["intermediate_position"],
            intermediate_euler_xyz=pose["intermediate_euler_xyz"],
            goal_position=pose["goal_position"],
            goal_euler_xyz=pose["goal_euler_xyz"],
            start_to_intermediate_duration=trajectory["start_to_intermediate_duration"],
            intermediate_to_goal_duration=trajectory["intermediate_to_goal_duration"],
            hold_duration=trajectory["final_hold_duration"],
        )


RunExecutor = Callable[[RunSelection, Path, PaperConfig], None]


def run_config(
    config: PaperConfig | str | Path,
    *,
    output_root: str | Path | None = None,
    smoke: bool = False,
    batch_dir: Path | None = None,
    executor: RunExecutor = execute_run,
) -> Path:
    """Execute one configured campaign and return its collision-safe batch path."""
    if not isinstance(config, PaperConfig):
        config = load_paper_config(config)
    if smoke:
        config = smoke_overlay(config)
    root = Path(output_root or paths.OUTPUTS_DIR / "paper_reproduction").expanduser().resolve()
    batch_dir = batch_dir or _unique_batch_directory(root)
    metadata_dir = batch_dir / "metadata"
    metadata_dir.mkdir(parents=True, exist_ok=True)
    selections = plan_runs(config)
    campaign = {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "paper_experiment": config.experiment,
        "smoke": smoke,
        "publication_statistics_reproduced": False if smoke else None,
        "planned_runs": len(selections),
        "source_config": str(config.source),
        "resolved_config": config.data,
    }
    _write_json(metadata_dir / f"{config.experiment}_run_metadata.json", campaign)
    (metadata_dir / f"{config.experiment}_resolved_config.toml").write_text(
        toml_dumps(config.data), encoding="utf-8"
    )

    experiment_dir = batch_dir / OUTPUT_SUBDIRECTORIES[config.experiment]
    for selection in selections:
        run_dir = experiment_dir / selection.identifier / selection.objective
        run_dir.mkdir(parents=True, exist_ok=False)
        metadata = build_run_metadata(config, selection)
        _write_json(run_dir / "run_metadata.json", metadata)
        (run_dir / "resolved_config.toml").write_text(
            toml_dumps(_resolved_for_run(config, selection)), encoding="utf-8"
        )
        try:
            executor(selection, run_dir, config)
        except BaseException as error:
            metadata["status"] = "failed"
            metadata["error"] = f"{type(error).__name__}: {error}"
            _write_json(run_dir / "run_metadata.json", metadata)
            raise
        metadata["status"] = "completed"
        _write_json(run_dir / "run_metadata.json", metadata)
    return batch_dir


def reproduce_paper(
    *,
    output_root: str | Path | None = None,
    smoke: bool = False,
    executor: RunExecutor = execute_run,
) -> Path:
    """Run all paper campaigns, or one representative smoke run."""
    root = Path(output_root or paths.OUTPUTS_DIR / "paper_reproduction").expanduser().resolve()
    batch_dir = _unique_batch_directory(root)
    names = ("static",) if smoke else PAPER_CONFIG_NAMES
    for name in names:
        run_config(
            load_named_paper_config(name),
            smoke=smoke,
            batch_dir=batch_dir,
            executor=executor,
        )
    return batch_dir
