"""Load, validate, and serialize paper-reproduction TOML configurations."""

from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
import math
import tomllib

from bimanual_redundancy import paths


SCHEMA_VERSION = 1
PAPER_CONFIG_NAMES = (
    "static",
    "translational",
    "six_d",
    "directional_direct_vs_indirect",
)
EXPECTED_OBJECTIVES = {
    "static": (
        "baseline",
        "velocity",
        "force",
        "directional_force",
        "directional_force_indirect",
    ),
    "translational": (
        "baseline",
        "velocity",
        "force",
        "directional_force",
    ),
    "six_d": (
        "baseline",
        "velocity",
        "force",
        "directional_force",
        "directional_force_indirect",
    ),
    "directional_direct_vs_indirect": (
        "directional_force",
        "directional_force_indirect",
    ),
}


class ConfigError(ValueError):
    """Raised when a reproduction configuration is incomplete or unsafe."""


@dataclass(frozen=True)
class PaperConfig:
    """A validated paper configuration and the file from which it came."""

    source: Path
    data: dict

    @property
    def experiment(self) -> str:
        return self.data["paper"]["experiment"]


def paper_config_path(name: str) -> Path:
    if name not in PAPER_CONFIG_NAMES:
        raise ConfigError(f"unknown paper experiment {name!r}")
    return paths.CONFIGS_DIR / "paper" / f"{name}.toml"


def load_paper_config(source: str | Path) -> PaperConfig:
    source = Path(source).expanduser().resolve()
    try:
        with source.open("rb") as stream:
            data = tomllib.load(stream)
    except (OSError, tomllib.TOMLDecodeError) as error:
        raise ConfigError(f"cannot load {source}: {error}") from error
    validate_paper_config(data, source=source)
    return PaperConfig(source=source, data=data)


def load_named_paper_config(name: str) -> PaperConfig:
    return load_paper_config(paper_config_path(name))


def _require(mapping, key, expected_type, context):
    if key not in mapping:
        raise ConfigError(f"{context}.{key} is required")
    value = mapping[key]
    if not isinstance(value, expected_type):
        raise ConfigError(f"{context}.{key} has the wrong type")
    return value


def _positive(value, name, *, allow_zero=False):
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise ConfigError(f"{name} must be numeric")
    if not math.isfinite(value) or (value < 0 if allow_zero else value <= 0):
        qualifier = "nonnegative" if allow_zero else "greater than zero"
        raise ConfigError(f"{name} must be finite and {qualifier}")


def _vector(value, size, name):
    if not isinstance(value, list) or len(value) != size:
        raise ConfigError(f"{name} must contain exactly {size} numbers")
    for item in value:
        if not isinstance(item, (int, float)) or not math.isfinite(item):
            raise ConfigError(f"{name} must contain only finite numbers")


def validate_paper_config(data: dict, *, source: Path | None = None) -> None:
    """Reject configs that could silently change the paper campaign shape."""
    origin = str(source) if source else "config"
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ConfigError(f"{origin}: schema_version must be {SCHEMA_VERSION}")
    paper = _require(data, "paper", dict, origin)
    experiment = _require(paper, "experiment", str, "paper")
    if experiment not in EXPECTED_OBJECTIVES:
        raise ConfigError(f"paper.experiment must be one of {tuple(EXPECTED_OBJECTIVES)}")
    objectives = tuple(_require(paper, "objectives", list, "paper"))
    expected = EXPECTED_OBJECTIVES[experiment]
    if objectives != expected:
        raise ConfigError(
            f"{experiment} objectives must be {expected}, got {objectives}"
        )
    if paper.get("robot") != "dual_franka_panda":
        raise ConfigError("paper.robot must be 'dual_franka_panda'")

    initial = _require(data, "initial_configurations", list, origin)
    if len(initial) != 6:
        raise ConfigError("paper experiments require exactly six initial configurations")
    prefix = "pose" if experiment == "six_d" else "position"
    expected_ids = {f"{prefix}_{index}" for index in range(1, 7)}
    identifiers = {_require(item, "id", str, "initial_configurations[]") for item in initial}
    if identifiers != expected_ids:
        raise ConfigError(f"initial configuration identifiers must be {sorted(expected_ids)}")
    for item in initial:
        _vector(item.get("position"), 3, f"{item['id']}.position")
        if experiment == "six_d":
            for key in (
                "euler_xyz",
                "intermediate_position",
                "intermediate_euler_xyz",
                "goal_position",
                "goal_euler_xyz",
            ):
                _vector(item.get(key), 3, f"{item['id']}.{key}")

    controller = _require(data, "controller", dict, origin)
    _positive(controller.get("frequency_hz"), "controller.frequency_hz")
    _vector(controller.get("feedback_gains"), 6, "controller.feedback_gains")
    _vector(controller.get("grasp_feedback_gains"), 6, "controller.grasp_feedback_gains")
    optimization = _require(data, "optimization", dict, origin)
    for key in ("gain", "finite_difference_step", "maximum_joint_velocity"):
        _positive(optimization.get(key), f"optimization.{key}")
    _vector(optimization.get("desired_wrench_direction"), 6, "optimization.desired_wrench_direction")
    if optimization.get("characteristic_length_mode") not in ("automatic", "manual"):
        raise ConfigError("optimization.characteristic_length_mode must be automatic or manual")
    if optimization["characteristic_length_mode"] == "manual":
        _positive(optimization.get("characteristic_length"), "optimization.characteristic_length")

    limits = _require(data, "joint_limits", dict, origin)
    for key in ("margin", "stop_distance", "slow_distance"):
        _positive(limits.get(key), f"joint_limits.{key}", allow_zero=True)
    if limits["stop_distance"] > limits["slow_distance"]:
        raise ConfigError("joint limit stop_distance cannot exceed slow_distance")
    collision = _require(data, "collision", dict, origin)
    _require(collision, "sphere_model", str, "collision")
    for section in ("inter_arm", "table", "self"):
        values = _require(collision, section, dict, "collision")
        _require(values, "enabled", bool, f"collision.{section}")
        for key in ("weight", "safety_margin", "proximity_scale"):
            _positive(values.get(key), f"collision.{section}.{key}", allow_zero=(key == "safety_margin"))

    stopping = _require(data, "stopping", dict, origin)
    if experiment in ("static", "directional_direct_vs_indirect"):
        _positive(stopping.get("convergence_speed"), "stopping.convergence_speed", allow_zero=True)
        _positive(stopping.get("convergence_hold_seconds"), "stopping.convergence_hold_seconds")
        _positive(stopping.get("minimum_run_seconds"), "stopping.minimum_run_seconds", allow_zero=True)
    trajectory = data.get("trajectory", {})
    if experiment == "translational":
        if not isinstance(trajectory, dict):
            raise ConfigError("trajectory must be a table")
        _positive(trajectory.get("lift_height"), "trajectory.lift_height")
        _positive(trajectory.get("lift_duration"), "trajectory.lift_duration")
        _positive(trajectory.get("lower_duration"), "trajectory.lower_duration")
    if experiment == "six_d":
        if not isinstance(trajectory, dict):
            raise ConfigError("trajectory must be a table")
        _positive(trajectory.get("start_to_intermediate_duration"), "trajectory.start_to_intermediate_duration")
        _positive(trajectory.get("intermediate_to_goal_duration"), "trajectory.intermediate_to_goal_duration")
    if experiment in ("translational", "six_d"):
        _positive(trajectory.get("final_hold_duration"), "trajectory.final_hold_duration", allow_zero=True)

    recording = _require(data, "recording", dict, origin)
    _require(recording, "data", bool, "recording")
    _require(recording, "video", bool, "recording")
    for key in ("video_width", "video_height", "video_fps"):
        _positive(recording.get(key), f"recording.{key}")
    if recording["video_width"] % 2 or recording["video_height"] % 2:
        raise ConfigError("recording video dimensions must be even")


def smoke_overlay(config: PaperConfig) -> PaperConfig:
    """Create an explicit short-run overlay without mutating paper defaults."""
    data = deepcopy(config.data)
    data["smoke"] = True
    data["paper"]["objectives"] = ["baseline"]
    data["initial_configurations"] = [data["initial_configurations"][0]]
    data["recording"]["video"] = False
    experiment = config.experiment
    if experiment in ("static", "directional_direct_vs_indirect"):
        data["stopping"]["fixed_duration_seconds"] = 0.02
    elif experiment == "translational":
        data["trajectory"]["lift_duration"] = 0.02
        data["trajectory"]["lower_duration"] = 0.02
        data["trajectory"]["final_hold_duration"] = 0.0
    else:
        data["trajectory"]["start_to_intermediate_duration"] = 0.02
        data["trajectory"]["intermediate_to_goal_duration"] = 0.02
        data["trajectory"]["final_hold_duration"] = 0.0
    return PaperConfig(source=config.source, data=data)


def toml_dumps(data: dict) -> str:
    """Serialize the supported resolved-config subset without a dependency."""
    lines: list[str] = []

    def scalar(value):
        if isinstance(value, bool):
            return "true" if value else "false"
        if isinstance(value, str):
            escaped = value.replace("\\", "\\\\").replace('"', '\\"')
            return f'"{escaped}"'
        if isinstance(value, (int, float)):
            return repr(value)
        if isinstance(value, list):
            return "[" + ", ".join(scalar(item) for item in value) + "]"
        raise TypeError(f"unsupported TOML value {value!r}")

    def write_table(mapping, prefix=""):
        scalars = {key: value for key, value in mapping.items() if not isinstance(value, (dict, list)) or (isinstance(value, list) and (not value or not isinstance(value[0], dict)))}
        tables = {key: value for key, value in mapping.items() if isinstance(value, dict)}
        arrays = {key: value for key, value in mapping.items() if isinstance(value, list) and value and isinstance(value[0], dict)}
        for key, value in scalars.items():
            lines.append(f"{key} = {scalar(value)}")
        for key, value in tables.items():
            name = f"{prefix}.{key}" if prefix else key
            lines.extend(("", f"[{name}]"))
            write_table(value, name)
        for key, entries in arrays.items():
            name = f"{prefix}.{key}" if prefix else key
            for entry in entries:
                lines.extend(("", f"[[{name}]]"))
                write_table(entry, name)

    write_table(data)
    return "\n".join(lines).lstrip() + "\n"
