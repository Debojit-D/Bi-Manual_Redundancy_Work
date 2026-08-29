"""Repository-relative filesystem locations, independent of caller depth.

Replaces the historical pattern of each module computing the repository
root via ``Path(__file__).resolve().parents[N]``, which silently breaks
whenever a module moves to a different directory depth.
"""

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

MODELS_DIR = REPO_ROOT / "models"
ROBOT_MODELS_DIR = MODELS_DIR / "robots"
OBJECT_MODELS_DIR = MODELS_DIR / "objects"

CONFIGS_DIR = REPO_ROOT / "configs"

OUTPUTS_DIR = REPO_ROOT / "outputs"
OUTPUT_VIDEOS_DIR = OUTPUTS_DIR / "mujoco_videos"
OUTPUT_DATA_DIR = OUTPUTS_DIR / "mujoco_data"
OUTPUT_FONTS_DIR = OUTPUTS_DIR / ".fonts"
OUTPUT_COMPARISON_BATCHES_DIR = OUTPUTS_DIR / "equation8_comparison_batches"

MUJOCO_CUROBO_BRIDGE_DIR = REPO_ROOT / "mujoco_curobo_bridge"
