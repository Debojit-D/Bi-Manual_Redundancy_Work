import os
from pathlib import Path

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = BRIDGE_ROOT.parent

MUJOCO_SCENE = Path(
    os.environ.get(
        "MUJOCO_SCENE",
        REPOSITORY_ROOT
        / "MUJOCO"
        / "robot_descriptions"
        / "franka_emika_panda"
        / "scene.xml",
    )
).expanduser().resolve()
