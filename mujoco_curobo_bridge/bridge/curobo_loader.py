"""cuRobo v0.8 robot-model and forward-kinematics adapter."""

import numpy as np
import torch

from curobo.kinematics import Kinematics, KinematicsCfg
from curobo.types import JointState


class CuroboRobot:
    """Load the standard Franka model and expose NumPy-friendly FK helpers."""

    ROBOT_FILE = "franka.yml"

    def __init__(self, robot_file: str = ROBOT_FILE):
        self.config = KinematicsCfg.from_robot_yaml_file(robot_file)
        self.model = Kinematics(self.config)
        self.joint_names = list(self.model.joint_names)
        self.tool_frames = list(self.model.tool_frames)
        self.device = self.config.device_cfg.device
        self.dtype = self.config.device_cfg.dtype

    def joint_state(self, q) -> JointState:
        """Convert one joint vector to a batched v0.8 ``JointState``."""
        position = torch.as_tensor(
            np.asarray(q, dtype=np.float32),
            device=self.device,
            dtype=self.dtype,
        ).reshape(1, -1)
        return JointState.from_position(position, joint_names=self.joint_names)

    def fk(self, q):
        """Compute the v0.8 ``KinematicsState`` for one joint vector."""
        return self.model.compute_kinematics(self.joint_state(q))

    def get_end_effector_pose(self, state):
        """Return the pose of the model's first configured tool frame."""
        if not self.tool_frames:
            raise RuntimeError("cuRobo robot configuration has no tool frame")
        return state.tool_poses.get_link_pose(self.tool_frames[0])
