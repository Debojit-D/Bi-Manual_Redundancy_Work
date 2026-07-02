"""
motion_gen_bridge.py

Wraps cuRobo's MotionGen for the Panda arm, using a cuRobo
WorldConfig (built via bridge.world_bridge.to_world_config) for
collision-aware planning.

Requires a CUDA-capable GPU. MotionGen.warmup() JIT-compiles CUDA
kernels on first use - this can take 30s-2min the first time you
run it, that is expected, not a hang.
"""

import torch

from curobo.types.math import Pose
from curobo.types.robot import JointState
from curobo.util_file import get_robot_configs_path, join_path, load_yaml
from curobo.wrap.reacher.motion_gen import (
    MotionGen,
    MotionGenConfig,
    MotionGenPlanConfig,
)
import numpy as np

class MotionGenBridge:

    ROBOT_FILE = "franka.yml"

    def __init__(self, world_cfg, interpolation_dt=0.02):
        """
        Parameters
        ----------
        world_cfg : curobo.geom.types.WorldConfig
            Output of bridge.world_bridge.to_world_config(world).
        """

        robot_dict = load_yaml(
            join_path(get_robot_configs_path(), self.ROBOT_FILE)
        )["robot_cfg"]

        motion_gen_config = MotionGenConfig.load_from_robot_config(
            robot_dict,
            world_cfg,
            interpolation_dt=interpolation_dt,
        )

        self.motion_gen = MotionGen(motion_gen_config)

        print("[MotionGenBridge] Warming up (first run can take a while)...")
        self.motion_gen.warmup()
        print("[MotionGenBridge] Warmup complete.")

        self.joint_names = motion_gen_config.robot_cfg.kinematics.cspace.joint_names

    def plan_to_pose(self, start_q, goal_xyz, goal_quat=(1.0, 0.0, 0.0, 0.0)):
        """
        Plan a collision-free joint trajectory from a start joint
        configuration to a target end-effector pose.

        Parameters
        ----------
        start_q : array-like, len 7
            Current arm joint configuration.
        goal_xyz : (x, y, z)
        goal_quat : (qw, qx, qy, qz)
            Defaults to identity orientation.

        Returns
        -------
        MotionGenResult
            result.success indicates whether planning succeeded.
            result.get_interpolated_plan() returns the trajectory.
        """

        start_q_arr = np.asarray(start_q, dtype=np.float32).reshape(1, -1)

        start_state = JointState.from_position(
            torch.from_numpy(start_q_arr).cuda(),
            joint_names=self.joint_names,
        )

        goal_pose = Pose.from_list([*goal_xyz, *goal_quat])

        result = self.motion_gen.plan_single(
            start_state,
            goal_pose,
            MotionGenPlanConfig(max_attempts=10),
        )

        return result