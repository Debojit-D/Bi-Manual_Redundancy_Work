"""cuRobo v0.8 collision-aware motion-planning adapter."""

import numpy as np
import torch

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState, Pose


class MotionPlannerBridge:
    """Expose a small NumPy-friendly interface around v0.8 MotionPlanner."""

    ROBOT_FILE = "franka.yml"
    ARM_JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]

    def __init__(self, world_cfg, interpolation_dt=0.02):
        # MotionPlanner's task configuration owns interpolation in v0.8. Keep
        # this attribute so playback code has an explicit requested cadence.
        self.interpolation_dt = interpolation_dt
        config = MotionPlannerCfg.create(
            robot=self.ROBOT_FILE,
            scene_model=world_cfg,
        )
        self.planner = MotionPlanner(config)
        self.joint_names = list(self.planner.joint_names)
        self.tool_frames = list(self.planner.tool_frames)
        self.device = config.device_cfg.device
        self.dtype = config.device_cfg.dtype

        print("[MotionPlannerBridge] Warming up cuRobo v0.8...")
        self.planner.warmup(enable_graph=True, num_warmup_iterations=5)
        print("[MotionPlannerBridge] Warmup complete.")

    def plan_to_pose(self, start_q, goal_xyz, goal_quat=(1.0, 0.0, 0.0, 0.0)):
        """Plan from seven Panda joints to one end-effector pose."""
        start = torch.as_tensor(
            np.asarray(start_q, dtype=np.float32),
            device=self.device,
            dtype=self.dtype,
        ).reshape(1, -1)
        start_state = JointState.from_position(start, joint_names=self.joint_names)

        goal_pose = Pose.from_list(
            [*goal_xyz, *goal_quat],
            device_cfg=self.planner.config.device_cfg,
        )
        goal = GoalToolPose.from_poses(
            {self.tool_frames[0]: goal_pose},
            ordered_tool_frames=[self.tool_frames[0]],
        )
        return self.planner.plan_pose(goal, start_state, max_attempts=10)

    def get_arm_positions(self, result) -> np.ndarray:
        """Extract ``[steps, 7]`` arm positions from a successful plan.

        cuRobo v0.8 restores locked joints in interpolated trajectories, so a
        Franka plan contains the seven arm joints plus two finger joints.
        Select by name rather than assuming either count or ordering.
        """
        if not self.succeeded(result):
            raise ValueError("cannot extract a trajectory from a failed plan")

        trajectory = result.get_interpolated_plan()
        names = list(trajectory.joint_names or [])
        missing = [name for name in self.ARM_JOINT_NAMES if name not in names]
        if missing:
            raise RuntimeError(f"planned trajectory is missing arm joints: {missing}")

        indices = [names.index(name) for name in self.ARM_JOINT_NAMES]
        positions = trajectory.position[..., indices]
        return positions.reshape(-1, len(indices)).detach().cpu().numpy()

    @staticmethod
    def succeeded(result) -> bool:
        return result is not None and bool(result.success.any().item())

    @staticmethod
    def status(result) -> str:
        if result is None:
            return "planner returned no result"
        return str(getattr(result, "status", "success" if result.success.any() else "failed"))
