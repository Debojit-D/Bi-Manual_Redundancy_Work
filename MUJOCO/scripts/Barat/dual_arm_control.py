from pathlib import Path
import mujoco
import numpy as np
import mink
from loop_rate_limiters import RateLimiter
import time

class DualArmCollisionController:
    def __init__(self, model_path, solver="quadprog", pos_threshold=1e-4, ori_threshold=1e-4,
                 max_iters=20, collision_gain=1.85, min_distance=0.05, collision_det_distance=0.01):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.solver = solver
        self.pos_threshold = pos_threshold
        self.ori_threshold = ori_threshold
        self.max_iters = max_iters
        
        # Reset robot state to keyframe "home1"
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key("home1").id)
        mujoco.mj_forward(self.model, self.data)
        
        # Set up the IK configuration
        self.configuration = mink.Configuration(self.model)
        
        # 2. Create a separate Mink configuration to store the initial joint state
        # self.initial_configuration = mink.Configuration(self.model)
        # self.initial_configuration.q = np.copy(self.configuration.q)
        
        self.initial_configuration = mink.Configuration(
            self.model,
            q=np.copy(self.configuration.q)
        )


        # Setup tasks.
        self.end_effector_task_left = mink.FrameTask(
            frame_name="attachment_site_left",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.end_effector_task_right = mink.FrameTask(
            frame_name="attachment_site_right",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        self.posture_task = mink.PostureTask(model=self.model, cost=1e-2)
        self.tasks = [
            self.end_effector_task_left,
            self.end_effector_task_right,
            self.posture_task,
        ]
        
        # Setup collision avoidance limit.
        self.limits = [
            mink.CollisionAvoidanceLimit(
                model=self.model,
                geom_pairs=(
                    (
                        ["link0_c_l", "link1_c_l", "link2_c_l", "link3_c_l", "link4_c_l",
                         "link5_c0_l", "link5_c1_l", "link5_c2_l", "link6_c_l", "link7_c_l", "hand_c_l"],
                        ["link0_c_r", "link1_c_r", "link2_c_r", "link3_c_r", "link4_c_r",
                         "link5_c0_r", "link5_c1_r", "link5_c2_r", "link6_c_r", "link7_c_r", "hand_c_r"],
                    ),
                ),
                gain=collision_gain,
                minimum_distance_from_collisions=min_distance,
                collision_detection_distance=collision_det_distance,
                bound_relaxation=0.0,
            ),
        ]
        
        # **Update mocap targets to match end-effector frames.**
        mink.move_mocap_to_frame(self.model, self.data, "target_left", "attachment_site_left", "site")
        mink.move_mocap_to_frame(self.model, self.data, "target_right", "attachment_site_right", "site")
        mujoco.mj_forward(self.model, self.data)
        
        # Now store the initial target positions from the updated simulation state.
        self.initial_target_position_left = self.data.mocap_pos[0].copy()
        self.initial_target_position_right = self.data.mocap_pos[1].copy()

        self.rate = RateLimiter(frequency=500.0, warn=False)

    def update_targets(self, target_left, target_right):
        """
        Update mocap targets.
        target_left/right: dictionaries with keys 'position' (3,) and 'orientation' (4,)
        """
        self.data.mocap_pos[0] = target_left['position']
        self.data.mocap_pos[1] = target_right['position']
        try:
            mink.set_mocap_quat(self.model, self.data, "target_left", target_left['orientation'])
            mink.set_mocap_quat(self.model, self.data, "target_right", target_right['orientation'])
        except AttributeError:
            self.data.mocap_quat[0] = target_left['orientation']
            self.data.mocap_quat[1] = target_right['orientation']

    
    
    
    def solve_ik_step(self, dt):
        for _ in range(self.max_iters):
            # Update end-effector task targets
            T_wt_left = mink.SE3.from_mocap_name(self.model, self.data, "target_left")
            T_wt_right = mink.SE3.from_mocap_name(self.model, self.data, "target_right")
            self.end_effector_task_left.set_target(T_wt_left)
            self.end_effector_task_right.set_target(T_wt_right)

            # Set the posture task target from the current configuration
            self.posture_task.set_target_from_configuration(self.configuration)

            # Define a regularization weight
            regularization_weight = 1e-2  

            # Solve IK with regularization
            vel = mink.solve_ik(
                self.configuration, 
                self.tasks, 
                dt, 
                self.solver, 
                1e-3,  # Solver tolerance
                limits=self.limits
            )

            # Apply regularization to prevent stretching
            # vel -= regularization_weight * np.clip(self.configuration - self.initial_configuration, -0.1, 0.1)

            # Clamp velocity
            # max_velocity = 0.2  # Tune this as needed
            # vel = np.clip(vel, -max_velocity, max_velocity)

            # Integrate joint values
            self.configuration.integrate_inplace(vel, dt)

            # Enforce joint limits
            # self.configuration = np.clip(self.configuration, self.limits[:, 0] + 0.1, self.limits[:, 1] - 0.1)

            # Compute IK error
            err_left = self.end_effector_task_left.compute_error(self.configuration)
            err_right = self.end_effector_task_right.compute_error(self.configuration)

            # Debugging output
            print(f"🛑 IK Error Left Pos: {np.linalg.norm(err_left[:3])}, Rot: {np.linalg.norm(err_left[3:])}")
            print(f"🛑 IK Error Right Pos: {np.linalg.norm(err_right[:3])}, Rot: {np.linalg.norm(err_right[3:])}")

            # Stop if error is small
            if (np.linalg.norm(err_left[:3]) <= self.pos_threshold and
                np.linalg.norm(err_left[3:]) <= self.ori_threshold and
                np.linalg.norm(err_right[:3]) <= self.pos_threshold and
                np.linalg.norm(err_right[3:]) <= self.ori_threshold):
                break 
    
    # def solve_ik_step(self, dt):
    #     for _ in range(self.max_iters):
    #         # Update end-effector task targets from current mocap targets.
    #         T_wt_left = mink.SE3.from_mocap_name(self.model, self.data, "target_left")
    #         T_wt_right = mink.SE3.from_mocap_name(self.model, self.data, "target_right")
    #         self.end_effector_task_left.set_target(T_wt_left)
    #         self.end_effector_task_right.set_target(T_wt_right)

    #         # Set the posture task target from the current configuration.
    #         self.posture_task.set_target_from_configuration(self.configuration)

    #         # Define a regularization weight (tune as needed)
    #         regularization_weight = 1e-2  

    #         # Solve IK to get joint velocities.
    #         vel = mink.solve_ik(
    #             self.configuration, 
    #             self.tasks, 
    #             dt, 
    #             self.solver, 
    #             1e-3,  # Solver tolerance
    #             limits=self.limits
    #         )

    #         # --- Regularization Step ---
    #         # Extract joint values from the current and initial configurations.
    #         current_joints = self.configuration.q  # should be a NumPy array, e.g. shape (24,)
    #         initial_joints = self.initial_configuration.q  # should have the same shape

    #         # Debug prints (optional)
    #         print(f"current_joints shape: {current_joints.shape}")
    #         print(f"initial_joints shape: {initial_joints.shape}")
    #         print(f"Vel shape before regularization: {vel.shape}")

    #         # Ensure vel has the correct shape.
    #         if vel.shape[0] > self.model.nv:
    #             vel = vel[:self.model.nv]
    #         elif vel.shape[0] < self.model.nv:
    #             vel = np.pad(vel, (0, self.model.nv - vel.shape[0]), mode='constant')

    #         # Apply regularization to dampen large changes.
    #         vel -= regularization_weight * np.clip(current_joints - initial_joints, -0.1, 0.1)

    #         # Double-check that vel now has the correct size.
    #         if vel.shape[0] != self.model.nv:
    #             if vel.shape[0] > self.model.nv:
    #                 vel = vel[:self.model.nv]
    #             else:
    #                 vel = np.pad(vel, (0, self.model.nv - vel.shape[0]), mode='constant')
    #         print(f"Vel shape after regularization and trim: {vel.shape}")

    #         # Integrate the joint velocities.
    #         self.configuration.integrate_inplace(vel, dt)

    #         # --- Clamping Joint Values ---
    #         # Convert self.limits to a NumPy array if needed.
    #         if isinstance(self.limits, list):
    #             self.limits = np.array(self.limits)
    #         # If self.limits is 1D, reshape it.
    #         if self.limits.ndim == 1:
    #             self.limits = self.limits.reshape(-1, 2)

    #         # Clamp joint values within safe limits.
    #         new_q = np.clip(self.configuration.q, self.limits[:, 0] + 0.1, self.limits[:, 1] - 0.1)
    #         self.configuration.set_q(new_q)

    #         # Compute IK error.
    #         err_left = self.end_effector_task_left.compute_error(self.configuration)
    #         err_right = self.end_effector_task_right.compute_error(self.configuration)

    #         print(f"🛑 IK Error Left Pos: {np.linalg.norm(err_left[:3])}, Rot: {np.linalg.norm(err_left[3:])}")
    #         print(f"🛑 IK Error Right Pos: {np.linalg.norm(err_right[:3])}, Rot: {np.linalg.norm(err_right[3:])}")

    #         # Break the loop if errors are small.
    #         if (np.linalg.norm(err_left[:3]) <= self.pos_threshold and
    #             np.linalg.norm(err_left[3:]) <= self.ori_threshold and
    #             np.linalg.norm(err_right[:3]) <= self.pos_threshold and
    #             np.linalg.norm(err_right[3:]) <= self.ori_threshold):
    #             break


    def reset_mocap_targets(self):
        """Reset mocap bodies to their correct initial positions."""
        self.data.mocap_pos[0] = np.array([0.1, 0.0, 0.5])  # Left Target
        self.data.mocap_quat[0] = np.array([0, 1, 0, 0])  # Identity Quaternion

        self.data.mocap_pos[1] = np.array([0.1, 0.0, 0.5])  # Right Target
        self.data.mocap_quat[1] = np.array([0, 1, 0, 0])  # Identity Quaternion

        # Force MuJoCo to apply these changes
        mujoco.mj_forward(self.model, self.data)

        print("✅ Mocap targets reset to:", self.data.mocap_pos)

    def step_simulation(self, dt):
        self.solve_ik_step(dt)
        self.data.ctrl[:8] = self.configuration.q[:8]
        self.data.ctrl[8:16] = self.configuration.q[9:17]
        mujoco.mj_step(self.model, self.data)
        self.rate.sleep()

    def run(self, total_time=5.0, dt=0.005):
        start_time = time.time()
        try:
            from mujoco.viewer import launch_passive
            with launch_passive(model=self.model, data=self.data) as viewer:
                while viewer.is_running() and (time.time() - start_time < total_time):
                    self.step_simulation(dt)
                    viewer.sync()
        except (AttributeError, ImportError):
            print("Viewer not available, running headless simulation.")
            while time.time() - start_time < total_time:
                self.step_simulation(dt)
                time.sleep(dt)

if __name__ == "__main__":
    _HERE = Path(__file__).parent
    _XML = _HERE / "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
    controller = DualArmCollisionController(model_path=_XML.as_posix())

    # Define new target poses for each arm.
    target_left = {
        "position": controller.initial_target_position_left + np.array([0.1, 0.2, 0.0]),
        "orientation": np.array([1, 0, 0, 0])
    }
    target_right = {
        "position": controller.initial_target_position_right + np.array([-0.1, -0.2, 0.0]),
        "orientation": np.array([1, 0, 0, 0])
    }
    controller.update_targets(target_left, target_right)
    controller.run(total_time=10.0, dt=0.005)
