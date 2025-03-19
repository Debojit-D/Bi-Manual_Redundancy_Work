from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink

_HERE = Path(__file__).parent
_XML = _HERE / "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/franka_emika_panda/dual_panda_scene.xml"

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    ## =================== ##
    ## Setup IK for both arms.
    ## =================== ##
    
    configuration = mink.Configuration(model)
    
    tasks = [
        end_effector_task_left := mink.FrameTask(
            frame_name="attachment_site_left",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        ),
        
        end_effector_task_right := mink.FrameTask(
            frame_name="attachment_site_right",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        ),
        
        posture_task := mink.PostureTask(model=model, cost=1e-2),
    ]

    ## =================== ##

    # IK settings.
    solver = "quadprog"
    pos_threshold = 1e-4
    ori_threshold = 1e-4
    max_iters = 20

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        mujoco.mj_forward(model, data)

        # Initialize the mocap targets at the end-effector sites.
        mink.move_mocap_to_frame(model, data, "target_left", "attachment_site_left", "site")
        mink.move_mocap_to_frame(model, data, "target_right", "attachment_site_right", "site")

        # Record the initial mocap target positions.
        initial_target_position_left = data.mocap_pos[0].copy()
        initial_target_position_right = data.mocap_pos[1].copy()

        # Trajectory parameters.
        amp = 0.15   # Amplitude of 5 cm.
        freq = 0.4   # Frequency of 0.5 Hz.

        rate = RateLimiter(frequency=500.0, warn=False)
        while viewer.is_running():
            # Update the mocap targets along a circular trajectory.
            t = data.time
            offset = np.array([amp * np.cos(2 * np.pi * freq * t),
                               amp * np.sin(2 * np.pi * freq * t),
                               0.0])
            
            data.mocap_pos[0] = initial_target_position_left + offset
            data.mocap_pos[1] = initial_target_position_right - offset

            # Read the updated mocap targets and set as new task targets.
            T_wt_left = mink.SE3.from_mocap_name(model, data, "target_left")
            T_wt_right = mink.SE3.from_mocap_name(model, data, "target_right")
            
            end_effector_task_left.set_target(T_wt_left)
            end_effector_task_right.set_target(T_wt_right)
            # print("Mocap Left:", data.mocap_pos[0])
            # print("Mocap Right:", data.mocap_pos[1])

            # Compute velocity and integrate into the next configuration.
            for i in range(max_iters):
                vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 1e-3)
                configuration.integrate_inplace(vel, rate.dt)
                
                # print("IK Velocities:", vel)
                print("Control (Left):", data.ctrl[:8])
                print("Control (Right):", data.ctrl[9:17])


                err_left = end_effector_task_left.compute_error(configuration)
                err_right = end_effector_task_right.compute_error(configuration)
                
                pos_achieved_left = np.linalg.norm(err_left[:3]) <= pos_threshold
                ori_achieved_left = np.linalg.norm(err_left[3:]) <= ori_threshold
                
                pos_achieved_right = np.linalg.norm(err_right[:3]) <= pos_threshold
                ori_achieved_right = np.linalg.norm(err_right[3:]) <= ori_threshold
                
                if pos_achieved_left and ori_achieved_left and pos_achieved_right and ori_achieved_right:
                    break

            data.ctrl[:8] = configuration.q[:8]  # Left Arm
            data.ctrl[8:16] = configuration.q[9:17]  # Right Arm

            # data.ctrl = configuration.q[:16]  # Assuming 8 DOF per arm
            mujoco.mj_step(model, data)

            # Visualize at fixed FPS.
            viewer.sync()
            rate.sleep()
