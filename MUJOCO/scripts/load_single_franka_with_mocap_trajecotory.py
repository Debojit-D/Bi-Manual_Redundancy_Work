from pathlib import Path
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter

import mink

_HERE = Path(__file__).parent
_XML = _HERE / "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/franka_emika_panda/mjx_scene.xml"

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(_XML.as_posix())
    data = mujoco.MjData(model)

    ## =================== ##
    ## Setup IK.
    ## =================== ##

    configuration = mink.Configuration(model)

    tasks = [
        end_effector_task := mink.FrameTask(
            frame_name="attachment_site",
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

        mujoco.mj_resetDataKeyframe(model, data, model.key("home").id)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        mujoco.mj_forward(model, data)

        # Initialize the mocap target at the end-effector site.
        mink.move_mocap_to_frame(model, data, "target", "attachment_site", "site")
        # Record the initial mocap target position.
        initial_target_position = data.mocap_pos[0].copy()

        # Trajectory parameters.
        amp = 0.15   # Amplitude of 5 cm.
        freq = 0.4   # Frequency of 0.5 Hz.

        rate = RateLimiter(frequency=500.0, warn=False)
        while viewer.is_running():
            # Update the mocap target along a circular trajectory.
            t = data.time
            offset = np.array([amp * np.cos(2 * np.pi * freq * t),
                               amp * np.sin(2 * np.pi * freq * t),
                               0.0])
            data.mocap_pos[0] = initial_target_position + offset

            # Read the updated mocap target and set as the new task target.
            T_wt = mink.SE3.from_mocap_name(model, data, "target")
            end_effector_task.set_target(T_wt)

            # Compute velocity and integrate into the next configuration.
            for i in range(max_iters):
                vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 1e-3)
                configuration.integrate_inplace(vel, rate.dt)
                err = end_effector_task.compute_error(configuration)
                pos_achieved = np.linalg.norm(err[:3]) <= pos_threshold
                ori_achieved = np.linalg.norm(err[3:]) <= ori_threshold
                if pos_achieved and ori_achieved:
                    break

            data.ctrl = configuration.q[:8]
            mujoco.mj_step(model, data)

            # Visualize at fixed FPS.
            viewer.sync()
            rate.sleep()
