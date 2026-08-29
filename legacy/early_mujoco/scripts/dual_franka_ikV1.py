"""
This code is now correctly implementing the IK for a dual arm franka setup.
"""

import mujoco
import mujoco.viewer
import mink
import os
import numpy as np
from loop_rate_limiters import RateLimiter

MODEL_PATH = os.path.join(
    os.path.dirname(__file__), 
    "../robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

if __name__ == "__main__":
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)
    
    configuration = mink.Configuration(model)
    print("Configuration q length:", len(configuration.q))

    tasks = [
        left_ee_task := mink.FrameTask(
            frame_name="attachment_site_left",
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        ),
        
        right_ee_task := mink.FrameTask(
            frame_name="attachment_site_right",  # ✅ Fixed frame name
            frame_type="site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        ),
        
        posture_task := mink.PostureTask(model=model, cost=1e-2),
    ]

    solver = "osqp"
    pos_threshold = 1e-4
    ori_threshold = 1e-4
    max_iters = 20
    
    target_position_left = np.array([-0.3, -0.2, 0.5])  
    target_quaternion_left = np.array([0, 1, 0, 1])  
    
    target_position_right = np.array([0.3, -0.4, 0.6])  
    target_quaternion_right = np.array([1, 1, 0, 0])  

    with mujoco.viewer.launch_passive(
        model=model, data=data, show_left_ui=False, show_right_ui=True
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        mujoco.mj_forward(model, data)

        # Initialize both targets
        mink.move_mocap_to_frame(model, data, "target_left", "attachment_site_left", "site")
        mink.move_mocap_to_frame(model, data, "target_right", "attachment_site_right", "site")

        rate = RateLimiter(frequency=50.0, warn=False)
        while viewer.is_running():
            data.mocap_pos[model.body("target_left").mocapid] = target_position_left
            data.mocap_quat[model.body("target_left").mocapid] = target_quaternion_left

            data.mocap_pos[model.body("target_right").mocapid] = target_position_right
            data.mocap_quat[model.body("target_right").mocapid] = target_quaternion_right

            # Update IK targets
            T_wt_left = mink.SE3.from_mocap_name(model, data, "target_left")
            T_wt_right = mink.SE3.from_mocap_name(model, data, "target_right")
            T_right_local = configuration.get_transform(
                source_name="attachment_site_right",
                source_type="site",
                dest_name="world",  # Right arm base
                dest_type="body",
            )


            left_ee_task.set_target(T_wt_left)
            right_ee_task.set_target(T_wt_right)

            for i in range(max_iters):
                vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 1e-3)
                configuration.integrate_inplace(vel, rate.dt)
                
                err_left = left_ee_task.compute_error(configuration)
                pos_achieved_left = np.linalg.norm(err_left[:3]) <= pos_threshold
                ori_achieved_left = np.linalg.norm(err_left[3:]) <= ori_threshold

                err_right = right_ee_task.compute_error(configuration)
                pos_achieved_right = np.linalg.norm(err_right[:3]) <= pos_threshold
                ori_achieved_right = np.linalg.norm(err_right[3:]) <= ori_threshold

                if pos_achieved_left and ori_achieved_left and pos_achieved_right and ori_achieved_right:
                    break

            data.ctrl[0:7] = configuration.q[0:7]
            data.ctrl[7] = 0
            data.ctrl[8:15] = configuration.q[9:16]
            data.ctrl[15] = 0 
            data.ctrl[15] = 0 
    

            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()
