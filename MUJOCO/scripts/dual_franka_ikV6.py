import mujoco
import mujoco.viewer
import mink
import os
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R

MODEL_PATH = os.path.join(
    os.path.dirname(__file__),
    "../robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

def initialize_model():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)  # Force update after stepping
    configuration = mink.Configuration(model)
    
    return model, data, configuration

def ik_task_constraints(model):
    left_ee_task = mink.FrameTask("attachment_site_left", "site", position_cost=1.0, orientation_cost=1.0, lm_damping=1.0)
    right_ee_task = mink.FrameTask("attachment_site_right", "site", position_cost=1.0, orientation_cost=1.0, lm_damping=1.0)
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    tasks = [left_ee_task, right_ee_task, posture_task]
    solver = "osqp"
    pos_threshold, ori_threshold = 0.008, 0.008
    max_iters = 20  # Inner iterations per planning cycle
    rate = RateLimiter(frequency=40.0, warn=False)
    
    return left_ee_task, right_ee_task, posture_task, solver, tasks, pos_threshold, ori_threshold, max_iters, rate

def define_waypoints(data, site_ref_left, site_ref_right, target_quaternion_left, target_quaternion_right):
    # Define waypoints
    target_position_left_1 = np.copy(data.site_xpos[site_ref_left])
    target_position_left_1[0] -= 0.05
    target_quaternion_left_1 = target_quaternion_left.copy()

    target_position_right_1 = np.copy(data.site_xpos[site_ref_right])
    target_position_right_1[0] += 0.05
    target_quaternion_right_1 = target_quaternion_right.copy()

    target_position_left_2 = target_position_left_1.copy()
    target_position_left_2[0] += 0.07
    target_quaternion_left_2 = target_quaternion_left_1.copy()

    target_position_right_2 = target_position_right_1.copy()
    target_position_right_2[0] -= 0.07
    target_quaternion_right_2 = target_quaternion_right_1.copy()

    lifted_left_pos = target_position_left_2.copy()
    lifted_left_pos[2] += 0.26
    
    lifted_right_pos = target_position_right_2.copy()
    lifted_right_pos[2] += 0.26

    left_waypoints = [(target_position_left_1, target_quaternion_left_1), (target_position_left_2, target_quaternion_left_2)]
    right_waypoints = [(target_position_right_1, target_quaternion_right_1), (target_position_right_2, target_quaternion_right_2)]

    return left_waypoints, right_waypoints, lifted_left_pos, lifted_right_pos

def check_reached(measured_left_pos, current_left_pos, measured_left_quat, current_left_quat, measured_right_pos, current_right_pos, measured_right_quat, current_right_quat, pos_threshold, ori_threshold):
    err_pos_left = np.linalg.norm(measured_left_pos - current_left_pos)
    err_ori_left = quaternion_error(measured_left_quat, current_left_quat)
    err_pos_right = np.linalg.norm(measured_right_pos - current_right_pos)
    err_ori_right = quaternion_error(measured_right_quat, current_right_quat)

    return err_pos_left <= pos_threshold and err_ori_left <= ori_threshold and err_pos_right <= pos_threshold and err_ori_right <= ori_threshold

def initialize_mocap_targets(model, data):
    # Get site IDs for left and right end-effector
    site_ref_left, site_ref_right = model.site("site_left").id, model.site("site_right").id

    # Initialize mocap targets to match current sites
    data.mocap_pos[model.body("target_left").mocapid] = np.copy(data.site_xpos[site_ref_left])
    data.mocap_pos[model.body("target_right").mocapid] = np.copy(data.site_xpos[site_ref_right])

    target_quaternion_left = np.zeros(4)
    target_quaternion_right = np.zeros(4)
    mujoco.mju_mat2Quat(target_quaternion_left, data.site_xmat[site_ref_left])
    mujoco.mju_mat2Quat(target_quaternion_right, data.site_xmat[site_ref_right])

    data.mocap_quat[model.body("target_left").mocapid] = target_quaternion_left
    data.mocap_quat[model.body("target_right").mocapid] = target_quaternion_right
    
    return target_quaternion_left, target_quaternion_right

def update_mocap_targets(data, model, site_ref_left, site_ref_right, current_left_pos, current_left_quat, current_right_pos, current_right_quat):
    # Update mocap positions and orientations for targets
    data.mocap_pos[model.body("target_left").mocapid] = current_left_pos
    data.mocap_quat[model.body("target_left").mocapid] = current_left_quat
    data.mocap_pos[model.body("target_right").mocapid] = current_right_pos
    data.mocap_quat[model.body("target_right").mocapid] = current_right_quat

def quaternion_error(q_current, q_target):
    err1, err2 = np.linalg.norm(q_current - q_target), np.linalg.norm(q_current + q_target)
    return min(err1, err2)

if __name__ == "__main__":
    
    model, data, configuration = initialize_model()
    
    target_quaternion_left, target_quaternion_right = initialize_mocap_targets(model, data)
    
    left_ee_task, right_ee_task, posture_task, solver, tasks, pos_threshold, ori_threshold, max_iters, rate = ik_task_constraints(model)

    site_left_id, site_right_id = model.site("attachment_site_left").id, model.site("attachment_site_right").id
    site_ref_left, site_ref_right = model.site("site_left").id, model.site("site_right").id

    left_waypoints, right_waypoints, lifted_left_pos, lifted_right_pos = define_waypoints(data, site_ref_left, site_ref_right, target_quaternion_left, target_quaternion_right)
    
    with mujoco.viewer.launch_passive(model=model, data=data, show_left_ui=False, show_right_ui=False) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        mujoco.mj_forward(model, data)

        for wp_index in range(len(left_waypoints)):
            print(f"\nPlanning to waypoint {wp_index + 1}:")

            current_left_pos, current_left_quat = left_waypoints[wp_index]
            current_right_pos, current_right_quat = right_waypoints[wp_index]

            reached = False
            while not reached and viewer.is_running():
                update_mocap_targets(data, model, site_ref_left, site_ref_right, current_left_pos, current_left_quat, current_right_pos, current_right_quat)

                T_wt_left, T_wt_right = mink.SE3.from_mocap_name(model, data, "target_left"), mink.SE3.from_mocap_name(model, data, "target_right")
                left_ee_task.set_target(T_wt_left)
                right_ee_task.set_target(T_wt_right)

                for i in range(max_iters):
                    vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 5e-3)  # Increased dt for smoother movements
                    configuration.integrate_inplace(vel, rate.dt)
                    data.ctrl[0:7], data.ctrl[8:15] = configuration.q[0:7], configuration.q[9:16]
                    data.ctrl[7], data.ctrl[15] = 255, 255
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    rate.sleep()

                    measured_left_pos, measured_right_pos = data.site_xpos[site_left_id], data.site_xpos[site_right_id]
                    measured_left_quat, measured_right_quat = np.zeros(4), np.zeros(4)
                    mujoco.mju_mat2Quat(measured_left_quat, data.site_xmat[site_left_id])
                    mujoco.mju_mat2Quat(measured_right_quat, data.site_xmat[site_right_id])

                    reached = check_reached(
                        measured_left_pos, current_left_pos, measured_left_quat, current_left_quat,
                        measured_right_pos, current_right_pos, measured_right_quat, current_right_quat,
                        pos_threshold, ori_threshold
                    )

                if not reached:
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    rate.sleep()

            print(f"Waypoint {wp_index + 1} reached.\n")

        print("All waypoints reached. Closing gripper...")
        # Close the gripper
        data.ctrl[7], data.ctrl[15] = 0, 0
        for _ in range(100):
            data.ctrl[7], data.ctrl[15] = 0, 0
            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()
        print("Gripper closed.")
        
        # Move robot to lifted positions
        data.mocap_pos[model.body("target_left").mocapid] = lifted_left_pos
        data.mocap_pos[model.body("target_right").mocapid] = lifted_right_pos

        T_wt_left, T_wt_right = mink.SE3.from_mocap_name(model, data, "target_left"), mink.SE3.from_mocap_name(model, data, "target_right")
        left_ee_task.set_target(T_wt_left)
        right_ee_task.set_target(T_wt_right)
        
        # Reset gripper to open state
        data.ctrl[7], data.ctrl[15] = 0, 0
        
        # Final IK step to lift the object
        for i in range(max_iters):
                vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 5e-3)
                configuration.integrate_inplace(vel, rate.dt)
                data.ctrl[0:7], data.ctrl[8:15] = configuration.q[0:7], configuration.q[9:16]
                data.ctrl[7], data.ctrl[15] = 0, 0
                mujoco.mj_step(model, data)
                viewer.sync()
                rate.sleep()

        # Keep viewer open after execution
        while viewer.is_running():
            data.ctrl[0]=0.1
            data.ctrl[7], data.ctrl[15] = 0, 0
            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()


