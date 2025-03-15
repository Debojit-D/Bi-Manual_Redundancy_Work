import sys
import os

# Get the absolute path of the scripts directory
script_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "scripts"))

# Add the scripts directory to sys.path
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

import mujoco
import mujoco.viewer
import mink
import os
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R
import trajecotory_generation
import redundnacy_optimization

print("Loaded trajecotory_generation from:", trajecotory_generation.__file__)
print("Loaded redundnacy_optimization from:", redundnacy_optimization.__file__)

MODEL_PATH = os.path.join(
    os.path.dirname(__file__),
    "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

def generate_trajecotory_for_the_object(model,data):
    table_joint_name = "table_joint"  # Example name; adjust to match your model
    jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, table_joint_name)
    if jnt_id < 0:
        raise ValueError(f"Joint '{table_joint_name}' not found in model!")
    jnt_adr = model.jnt_qposadr[jnt_id]

    # MuJoCo expects: qpos for free joint = [x, y, z, qw, qx, qy, qz]
    initial_pos_mj = data.qpos[jnt_adr : jnt_adr + 3]
    initial_quat_mj = data.qpos[jnt_adr + 3 : jnt_adr + 7]  # [qw, qx, qy, qz]

    # Convert MuJoCo's [qw,qx,qy,qz] to the more common [x,y,z,w] if your
    # generate_smooth_quintic_trajectory expects [x, y, z, w]. 
    # If your function expects the scalar-last format, let's reorder:
    qw, qx, qy, qz = initial_quat_mj
    initial_quat = np.array([qx, qy, qz, qw])  # (x, y, z, w)

    # Define final position (move +0.2m in the z direction)
    final_pos = np.array(initial_pos_mj)
    final_pos[2] += 0.2

    # Keep orientation the same for demonstration
    final_quat = np.array([qx, qy, qz, qw])  # no change

    # Generate the trajectory
    pos_traj, quat_traj, lin_vel, lin_acc, ang_vel, t_vals = \
        trajecotory_generation.generate_smooth_quintic_trajectory(
            initial_position=initial_pos_mj,  # same as above
            final_position=final_pos,
            initial_quat=initial_quat,
            final_quat=final_quat,
            total_time=20.0,
            time_step=0.001
        )
        
    return pos_traj, quat_traj, lin_vel, ang_vel

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
    target_position_left_1[1] -= 0.05
    target_quaternion_left_1 = target_quaternion_left.copy()

    target_position_right_1 = np.copy(data.site_xpos[site_ref_right])
    target_position_right_1[1] += 0.05
    target_quaternion_right_1 = target_quaternion_right.copy()

    target_position_left_2 = target_position_left_1.copy()
    target_position_left_2[1] += 0.07
    target_quaternion_left_2 = target_quaternion_left_1.copy()

    target_position_right_2 = target_position_right_1.copy()
    target_position_right_2[1] -= 0.07
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
    
    desired_object_pose, desired_object_orientation, desired_object_linear_velocity, desired_object_angular_velocity = generate_trajecotory_for_the_object(model,data)
    
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

        # Initialize previous joint states correctly, skipping index 7 and 15
        previous_joint_states = np.zeros(14)
        previous_joint_states[:7] = data.qpos[0:7]   # Left arm joints (0-6)
        previous_joint_states[7:14] = data.qpos[9:16]  # Right arm joints (8-14), skipping gripper control index

        for i in range(len(desired_object_pose)):
            # Extract current object pose (position & quaternion)
            current_table_position = data.qpos[model.jnt_qposadr[model.jnt("table_joint").id]:
                                            model.jnt_qposadr[model.jnt("table_joint").id] + 3]

            current_table_quat = data.qpos[model.jnt_qposadr[model.jnt("table_joint").id] + 3:
                                        model.jnt_qposadr[model.jnt("table_joint").id] + 7]

            # Convert current quaternion to Euler angles
            rot_mat_current = np.zeros((3, 3), dtype=np.float64)
            mujoco.mju_quat2Mat(rot_mat_current.ravel(), current_table_quat)
            current_table_euler = R.from_matrix(rot_mat_current).as_euler('xyz', degrees=False)

            # Extract current linear & angular velocity
            current_table_linear_velocity = data.qvel[model.jnt_dofadr[model.jnt("table_joint").id]:
                                                    model.jnt_dofadr[model.jnt("table_joint").id] + 3]
            
            current_table_angular_velocity = data.qvel[model.jnt_dofadr[model.jnt("table_joint").id] + 3:
                                                    model.jnt_dofadr[model.jnt("table_joint").id] + 6]

            # Prepare full object state vectors
            object_current_position = np.concatenate((current_table_position, current_table_euler))
            object_current_velocity = np.concatenate((current_table_linear_velocity, current_table_angular_velocity))

            # Convert desired quaternion to Euler angles
            desired_table_quat = desired_object_orientation[i]
            rot_mat_desired = np.zeros((3, 3), dtype=np.float64)
            mujoco.mju_quat2Mat(rot_mat_desired.ravel(), desired_table_quat)
            desired_table_euler = R.from_matrix(rot_mat_desired).as_euler('xyz', degrees=False)

            # Prepare desired full state vectors
            object_desired_position = np.concatenate((desired_object_pose[i], desired_table_euler))
            object_desired_velocity = np.concatenate((desired_object_linear_velocity[i], desired_object_angular_velocity[i]))

            # Compute new joint states using redundancy optimization
            joint_vector_left_arm, joint_vector_right_arm = redundnacy_optimization.compute_next_phi(
                model, data,
                current_joint_states=previous_joint_states,
                object_desired_position=object_desired_position,
                object_current_position=object_current_position,
                object_desired_velocity=object_desired_velocity,
                object_current_velocity=object_current_velocity,
                delta_t=0.01,
                K_p=1.0
            )

            # Apply computed joint states to the robot
            data.ctrl[0:7] = joint_vector_left_arm  # Left arm
            data.ctrl[8:15] = joint_vector_right_arm  # Right arm

            # Update previous joint states
            previous_joint_states[:7] = joint_vector_left_arm
            previous_joint_states[7:14] = joint_vector_right_arm

            # Step the simulation
            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()




