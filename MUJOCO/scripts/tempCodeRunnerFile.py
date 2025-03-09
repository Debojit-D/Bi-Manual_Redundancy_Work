import mujoco
import mujoco.viewer
import mink
import os
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation as R
import imageio


MODEL_PATH = os.path.join(
    os.path.dirname(__file__),
    "../robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

def initialize_model():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    configuration = mink.Configuration(model)
    return model, data, configuration

def ik_task_constraints(model):
    left_ee_task = mink.FrameTask(
        "attachment_site_left", "site",
        position_cost=1.0, orientation_cost=1.0, lm_damping=1.0
    )
    right_ee_task = mink.FrameTask(
        "attachment_site_right", "site",
        position_cost=1.0, orientation_cost=1.0, lm_damping=1.0
    )
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    tasks = [left_ee_task, right_ee_task, posture_task]
    solver = "osqp"
    pos_threshold, ori_threshold = 0.008, 0.008
    max_iters = 20
    rate = RateLimiter(frequency=40.0, warn=False)
    return left_ee_task, right_ee_task, posture_task, solver, tasks, pos_threshold, ori_threshold, max_iters, rate

def define_waypoints(data, site_ref_left, site_ref_right, 
                     target_quaternion_left, target_quaternion_right):
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

    left_waypoints = [
        (target_position_left_1, target_quaternion_left_1),
        (target_position_left_2, target_quaternion_left_2),
    ]
    right_waypoints = [
        (target_position_right_1, target_quaternion_right_1),
        (target_position_right_2, target_quaternion_right_2),
    ]
    return left_waypoints, right_waypoints, lifted_left_pos, lifted_right_pos

def initialize_mocap_targets(model, data):
    site_ref_left = model.site("site_left").id
    site_ref_right = model.site("site_right").id
    data.mocap_pos[model.body("target_left").mocapid] = np.copy(data.site_xpos[site_ref_left])
    data.mocap_pos[model.body("target_right").mocapid] = np.copy(data.site_xpos[site_ref_right])

    target_quaternion_left = np.zeros(4)
    target_quaternion_right = np.zeros(4)
    mujoco.mju_mat2Quat(target_quaternion_left, data.site_xmat[site_ref_left])
    mujoco.mju_mat2Quat(target_quaternion_right, data.site_xmat[site_ref_right])

    data.mocap_quat[model.body("target_left").mocapid] = target_quaternion_left
    data.mocap_quat[model.body("target_right").mocapid] = target_quaternion_right
    return target_quaternion_left, target_quaternion_right

def update_mocap_targets(data, model,
                         site_ref_left, site_ref_right,
                         current_left_pos, current_left_quat,
                         current_right_pos, current_right_quat):
    data.mocap_pos[model.body("target_left").mocapid] = current_left_pos
    data.mocap_quat[model.body("target_left").mocapid] = current_left_quat
    data.mocap_pos[model.body("target_right").mocapid] = current_right_pos
    data.mocap_quat[model.body("target_right").mocapid] = current_right_quat

def check_reached(measured_left_pos, current_left_pos, measured_left_quat, current_left_quat,
                  measured_right_pos, current_right_pos, measured_right_quat, current_right_quat,
                  pos_threshold, ori_threshold):
    err_pos_left = np.linalg.norm(measured_left_pos - current_left_pos)
    err_ori_left = quaternion_error(measured_left_quat, current_left_quat)
    err_pos_right = np.linalg.norm(measured_right_pos - current_right_pos)
    err_ori_right = quaternion_error(measured_right_quat, current_right_quat)

    return (err_pos_left <= pos_threshold and err_ori_left <= ori_threshold
            and err_pos_right <= pos_threshold and err_ori_right <= ori_threshold)

def quaternion_error(q_current, q_target):
    err1 = np.linalg.norm(q_current - q_target)
    err2 = np.linalg.norm(q_current + q_target)
    return min(err1, err2)

if __name__ == "__main__":
    model, data, configuration = initialize_model()
    target_quaternion_left, target_quaternion_right = initialize_mocap_targets(model, data)
    
    (left_ee_task, right_ee_task, posture_task, solver, tasks,
     pos_threshold, ori_threshold, max_iters, rate) = ik_task_constraints(model)

    site_left_id = model.site("attachment_site_left").id
    site_right_id = model.site("attachment_site_right").id
    site_ref_left = model.site("site_left").id
    site_ref_right = model.site("site_right").id

    # Print rotation matrices for reference
    rotation_matrix_ref_left = data.site_xmat[site_ref_left].reshape(3, 3)
    rotation_matrix_ref_right = data.site_xmat[site_ref_right].reshape(3, 3)
    print("Rotation Matrix (site_left):\n", rotation_matrix_ref_left)
    print("Rotation Matrix (site_right):\n", rotation_matrix_ref_right)

    left_waypoints, right_waypoints, lifted_left_pos, lifted_right_pos = define_waypoints(
        data, site_ref_left, site_ref_right,
        target_quaternion_left, target_quaternion_right
    )

    # --------------------------
    # Create an off-screen context (for recording)
    # --------------------------
    # Scene/camera for off-screen rendering
    offscreen_scene = mujoco.MjvScene(model, maxgeom=2000)
    offscreen_cam = mujoco.MjvCamera()
    offscreen_ctx = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
    
    # We'll choose a 1280x720 viewport for the video
    VIDEO_WIDTH = 1280
    VIDEO_HEIGHT = 720
    viewport = mujoco.MjrRect(0, 0, VIDEO_WIDTH, VIDEO_HEIGHT)
    
    # Optionally set some camera parameters if you want a default
    # (We'll sync them from the viewer each frame, so starting value is not critical)
    mujoco.mjv_defaultCamera(offscreen_cam)

    # Open a video writer at 30 fps
    writer = imageio.get_writer("simulation_output.mp4", fps=30)

    # Launch the interactive viewer
    with mujoco.viewer.launch_passive(model=model, data=data,
                                      show_left_ui=False, show_right_ui=False) as viewer:
        # Initialize viewer camera
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.lookat[:] = [0.1, 0.0, 0.0]
        viewer.cam.azimuth = 70
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.5

        # Reset to a keyframe
        mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
        configuration.update(data.qpos)
        posture_task.set_target_from_configuration(configuration)
        mujoco.mj_forward(model, data)

        try:
            # Go through waypoints
            for wp_index in range(len(left_waypoints)):
                print(f"\nPlanning to waypoint {wp_index + 1}:")
                current_left_pos, current_left_quat = left_waypoints[wp_index]
                current_right_pos, current_right_quat = right_waypoints[wp_index]

                reached = False
                while not reached and viewer.is_running():
                    # Update the mocap targets
                    update_mocap_targets(
                        data, model,
                        site_ref_left, site_ref_right,
                        current_left_pos, current_left_quat,
                        current_right_pos, current_right_quat
                    )

                    T_wt_left = mink.SE3.from_mocap_name(model, data, "target_left")
                    T_wt_right = mink.SE3.from_mocap_name(model, data, "target_right")
                    left_ee_task.set_target(T_wt_left)
                    right_ee_task.set_target(T_wt_right)

                    for _ in range(max_iters):
                        vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 5e-3)
                        configuration.integrate_inplace(vel, rate.dt)

                        # Update joint positions / grippers
                        data.ctrl[0:7] = configuration.q[0:7]
                        data.ctrl[8:15] = configuration.q[9:16]
                        # keep grippers open
                        data.ctrl[7], data.ctrl[15] = 255, 255
                        mujoco.mj_step(model, data)

                        # Draw the viewer
                        viewer.sync()

                        # Off-screen render:
                        # 1) Copy camera from viewer to offscreen_cam
                        offscreen_cam.lookat[:] = viewer.cam.lookat[:]
                        offscreen_cam.distance  = viewer.cam.distance
                        offscreen_cam.azimuth   = viewer.cam.azimuth
                        offscreen_cam.elevation = viewer.cam.elevation

                        # 2) Update the scene
                        mujoco.mjv_updateScene(
                            model, data, offscreen_scene,
                            mujoco.MjvOption(), None, offscreen_cam,
                            mujoco.mjtCatBit.mjCAT_ALL.value
                        )

                        # 3) Render off-screen
                        mujoco.mjr_render(viewport, offscreen_scene, offscreen_ctx)

                        # 4) Read pixels
                        rgb = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
                        depth = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32)
                        mujoco.mjr_readPixels(rgb, depth, viewport, offscreen_ctx)
                        # Flip Y to get a right-side-up image
                        frame = np.flipud(rgb)
                        writer.append_data(frame)

                        rate.sleep()

                        # Check if we’ve reached the waypoint
                        measured_left_pos = data.site_xpos[site_left_id]
                        measured_right_pos = data.site_xpos[site_right_id]
                        measured_left_quat = np.zeros(4)
                        measured_right_quat = np.zeros(4)
                        mujoco.mju_mat2Quat(measured_left_quat, data.site_xmat[site_left_id])
                        mujoco.mju_mat2Quat(measured_right_quat, data.site_xmat[site_right_id])

                        reached = check_reached(
                            measured_left_pos, current_left_pos,
                            measured_left_quat, current_left_quat,
                            measured_right_pos, current_right_pos,
                            measured_right_quat, current_right_quat,
                            pos_threshold, ori_threshold
                        )
                        if reached:
                            break

                    if not reached:
                        mujoco.mj_step(model, data)
                        viewer.sync()
                        # Also capture an off-screen frame here
                        offscreen_cam.lookat[:] = viewer.cam.lookat[:]
                        offscreen_cam.distance  = viewer.cam.distance
                        offscreen_cam.azimuth   = viewer.cam.azimuth
                        offscreen_cam.elevation = viewer.cam.elevation
                        mujoco.mjv_updateScene(
                            model, data, offscreen_scene,
                            mujoco.MjvOption(), None, offscreen_cam,
                            mujoco.mjtCatBit.mjCAT_ALL.value
                        )
                        mujoco.mjr_render(viewport, offscreen_scene, offscreen_ctx)
                        rgb = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
                        depth = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32)
                        mujoco.mjr_readPixels(rgb, depth, viewport, offscreen_ctx)
                        writer.append_data(np.flipud(rgb))
                        rate.sleep()

                print(f"Waypoint {wp_index + 1} reached.\n")

            # Now close the gripper, etc.
            print("All waypoints reached. Closing gripper...")
            data.ctrl[7], data.ctrl[15] = 0, 0
            for _ in range(100):
                data.ctrl[7], data.ctrl[15] = 0, 0
                mujoco.mj_step(model, data)
                viewer.sync()

                # Capture frame
                offscreen_cam.lookat[:] = viewer.cam.lookat[:]
                offscreen_cam.distance  = viewer.cam.distance
                offscreen_cam.azimuth   = viewer.cam.azimuth
                offscreen_cam.elevation = viewer.cam.elevation
                mujoco.mjv_updateScene(
                    model, data, offscreen_scene,
                    mujoco.MjvOption(), None, offscreen_cam,
                    mujoco.mjtCatBit.mjCAT_ALL.value
                )
                mujoco.mjr_render(viewport, offscreen_scene, offscreen_ctx)
                rgb = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
                depth = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32)
                mujoco.mjr_readPixels(rgb, depth, viewport, offscreen_ctx)
                writer.append_data(np.flipud(rgb))

                rate.sleep()

            print("Gripper closed.")

            # Move robot to lifted positions
            data.mocap_pos[model.body("target_left").mocapid] = lifted_left_pos
            data.mocap_pos[model.body("target_right").mocapid] = lifted_right_pos

            T_wt_left = mink.SE3.from_mocap_name(model, data, "target_left")
            T_wt_right = mink.SE3.from_mocap_name(model, data, "target_right")
            left_ee_task.set_target(T_wt_left)
            right_ee_task.set_target(T_wt_right)

            # Final IK step to lift the object
            for _ in range(max_iters):
                vel = mink.solve_ik(configuration, tasks, rate.dt, solver, 5e-3)
                configuration.integrate_inplace(vel, rate.dt)

                data.ctrl[0:7] = configuration.q[0:7]
                data.ctrl[8:15] = configuration.q[9:16]
                data.ctrl[7], data.ctrl[15] = 0, 0
                mujoco.mj_step(model, data)

                viewer.sync()
                # Capture frame
                offscreen_cam.lookat[:] = viewer.cam.lookat[:]
                offscreen_cam.distance  = viewer.cam.distance
                offscreen_cam.azimuth   = viewer.cam.azimuth
                offscreen_cam.elevation = viewer.cam.elevation
                mujoco.mjv_updateScene(
                    model, data, offscreen_scene,
                    mujoco.MjvOption(), None, offscreen_cam,
                    mujoco.mjtCatBit.mjCAT_ALL.value
                )
                mujoco.mjr_render(viewport, offscreen_scene, offscreen_ctx)
                rgb = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
                depth = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32)
                mujoco.mjr_readPixels(rgb, depth, viewport, offscreen_ctx)
                writer.append_data(np.flipud(rgb))

                rate.sleep()

            print("Robot holding object. Close viewer to end.")

            # Run until viewer closed
            while viewer.is_running():
                data.ctrl[7], data.ctrl[15] = 0, 0
                mujoco.mj_step(model, data)
                viewer.sync()

                # Off-screen capture again
                offscreen_cam.lookat[:] = viewer.cam.lookat[:]
                offscreen_cam.distance  = viewer.cam.distance
                offscreen_cam.azimuth   = viewer.cam.azimuth
                offscreen_cam.elevation = viewer.cam.elevation

                mujoco.mjv_updateScene(
                    model, data, offscreen_scene,
                    mujoco.MjvOption(), None, offscreen_cam,
                    mujoco.mjtCatBit.mjCAT_ALL.value
                )
                mujoco.mjr_render(viewport, offscreen_scene, offscreen_ctx)
                rgb = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH, 3), dtype=np.uint8)
                depth = np.zeros((VIDEO_HEIGHT, VIDEO_WIDTH), dtype=np.float32)
                mujoco.mjr_readPixels(rgb, depth, viewport, offscreen_ctx)
                writer.append_data(np.flipud(rgb))

                rate.sleep()

        finally:
            writer.close()  # Ensure video is finalized

    print("Simulation finished. Video saved to 'simulation_output.mp4'.")
