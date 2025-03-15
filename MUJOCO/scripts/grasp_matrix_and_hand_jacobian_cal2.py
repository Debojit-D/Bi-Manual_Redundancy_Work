import numpy as np
import mujoco


def grasp_matrix_calculator(model, data):
    """
    Compute the full grasp matrix G by horizontally stacking the grasp sub-matrices
    for both left and right contact sites. The contact frames are rotated using the
    rotation matrix of the vention_table body.
    """
    # Get site IDs for the contacts
    site_left_id = model.site("site_left").id
    site_right_id = model.site("site_right").id
    site_top_middle_id = model.site("site_top_middle").id

    # Extract rotation matrices
    R_left = data.site_xmat[site_left_id].reshape(3, 3)   # Rotation of left contact site
    R_right = data.site_xmat[site_right_id].reshape(3, 3)  # Rotation of right contact site
    R_table = data.site_xmat[site_top_middle_id].reshape(3, 3)  # Rotation of site_top_middle

    R_left_rotated = R_left
    R_right_rotated = R_right

    # Extract the rotated contact frame axes
    s_left, t_left, n_left = R_left_rotated[:, 0], R_left_rotated[:, 1], R_left_rotated[:, 2]
    s_right, t_right, n_right = R_right_rotated[:, 0], R_right_rotated[:, 1], R_right_rotated[:, 2]

    # # Define contact positions relative to the table frame
    # p_left = data.site_xpos[site_left_id] - data.xpos[table_body_id]   # Contact position w.r.t table
    # p_right = data.site_xpos[site_right_id] - data.xpos[table_body_id]  # Contact position w.r.t table

    p_left = np.array([-0.1685, 0, 0])
    p_right = np.array([0.1685, 0, 0])

    # Rotate the contact positions into the table's frame
    p_left_rotated = np.dot(R_table, p_left)
    p_right_rotated = np.dot(R_table, p_right)

    # Compute the grasp sub-matrices for the left contact
    G_left = np.zeros((6, 6))
    G_left[0:3, 0] = s_left
    G_left[0:3, 1] = t_left
    G_left[0:3, 2] = n_left
    G_left[3:, 0] = np.cross(p_left_rotated, s_left)
    G_left[3:, 1] = np.cross(p_left_rotated, t_left)
    G_left[3:, 2] = np.cross(p_left_rotated, n_left)
    G_left[3:, 3] = s_left
    G_left[3:, 4] = t_left
    G_left[3:, 5] = n_left

    # Compute the grasp sub-matrices for the right contact
    G_right = np.zeros((6, 6))
    G_right[0:3, 0] = s_right
    G_right[0:3, 1] = t_right
    G_right[0:3, 2] = n_right
    G_right[3:, 0] = np.cross(p_right_rotated, s_right)
    G_right[3:, 1] = np.cross(p_right_rotated, t_right)
    G_right[3:, 2] = np.cross(p_right_rotated, n_right)
    G_right[3:, 3] = s_right
    G_right[3:, 4] = t_right
    G_right[3:, 5] = n_right

    # Compute full grasp matrix by horizontally stacking both sub-matrices
    G_full = np.hstack((G_left, G_right))

    #print(G_full)

    return G_full


def hand_jacobian_calculator(model, data):
    """
    Compute the new hand Jacobian matrix (size 12x14) using transformed contact frames
    from MuJoCo's site rotation matrices, ensuring computation is relative to the robot's base.
    """

    # Path to the MuJoCo XML file
    xml_path = "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/franka_emika_panda/mjx_panda.xml"

    # Load the MuJoCo model and create a data instance
    arm = mujoco.MjModel.from_xml_path(xml_path)

    # Get site IDs for attachment sites (where manipulators attach)
    site_left_id = model.site("site_left").id
    site_right_id = model.site("site_right").id

    left_franka_jacobian_site = arm.site("attachment_site").id
    right_franka_jacobian_site = arm.site("attachment_site").id

    # Extract rotation matrices for robot bases (franka1 and franka2)
    R_franka1 = data.xmat[model.body("franka1").id].reshape(3, 3)  # Rotation of franka1's base
    R_franka2 = data.xmat[model.body("franka2").id].reshape(3, 3)  # Rotation of franka2's base

    print(R_franka2)

    site_top_middle_id = model.site("site_top_middle").id

    # Get table body ID
    table_body_id = site_top_middle_id

    # Extract rotation matrices
    R_left = data.site_xmat[site_left_id].reshape(3, 3)  # Rotation of left attachment site
    R_right = data.site_xmat[site_right_id].reshape(3, 3)  # Rotation of right attachment site

    R_left_rotated = R_left 
    R_right_rotated = R_right 

    # Extract transformed contact frame basis (Wpki)
    Wpki_left = R_left_rotated  # Wpki for left
    Wpki_right = R_right_rotated  # Wpki for right

    # **Fix: Correct MuJoCo API Call for Jacobians**
    J_left = np.zeros((6, arm.nv))
    J_right = np.zeros((6, arm.nv))
    
    mujoco.mj_jacSite(arm, data, J_left[:3, :], J_left[3:, :], left_franka_jacobian_site)
    mujoco.mj_jacSite(arm, data, J_right[:3, :], J_right[3:, :], right_franka_jacobian_site)

    # **Fix: Correct DOF slicing (7 DOF per arm)**
    J_left = J_left[:, 0:7]      # columns 0..6 for left arm
    J_right = J_right[:, 0:7]   # columns 9..15 for right arm

    print(J_left)
    print(J_right)

    # print("J_left",J_left)
    # print("J_right",J_right)

    # Number of DOFs per arm
    dof = J_left.shape[1]  # Expected to be 7
    Jh_new = np.zeros((12, 14))  # (12x14) hand Jacobian matrix

    # Compute Jh for left and right manipulators
    for i, (Wpki, Rpki, Ji) in enumerate([(Wpki_left, R_franka1, J_left), 
                                          (Wpki_right, R_franka2, J_right)]):

        # Create block diagonal transformation
        W_block = np.block([
            [Wpki.T, np.zeros_like(Wpki.T)],  # First diagonal block
            [np.zeros_like(Wpki.T), Wpki.T]   # Second diagonal block
        ])

        R_block = np.block([
            [Rpki, np.zeros_like(Rpki)],  # First diagonal block
            [np.zeros_like(Rpki), Rpki]   # Second diagonal block
        ])

        # Apply transformations to Jacobian
        Jh_new[6 * i:6 * i + 6, dof * i:dof * i + dof] = np.dot(W_block, np.dot(R_block, Ji))

    return Jh_new
