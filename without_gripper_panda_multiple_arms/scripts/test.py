import numpy as np

def euler_to_rotation_matrix(euler_angles):
    """
    Converts Euler angles (roll, pitch, yaw) into a rotation matrix.
    """
    roll, pitch, yaw = map(float, euler_angles)  # Ensure angles are scalars

    print(f"Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

    # Rotation matrix for roll (rotation around x-axis)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    print(f"R_x:\n{R_x}")

    # Rotation matrix for pitch (rotation around y-axis)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    print(f"R_y:\n{R_y}")

    # Rotation matrix for yaw (rotation around z-axis)
    R_z = np.array([
        [float(np.cos(yaw)), float(-np.sin(yaw)), 0],
        [float(np.sin(yaw)), float(np.cos(yaw)), 0],
        [0, 0, 1]
    ])

    print(f"R_z:\n{R_z}")

    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))

    print(f"Combined Rotation Matrix:\n{R}")

    return R

# Test with small Euler angles
euler_angles = [1.00520088084074e-06, -1.8808975043634033e-06, -5.141887881606206e-05]
R = euler_to_rotation_matrix(euler_angles)
