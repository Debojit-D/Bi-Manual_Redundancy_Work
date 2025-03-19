import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
from armds_control import DualArmController

# Define initial and target positions
x_R_left_init = np.array([0.0, 0.0, 0.0])
x_R_right_init = np.array([0.5, 0.0, 0.0])

x_A = np.array([0.2, 0.2, 0.0])
x_B = np.array([0.7, 0.2, 0.0])
x_C = np.array([0.2, 0.5, 0.0])
x_D = np.array([0.4, 0.7, 0.0])

# Initialize DualArmController
dual_arm = DualArmController(K_left=2.0, K_right=2.0)

# Initialize state variables
x_R_left = x_R_left_init.copy()
x_R_right = x_R_right_init.copy()
dt = 0.02  # time step
num_steps = 500  # total steps

# Define keyframe transitions (time at which transitions occur)
level_1_time = 100  # Asynchronous phase 1
level_1b_time = 200  # Asynchronous phase 2
level_2_time = 300  # Synchronous motion
level_3_time = 400  # Asynchronous with blending

# Animation setup
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-0.1, 1.0])
ax.set_ylim([-0.1, 1.0])
ax.set_zlim([-0.1, 1.0])
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

left_dot, = ax.plot([], [], [], 'ro', markersize=10)  # Left robot (red dot)
right_dot, = ax.plot([], [], [], 'bo', markersize=10)  # Right robot (blue dot)

def init():
    left_dot.set_data([], [])
    left_dot.set_3d_properties([])
    right_dot.set_data([], [])
    right_dot.set_3d_properties([])
    return left_dot, right_dot

def update(frame):
    global x_R_left, x_R_right
    
    if frame < level_1_time:
        dual_arm.set_mode("asynchronous")
        x_d_left, x_d_right = x_A, x_B  # Move left to A, right to B
    elif frame < level_1b_time:
        x_d_left, x_d_right = x_C, x_B  # Move left to C, right holds B
    elif frame < level_2_time:
        dual_arm.set_mode("synchronous")
        x_d_left, x_d_right = x_D, x_D  # Both move to D synchronously
    elif frame < level_3_time:
        dual_arm.set_mode("blending", frame, blend_duration=100)
        x_d_left, x_d_right = x_R_left_init, x_R_right_init  # Return to initial positions
    else:
        return left_dot, right_dot  # Stop updates
    
    # Compute control commands
    v_left, v_right = dual_arm.compute_commands(
        x_R_left, x_R_right, x_d_left, x_d_right, x_d_left, x_d_right, frame
    )
    
    # Update positions
    x_R_left += dt * v_left
    x_R_right += dt * v_right
    
    # Update plot (wrap values in lists to avoid RuntimeError)
    left_dot.set_data([x_R_left[0]], [x_R_left[1]])
    left_dot.set_3d_properties([x_R_left[2]])
    right_dot.set_data([x_R_right[0]], [x_R_right[1]])
    right_dot.set_3d_properties([x_R_right[2]])
    
    return left_dot, right_dot

ani = animation.FuncAnimation(fig, update, frames=num_steps, init_func=init, blit=False, interval=50)
plt.show()
