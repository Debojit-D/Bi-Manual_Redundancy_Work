import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np
import tf.transformations as tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def get_box_state():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        box_state = get_model_state('compliant_box', 'world')  # 'world' frame
        
        position = box_state.pose.position
        orientation = box_state.pose.orientation
        
        rospy.loginfo(f"Box Position: x={position.x}, y={position.y}, z={position.z}")
        rospy.loginfo(f"Box Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
        
        return position, orientation
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None

def minimum_jerk_trajectory(t, t_final, start, end):
    """Generate minimum jerk trajectory for smooth transitions."""
    tau = t / t_final
    position = start + (end - start) * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)
    return position

def slerp_quaternion(start_quat, end_quat, t, t_final):
    """SLERP (Spherical Linear Interpolation) for smooth quaternion transitions."""
    tau = t / t_final
    return tf.quaternion_slerp(start_quat, end_quat, tau)

def generate_minimum_jerk_trajectory(points, orientations, total_time=5.0, time_step=0.01):
    """Generate a minimum jerk and quaternion trajectory between multiple waypoints."""
    position_trajectory = []
    orientation_trajectory = []

    # Divide the time equally for each segment
    segment_time = total_time / (len(points) - 1)

    # Generate trajectory for each segment
    for i in range(len(points) - 1):
        start_pos = np.array(points[i])
        end_pos = np.array(points[i + 1])
        
        start_orient = orientations[i]
        end_orient = orientations[i + 1]

        t = 0
        while t <= segment_time:
            # Interpolate position with minimum jerk interpolation
            interpolated_position = minimum_jerk_trajectory(t, segment_time, start_pos, end_pos)
            position_trajectory.append(interpolated_position.tolist())
            
            # Interpolate orientation with SLERP
            interpolated_orientation = slerp_quaternion(start_orient, end_orient, t, segment_time)
            orientation_trajectory.append(interpolated_orientation)

            t += time_step

    return position_trajectory, orientation_trajectory

def set_box_state(position, orientation):
    """Set the box position and orientation in Gazebo."""
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        box_state = ModelState()
        box_state.model_name = 'compliant_box'
        box_state.pose.position.x = position[0]
        box_state.pose.position.y = position[1]
        box_state.pose.position.z = position[2]
        box_state.pose.orientation.x = orientation[0]
        box_state.pose.orientation.y = orientation[1]
        box_state.pose.orientation.z = orientation[2]
        box_state.pose.orientation.w = orientation[3]
        
        set_model_state(box_state)
        rospy.loginfo(f"Moved box to Position: {position}, Orientation: {orientation}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def move_box_along_trajectory(position_trajectory, orientation_trajectory, marker_pub, delay=0.01):
    """Move the box along the calculated position and orientation trajectory, and publish markers for RViz."""
    
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # Set marker properties
    marker.scale.x = 0.01  # Line thickness
    marker.color.a = 1.0   # Opacity
    marker.color.r = 0.0   # Red
    marker.color.g = 1.0   # Green
    marker.color.b = 0.0   # Blue

    for i in range(len(position_trajectory)):
        set_box_state(position_trajectory[i], orientation_trajectory[i])

        # Add points to the marker for visualization in RViz
        point = Point()
        point.x = position_trajectory[i][0]
        point.y = position_trajectory[i][1]
        point.z = position_trajectory[i][2]
        marker.points.append(point)

        # Publish the marker
        marker_pub.publish(marker)

        rospy.sleep(delay)

def move_box_to_target():
    """Main function to control the movement of the box along the trajectory and publish RViz markers."""
    # Get the initial position and orientation of the box
    initial_pos, initial_orient = get_box_state()

    if initial_pos is None:
        rospy.logerr("Failed to get initial position and orientation of the box.")
        return

    # Hardcoded positions and orientations (modifiable)
    initial_pos_list = [initial_pos.x, initial_pos.y, initial_pos.z]
    point_1 = [0.3, -0.5, 1.6]  # Intermediate point 1
    target_position = [0.1660, -0.7158, 1.1]  # Target position

    # Quaternions for the corresponding positions (modifiable orientations)
    initial_orient_list = [initial_orient.x, initial_orient.y, initial_orient.z, initial_orient.w]
    orient_1 = [0, 0, 0, 1]  # Intermediate orientation
    target_orientation = [0, 0, -0.7157, 0.7157]  # Target orientation

    # All points and orientations in sequence (initial -> point 1 -> target)
    points = [initial_pos_list, point_1, target_position]
    orientations = [initial_orient_list, orient_1, target_orientation]

    # Generate a minimum jerk trajectory for the box's position and orientation
    position_trajectory, orientation_trajectory = generate_minimum_jerk_trajectory(points, orientations)

    # Create a publisher for RViz markers
    marker_pub = rospy.Publisher('/box_trajectory_marker', Marker, queue_size=10)

    # Move the box along the generated trajectory and visualize the trajectory in RViz
    move_box_along_trajectory(position_trajectory, orientation_trajectory, marker_pub)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('box_trajectory_mover')

        # Move the box along the trajectory and publish the trajectory in RViz
        move_box_to_target()

    except rospy.ROSInterruptException:
        pass
