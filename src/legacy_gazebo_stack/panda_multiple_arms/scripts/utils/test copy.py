import rospy
from geometry_msgs.msg import Pose
import threading

# Global variables to store the pose data
current_position = None
current_orientation = None

def pose_callback(data):
    global current_position, current_orientation
    current_position = data.position
    current_orientation = data.orientation

def start_listener():
    rospy.init_node('pose_listener', anonymous=True)
    rospy.Subscriber('/compliant_box_pose', Pose, pose_callback)
    rospy.spin()

def get_current_pose():
    # This function returns the latest position and orientation
    print(current_position, current_orientation)
    return current_position, current_orientation

# Function to start the listener in a separate thread
def run_listener_in_background():
    listener_thread = threading.Thread(target=start_listener)
    listener_thread.daemon = True
    listener_thread.start()

if __name__ == '__main__':
    # Start the listener in the background
    run_listener_in_background()

    # Test by calling get_current_pose() multiple times
    import time

    # Wait for some time to ensure the listener has received some data
    time.sleep(2)

    # Fetch the current pose
    position, orientation = get_current_pose()

    # Output the results
    if position and orientation:
        print("Test: Position:", position)
        print("Test: Orientation:", orientation)
    else:
        print("Test: Pose data is not available.")
