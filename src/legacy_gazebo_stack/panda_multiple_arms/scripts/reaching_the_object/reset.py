import rospy
from std_srvs.srv import Empty

def reset_world():
    """Reset the world in Gazebo (positions, velocities, etc.)."""
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world_srv()
        rospy.loginfo("World has been reset.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset world: {e}")

reset_world()