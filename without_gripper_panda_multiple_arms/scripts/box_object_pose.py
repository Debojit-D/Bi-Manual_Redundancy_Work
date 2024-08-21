#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def box_pose_callback(data):
    try:
        # Get the index of the box model in the ModelStates message
        box_index = data.name.index('compliant_box')

        # Extract the pose of the box
        box_pose = data.pose[box_index]

        # Publish the pose to a new topic
        box_pose_pub.publish(box_pose)
    except ValueError:
        rospy.logwarn("Compliant box model not found in /gazebo/model_states")

def box_pose_publisher():
    global box_pose_pub

    # Initialize the ROS node
    rospy.init_node('box_pose_publisher', anonymous=True)

    # Publisher for the box pose
    box_pose_pub = rospy.Publisher('/compliant_box_pose', Pose, queue_size=10)

    # Subscriber to the /gazebo/model_states topic
    rospy.Subscriber('/gazebo/model_states', ModelStates, box_pose_callback)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        box_pose_publisher()
    except rospy.ROSInterruptException:
        pass
