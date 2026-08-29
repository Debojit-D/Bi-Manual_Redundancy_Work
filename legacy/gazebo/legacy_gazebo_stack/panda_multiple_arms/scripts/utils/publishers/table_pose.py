#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose

def table_pose_callback(data):
    try:
        # Get the index of the table model in the ModelStates message
        table_index = data.name.index('table')

        # Extract the pose of the table
        table_pose = data.pose[table_index]

        # Publish the pose to a new topic
        table_pose_pub.publish(table_pose)
    except ValueError:
        rospy.logwarn("Compliant table model not found in /gazebo/model_states")

def table_pose_publisher():
    global table_pose_pub

    # Initialize the ROS node
    rospy.init_node('table_pose_publisher', anonymous=True)

    # Publisher for the table pose
    table_pose_pub = rospy.Publisher('/table_pose', Pose, queue_size=10)

    # Subscriber to the /gazebo/model_states topic
    rospy.Subscriber('/gazebo/model_states', ModelStates, table_pose_callback)

    # Set rate to 100 Hz
    rate = rospy.Rate(100)

    # Keep the node running
    while not rospy.is_shutdown():
        rate.sleep()  # Sleep to maintain the 100 Hz rate

if __name__ == '__main__':
    try:
        table_pose_publisher()
    except rospy.ROSInterruptException:
        pass
