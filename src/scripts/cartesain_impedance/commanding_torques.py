#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray

def publish_position_commands():
    rospy.init_node('position_command_node', anonymous=True)

    # Publishers for joint positions for panda_1 and panda_2
    pub_panda_1 = rospy.Publisher('/panda_1/joint_command', Float64MultiArray, queue_size=10)
    pub_panda_2 = rospy.Publisher('/panda_2/joint_command', Float64MultiArray, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Example positions for the 7 joints of panda_1
        position_command_1 = Float64MultiArray()
        position_command_1.data = [-0.00118, -0.785, 0.00088, -2.356, 0.0005, 1.570, 0.784]

        # Example positions for the 7 joints of panda_2
        position_command_2 = Float64MultiArray()
        position_command_2.data = [0.00118, 0.785, -0.00088, 2.356, -0.0005, -1.570, -0.784]

        # Publish to the topics
        pub_panda_1.publish(position_command_1)
        pub_panda_2.publish(position_command_2)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_position_commands()
    except rospy.ROSInterruptException:
        pass
