#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_commander import RobotCommander

class EndEffectorPublisher:
    def __init__(self):
        rospy.init_node('end_effector_pose_publisher', anonymous=True, log_level=rospy.WARN)

        
        # Subscribers to joint states
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        
        # Publishers for end-effector poses
        self.left_ee_pub = rospy.Publisher("/left_end_effector_pose", PoseStamped, queue_size=10)
        self.right_ee_pub = rospy.Publisher("/right_end_effector_pose", PoseStamped, queue_size=10)
        
        # Robot Commander for kinematics
        self.robot = RobotCommander()
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def joint_states_callback(self, joint_states):
        # Get the pose of the left and right end-effectors
        try:
            left_transform = self.tf_buffer.lookup_transform("world", "left_arm_link7", rospy.Time())
            right_transform = self.tf_buffer.lookup_transform("world", "right_arm_link7", rospy.Time())
            
            # Create PoseStamped messages
            left_pose = PoseStamped()
            left_pose.header.stamp = rospy.Time.now()
            left_pose.header.frame_id = "world"
            left_pose.pose.position.x = left_transform.transform.translation.x
            left_pose.pose.position.y = left_transform.transform.translation.y
            left_pose.pose.position.z = left_transform.transform.translation.z
            left_pose.pose.orientation = left_transform.transform.rotation

            right_pose = PoseStamped()
            right_pose.header.stamp = rospy.Time.now()
            right_pose.header.frame_id = "world"
            right_pose.pose.position.x = right_transform.transform.translation.x
            right_pose.pose.position.y = right_transform.transform.translation.y
            right_pose.pose.position.z = right_transform.transform.translation.z
            right_pose.pose.orientation = right_transform.transform.rotation

            # Publish the end-effector poses
            self.left_ee_pub.publish(left_pose)
            self.right_ee_pub.publish(right_pose)
            
            rospy.loginfo("Published left and right end-effector poses.")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF lookup failed: {}".format(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ee_publisher = EndEffectorPublisher()
        ee_publisher.run()
    except rospy.ROSInterruptException:
        pass
