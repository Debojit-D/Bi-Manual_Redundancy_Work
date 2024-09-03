import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

# Global variable to store the matrix A and external forces F
A = None
F = None

def velocity_manipulability(A):
    AA_T = np.dot(A, A.T)
    return np.sqrt(np.linalg.det(AA_T))

def force_manipulability(A):
    AA_T = np.dot(A, A.T)
    return np.sqrt(np.linalg.det(np.linalg.matrix_power(AA_T, -1)))  # Assuming t = -1 for the inverse

def custom_manipulability(A, F):
    AA_T = np.dot(A, A.T)
    trace_AAT = np.trace(AA_T)
    trace_F = np.trace(F)
    return np.sqrt(trace_AAT / (trace_AAT - trace_F))

def A_callback(msg):
    global A
    # Assuming A is received as a flat list and needs to be reshaped
    A = np.array(msg.data).reshape((6, 6))  # Update this shape based on your specific application

def F_callback(msg):
    global F
    F = np.array(msg.data).reshape((6, 6))  # Similarly, update this based on actual dimensions

def main():
    rospy.init_node('manipulability_node')
    
    # Subscribers to the topics where A and F matrices are published
    rospy.Subscriber("/A_matrix", Float64MultiArray, A_callback)
    rospy.Subscriber("/force_matrix", Float64MultiArray, F_callback)
    
    # Publisher for manipulability indices
    manip_pub = rospy.Publisher("/manipulability_index", Float64MultiArray, queue_size=10)
    
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        if A is not None and F is not None:
            manip_indices = Float64MultiArray()
            manip_indices.data = [
                velocity_manipulability(A),
                force_manipulability(A),
                custom_manipulability(A, F)
            ]
            manip_pub.publish(manip_indices)
        rate.sleep()

if __name__ == '__main__':
    main()
