#!/bin/bash
sleep 6

# Run all three scripts in parallel
rosrun panda_multiple_arms contact_bases_publisher.py &
rosrun panda_multiple_arms table_pose.py &
rosrun panda_multiple_arms manipulator_jacobian_and_end_effector_pos_publisher.py &
rosrun panda_multiple_arms grasp_matrix_and_hand_jacobian_publisher.py

# Wait for all background processes to finish
wait
