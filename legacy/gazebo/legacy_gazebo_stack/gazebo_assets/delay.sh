#!/bin/bash
echo "Waiting for Gazebo to start..."
sleep 3  # Adjust if necessary
echo "Spawning table..."
rosrun gazebo_ros spawn_model -urdf -file $(rospack find gazebo_assets)/table.urdf -model table -x 0.35 -y 0 -z 0.4 -Y 1.571

