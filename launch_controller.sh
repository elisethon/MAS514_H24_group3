#!/bin/bash
# Script to launch ROS 2 nodes for controller on the Dev PC

# Start the joy_node
echo "Starting joy_node..."
gnome-terminal -- bash -c "ros2 run joy joy_node; exec bash"

# Launch teleop_twist_joy 
echo "Starting teleop_twist_joy..."
gnome-terminal -- bash -c "ros2 launch teleop_twist_joy teleop-launch.py; exec bash"

