#!/bin/bash
# Script to launch ROS 2 nodes on the Dev PC

# Start the joy_node for joystick control
echo "Starting joy_node..."
gnome-terminal -- bash -c "ros2 run joy joy_node; exec bash"

# Launch teleop_twist_joy to convert joystick input to twist messages
echo "Starting teleop_twist_joy..."
gnome-terminal -- bash -c "ros2 launch teleop_twist_joy teleop-launch.py; exec bash"

