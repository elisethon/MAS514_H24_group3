# MAS515 Project

## Project Overview
This project implements a ROS2 package for a differential drive robot simulation, featuring an odometry publisher node and robot description files. The robot can be visualized and controlled in RViz2.

## launching the robot
for launching the robot first make sure that you are in the workspace mas515
    use this command to enter in the mas515
        cd mas511
after that you need to build the robot by using this command
    build colcon 
than source it by using this command
    source install/setup.bash
finaly launch the launch file of the robot by this command
    ros2 launch robot_description display.launch.py 

NB!!!! the initial value of wheel velocity in the code is equal to zero. you need to follow bellow

## Project Structure
If you want to to add an initial wheel velocity, then change this part of the code in 
odometry_publisher.py file
            # Initialize wheel velocities
        self.left_wheel_velocity = 0.0 # add your desired wheel velocity (rad/s)
        self.right_wheel_velocity = 0.0 # add your desired wheel velocity (rad/s)

if these values are equal to 0.0, than you need to push a velocity command manually
using this commands in seperate terminal after launching the display.launch.py file. 

    ros2 topic pub /left_wheel_velocity std_msgs/Float32 "data: 2.0" --once
    ros2 topic pub /right_wheel_velocity std_msgs/Float32 "data: 2.0" --once


