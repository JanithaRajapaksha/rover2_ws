#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/rover2/rover2_ws/install/setup.bash

# Run your ROS launch file inside docker or locally
ros2 launch mecanum_control full_robot.launch.py
