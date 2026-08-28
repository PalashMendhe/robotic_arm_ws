#!/bin/bash
source /opt/ros/lyrical/setup.bash
source /home/plsh/robotic_arm/install/setup.bash
ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false > /home/plsh/robotic_arm/moveit_new.log 2>&1
