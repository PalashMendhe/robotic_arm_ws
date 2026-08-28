#!/usr/bin/env python3
import os
import subprocess
import time

print("Killing old processes...")
os.system("pkill -9 -f ruby || true")
os.system("pkill -9 -f ros || true")
os.system("pkill -9 -f move_group || true")
os.system("pkill -9 -f rviz2 || true")
os.system("pkill -9 -f gz || true")

time.sleep(2)
print("Starting MoveIt...")
os.system("bash -c 'source /opt/ros/lyrical/setup.bash && source /home/plsh/robotic_arm/install/setup.bash && nohup ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false > /home/plsh/robotic_arm/moveit.log 2>&1 &'")

time.sleep(15)
print("Running pick and place...")
os.system("bash -c 'source /opt/ros/lyrical/setup.bash && source /home/plsh/robotic_arm/install/setup.bash && ros2 run robotic_4dof_arm pick_and_place.py > /home/plsh/robotic_arm/pnp.log 2>&1'")
print("Done!")

