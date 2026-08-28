#!/usr/bin/env python3
import subprocess
import time
import os

print("Killing old...")
os.system("pkill -9 -f ruby || true")
os.system("pkill -9 -f ros || true")
os.system("pkill -9 -f move_group || true")
os.system("pkill -9 -f gz || true")
time.sleep(2)

print("Starting moveit launch...")
launch_cmd = "source /opt/ros/lyrical/setup.bash && source /home/plsh/robotic_arm/install/setup.bash && ros2 launch arm_moveit_config moveit.launch.py use_rviz:=false"
p_launch = subprocess.Popen(["bash", "-c", launch_cmd], stdout=open("/tmp/moveit_clean.log", "w"), stderr=subprocess.STDOUT)

print("Waiting for MoveGroup to be ready...")
time.sleep(15)

print("Running pick_and_place...")
pnp_cmd = "source /opt/ros/lyrical/setup.bash && source /home/plsh/robotic_arm/install/setup.bash && ros2 run robotic_4dof_arm pick_and_place.py"
p_pnp = subprocess.Popen(["bash", "-c", pnp_cmd], stdout=open("/tmp/pnp_clean.log", "w"), stderr=subprocess.STDOUT)

p_pnp.wait()
print(f"pick_and_place finished with code {p_pnp.returncode}")

p_launch.terminate()
time.sleep(2)
os.system("pkill -9 -f ros || true")
os.system("pkill -9 -f gz || true")

