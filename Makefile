# ── Robotic Arm Workspace Makefile ──────────────────────────────────────────
SHELL  := /bin/bash
ROS    := /opt/ros/lyrical/setup.bash
WS     := $(shell pwd)/install/setup.bash

.PHONY: build sim run test clean docker-build docker-sim docker-run

## Local (sourced terminal) targets
build:
	source $(ROS) && colcon build --symlink-install

test:
	source $(ROS) && source $(WS) && \
	colcon test --packages-select robotic_4dof_arm && \
	colcon test-result --verbose

sim:
	source $(ROS) && source $(WS) && \
	ros2 launch arm_moveit_config moveit.launch.py

run:
	source $(ROS) && source $(WS) && \
	ros2 run robotic_4dof_arm pick_and_place.py

clean:
	rm -rf build/ install/ log/
	-killall -9 move_group ruby ros2 rviz2 >/dev/null 2>&1 || true
	-pkill -9 -f "gz sim" >/dev/null 2>&1 || true
	-pkill -9 -f "ros2" >/dev/null 2>&1 || true

## Docker targets
docker-build:
	docker build -t robotic_arm:lyrical .

docker-sim:
	xhost +local:docker
	docker compose up sim

docker-run:
	docker compose up run

docker-stop:
	docker compose down