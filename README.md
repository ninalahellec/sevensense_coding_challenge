# Sevensense Coding Challenge

## Overview

This repository contains:

- A Docker setup for ROS 2 Humble and Gazebo simulation.

- A ROS 2 package drive_transformer to transform Differential Drive velocity commands into Ackermann Drive commands.

- The Hunter UGV simulation, as a Git submodule.

```
.
├── docker_container/    # Dockerfile and helper scripts
├── ros2_ws/
│   ├── drive_transformer/   # diff_drive -> ackermann_drive
│   └── ugv_sim/             # Hunter UGV simulation (git submodule)
└── README.md
```

## Setup Instructions

1. Clone the repository with the submodule
```
git clone --recurse-submodules git@github.com:ninalahellec/sevensense_coding_challenge.git
cd sevensense_coding_challenge
```

2. Build and run the Docker container

Build the container
```
cd docker_container
./build.sh
```
Run the container:
```
./run.sh
```
This scripts mounts your local ROS 2 workspace into the container, so that you can edit code outside Docker and still use it inside.
By default, the following volume is mounted:
```
-v ~/Documents/sevensense/ros2_ws:/home/nina/ros2_ws/src
```
It must be adapted to your local folder structure.

3. Build your ROS 2 workspace

Once inside the Docker container:
```
colcon build
source install/setup.bash
```

## Running the gazebo simulation
Launch the full gazebo Hunter simulation:
```
ros2 launch hunter_se_gazebo hunter_se.launch.py
```

Launch the drive_transformer node:
```
ros2 run drive_transformer differential_to_ackermann.py
```

## Testing the Node

Publish a circular driving command:
```
ros2 topic pub /cmd_vel_diff geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 1.0}}"
```
Publish a pure rotation command (rotation on the spot):
```
ros2 topic pub /cmd_vel_diff geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}"
```
For a detailed explanation of the Drive Transformer node, please refer to the dedicated `README.md` inside the `drive_transformer` package.