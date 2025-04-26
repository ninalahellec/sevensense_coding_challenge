#!/bin/bash
set -e

echo "Building ROS 2 Humble Docker image..."

docker build --progress=plain -t ros2_humble -f Dockerfile .
