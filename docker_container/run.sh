#!/bin/bash
set -e

IMAGE_NAME=ros2_humble
CONTAINER_NAME=ros2_humble_dev

docker run -it \
    --name $CONTAINER_NAME \
    --rm \
    --net=host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/Documents/sevensense/ros2_ws:/home/nina/ros2_ws/src \
    $IMAGE_NAME /bin/bash


