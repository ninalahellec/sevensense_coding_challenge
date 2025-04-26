# Base image: Ubuntu 22.04
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Install essential tools and set up locales
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    software-properties-common && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8

# Add ROS 2 GPG key and Humble repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && \
    echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble and simulation dependencies
RUN apt update && apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ackermann-steering-controller \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rqt-robot-steering \
    ros-humble-xacro \
    gazebo \
    libgazebo-dev && \
    rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup automatically in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# Install ROS 2 build tools
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
RUN rosdep init && rosdep update

# Create a non-root user matching host (change these if needed)
ARG USERNAME=nina
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid $USER_GID $USERNAME && \
    useradd --uid $USER_UID --gid $USER_GID -m $USERNAME && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Switch to non-root user
USER $USERNAME
WORKDIR /home/$USERNAME

# Create workspace
RUN mkdir -p /home/$USERNAME/ros2_ws/src

# Default working directory
WORKDIR /home/$USERNAME/ros2_ws
