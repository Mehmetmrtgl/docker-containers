FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------
# Install base tools
# -----------------------------
RUN apt-get update && apt-get install -y \
    wget curl git nano build-essential \
    python3 python3-pip \
    apt-utils lsb-release gnupg2

# -----------------------------
# Install ROS 1 Noetic
# -----------------------------
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop-full \
    python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool \
    python3-catkin-tools

# Initialize rosdep
RUN rosdep init && rosdep update

# -----------------------------
# Create catkin workspace
# -----------------------------
RUN mkdir -p /root/ws/src
WORKDIR /root/ws/

# -----------------------------
# Clone KAIST2BAG dependencies
# -----------------------------
RUN cd src && \
    git clone https://github.com/RPM-Robotics-Lab/irp_sen_msg.git && \
    git clone https://github.com/tsyxyz/kaist2bag.git

# -----------------------------
# Build workspace
# -----------------------------
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

# Source workspace at shell startup
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source /root/ws/devel/setup.bash" >> ~/.bashrc

WORKDIR /root/ws/

