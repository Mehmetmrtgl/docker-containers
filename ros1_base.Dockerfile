FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    apt-utils lsb-release gnupg2 \
    git wget nano vim autoconf automake build-essential curl \
    python3-dev python3-pip python3-tk python3-scipy python3-matplotlib python3-wxgtk4.0\
    python3-catkin-tools python3-osrf-pycommon python3-rosdep \
    python3-rosinstall-generator \
    libeigen3-dev libboost-all-dev libsuitesparse-dev \
    libopencv-dev libpoco-dev libtbb-dev libblas-dev liblapack-dev libv4l-dev ros-noetic-cv-bridge \
    doxygen psmisc bash-completion \
    && rm -rf /var/lib/apt/lists/*


RUN pip3 install --upgrade pip && \
    pip3 install --upgrade scipy
    pip3 install numpy==1.24.4 rosbags ipython igraph pyx \
    && pip3 install rosbags

RUN if [ ! -d /etc/ros/rosdep/sources.list.d ]; then rosdep init; fi && rosdep update
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

WORKDIR /root
