FROM mehmet/ros1-base:latest

WORKDIR /root/catkin_ws/src
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    git clone https://github.com/ori-drs/allan_variance_ros.git"

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
WORKDIR /root/catkin_ws
ENTRYPOINT ["/bin/bash"]
