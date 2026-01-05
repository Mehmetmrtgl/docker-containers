WORKDIR /root/ws/



FROM mehmet/ros1-base:latest

WORKDIR /root/ws/src
RUN git clone https://github.com/RPM-Robotics-Lab/irp_sen_msg.git && \
    git clone https://github.com/tsyxyz/kaist2bag.git

WORKDIR /root/ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

RUN echo "source /root/ws/devel/setup.bash" >> ~/.bashrc
WORKDIR /root/ws
ENTRYPOINT ["/bin/bash"]
