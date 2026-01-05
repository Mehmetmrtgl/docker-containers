FROM mehmet/ros1-base:latest


WORKDIR /root/catkin_ws/src

RUN git clone https://github.com/ethz-asl/kalibr.git

WORKDIR /root/catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
    catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    catkin build -j$(nproc)"

RUN echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc

ENV KALIBR_MANUAL_FOCAL_LENGTH_INIT=1
WORKDIR /root/catkin_ws
ENTRYPOINT ["/bin/bash"]
