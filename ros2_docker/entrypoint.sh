#!/bin/bash
set -e

# ROS 2 ortamını yükle
source "/opt/ros/humble/setup.bash"

# Eğer workspace içinde build alınmışsa onu da yükle
if [ -f "/home/ros/ros2_ws/install/setup.bash" ]; then
    source "/home/ros/ros2_ws/install/setup.bash"
fi

exec "$@"
