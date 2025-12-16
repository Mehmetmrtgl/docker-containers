#!/bin/bash
set -e

# ROS 1 Noetic ortamını yükle
source "/opt/ros/noetic/setup.bash"

# Komutu çalıştır
exec "$@"
