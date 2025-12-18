# ROS Docker Ecosystem 

A modular Docker collection designed for seamless robotics data engineering. This toolkit handles everything from raw KAIST dataset extraction to modern ROS 2 Humble development.

## Project Structure
```
docker-containers/
├── convert_bags/      # Python 3.11 + ROS Noetic (for rosbags conversion)
├── kaist2bag_ws/      # ROS Noetic + KAIST2BAG parser
└── ros2_docker/       # ROS 2 Humble Desktop (Development Env)
```

## Prerequisites

    Docker and Docker Compose installed.

    (Optional) NVIDIA Container Toolkit for GPU acceleration in ROS 2.
### Removing old Docker versions (if any)
  ```bash
sudo apt remove -y docker docker-engine docker.io containerd runc
```
### Update Package List
```bash
sudo apt update
```

### Installation of Necessary Packages
```bash
sudo apt install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
```
### Adding Docker's Official GPG Key
```bash
sudo mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | \
sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
```
### Adding a Docker Repository to the System
```bash
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

### Update Package List
```bash
sudo apt update
```

### Docker Engine Installation
```bash
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### How to use Docker without Sudo.
```bash
sudo usermod -aG docker $USER
newgrp docker
reboot
```
## Containers Overview
### 1. KAIST2BAG (kaist2bag_ws)

Purpose: Converts the raw KAIST Urban Data Set into standard ROS 1 .bag files.

    Base Image: ubuntu:20.04 (ROS Noetic installed manually).

    Key Features: Includes irp_sen_msg and kaist2bag packages.

    Volumes: * Input: /your/dataset/file/path (Read-only)

    Output: /your/bag/file/path

### 2. ROS Bags Converter (convert_bags)

Purpose: A specialized environment to migrate or modify ROS bags using the rosbags Python library.

    Base Image: ros:noetic-ros-base.

    Key Features: Custom compiled Python 3.11.9 (using altinstall to avoid breaking system ROS tools) and the rosbags high-performance migration library.

    Use Case: Converting .bag (ROS 1) to .mcap or .db3 (ROS 2).

### 3. ROS 2 Humble Dev (ros2_docker)

Purpose: A full desktop-class ROS 2 development environment.

    Base Image: osrf/ros:humble-desktop.

    Key Features: * Non-root user (ros) mapping to host UID 1000 for permission harmony.

        GUI support (X11 forwarding).

        NVIDIA GPU support (requires nvidia-container-toolkit).

    Tools: colcon, terminator, git.



## Configuration Details
### Volume Mapping

Ensure your external drives or data paths match the volumes section in each docker-compose.yml:

    Default Data Path: /media/mehmet/Elements/...

    If your drive name is different, update the left side of the : in the .yml files.

### Aligning KAIST2BAG Paths

The kaist2bag tool relies on a config.yaml file to locate your data. By default, the source code points to local paths (e.g., /home/tao/...). You must update this inside the container to match the Docker volume mount points.

1. Locate the config file:
```Bash
docker exec -it kaist2bag bash
nano /root/ws/src/kaist2bag/config/config.yaml
```
2. Update the paths as follows:
   
| Parameter | Default (Incorrect) | For Docker (Correct) | 
|---|---|---|
| dataset | /home/tao/sda/dataset/... | /dataset |   
| save_to | /home/tao/sda/dataset/.../bag | /output | 


### Data Mapping Reference

To ensure your files are saved correctly to your external drive, verify your docker-compose.yml volume mappings:
```YAML

volumes:
  - /media/mehmet/Elements/urban39:/dataset:ro  # <--- Source Raw Data
  - /media/mehmet/Elements/bag:/output          # <--- Target .bag Destination
```

### GUI Forwarding (ROS 2)

To see Rviz or Gazebo from the ROS 2 container, run this on your host machine before starting the container:
Bash

xhost +local:docker

🛠 Troubleshooting

    Permissions: The ROS 2 container uses UID 1000. If your host user is not 1000, change the args in ros2_docker/docker-compose.yml.

    Python Versions: In the convert_bags container, always use python3.11 to access the modern libraries; python3 refers to the system's ROS Noetic default (3.8).

Would you like me to create a specific entrypoint.sh script for any of these containers to automate the sourcing of workspaces?
