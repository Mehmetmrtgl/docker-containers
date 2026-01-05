# ROS Docker Workspace

This repository hosts a modular ROS 1 development ecosystem. It uses a unified Docker structure to manage multiple robotics tools (Calibration, IMU Analysis, Dataset Conversion) using a single shared base image.

Each tool resides in its own directory with specific documentation, keeping the project organized and easy to maintain.

##  Repository Structure

The workspace is organized as a flat hierarchy managed by a central `docker-compose.yml`:

```text
.
├── allan_variance_ros/    # IMU Noise Analysis modules
├── kaist2bag_ws/          # Dataset conversion tools
├── kalibr/                # Camera/IMU calibration tools
├── docker-compose.yml     # Central orchestration for all services
├── ros1_base.Dockerfile   # Shared ROS Noetic Base Image
└── README.md              # Global documentation

```

##  Architecture & Build

This ecosystem relies on a **Single Base Image Strategy**. The `ros1_base.Dockerfile` in the root directory serves as the foundation for all services. This ensures that all tools use the exact same OS and ROS versions without downloading dependencies multiple times.

* **Consistency:** All tools use the exact same OS and ROS versions.
* **Efficiency:** Dependencies are downloaded and cached only once.

### Building the Ecosystem

#### Option 1: Build Everything (Recommended for first setup) This builds the base image and all registered services.

```bash
docker compose build

```

#### Option 2: Build Individual Services If you only want to update or rebuild a specific tool:


#### Build only the Calibration tool

```bash
docker compose build kalibr
```

#### Build only the IMU Analysis tool
```bash

docker compose build allan_variance_ros
```
#### Build only the Dataset Converter
```bash
docker compose build kaist2bag_ws
```
## Services (Modules)

For detailed usage, scripts, and configuration for each tool, please refer to the **README.md** files inside their respective directories:

* **[📂 kalibr/]([https://www.google.com/search?q=./kalibr/](https://github.com/ethz-asl/kalibr))**
* Automated Camera & IMU calibration (Static/Dynamic) workflows.


* **[📂 allan_variance_ros/]([https://www.google.com/search?q=./allan_variance_ros/](https://github.com/ori-drs/allan_variance_ros))**
* IMU noise characterization and parameter extraction.


* **[📂 kaist2bag_ws/]([https://www.google.com/search?q=./kaist2bag_ws/](https://github.com/rpng/kaist2bag))**
* Tools for parsing and converting raw KAIST Urban datasets.



## Global Configuration

### Data Volumes

By default, the `docker-compose.yml` mounts the `./data` directory to all containers to share bags and results easily:

```yaml
volumes:
  - ./data:/root/catkin_ws/data

```

### GUI Support

All containers are configured for X11 forwarding. To enable visualization (Rviz/Plots) on the host:

```bash
xhost +local:docker

```
