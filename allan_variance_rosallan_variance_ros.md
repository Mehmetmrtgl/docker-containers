# Allan Variance ROS (IMU Noise Analysis)

This module automates the computation of stochastic noise parameters (Noise Density & Random Walk) for IMUs using the Allan Variance method. It includes a helper script to handle bag timestamp correction ("cooking"), analysis, and visualization in one go.
##  Internal Structure
```Plaintext

allan_variance_ros/
├── config/              # YAML configuration files
├── data/                # Data mount point (Output results go here)
├── script/
│   └── run_analysis.sh  # Automated analysis script
├── src/                 # Source code
├── Dockerfile
└── README.md
```
## Prerequisites

    Stationary Data: The IMU bag file must be recorded while the sensor is absolutely still (2-4 hours recommended).

    Input Location: Place your .bag files in the host's data/ros1bag/ directory.

## Build & Run
1. Build the Module

From the project root:

```Bash
docker compose build allan
```
2. Start the Container
   
```Bash
docker compose up -d allan
docker exec -it allan_variance_container bash
```
## Usage (Automated Script)

### 1. Check Configuration

Before running the analysis, check the config folder. It contains pre-defined examples for common sensors.
```Bash

cd /root/catkin_ws/src/allan_variance_ros/config
ls
# alphasense.yaml   anymal_c.yaml   realsense_d425i.yaml   sim.yaml
# anymal_b.yaml     ouster.yaml     realsense_t265.yaml    simulation
```
You can either use one of these defaults or create your own (e.g., imu.yaml) with your specific IMU topic and rate.
### 2. Run the Analysis Script

Assuming you have static_data.bag in your data folder and you created (or chose) imu.yaml in the config folder:
```Bash
./script/run_analysis.sh <bag_filename> <config_filename>
```
```bash
cd /root/catkin_ws/src/allan_variance_ros
./script/run_analysis.sh static_data.bag imu.yaml
```
What the script does:

    Checks Roscore: Starts it in the background if not running.

    Cooks the Bag: Fixes timestamp issues (saves as cooked_filename.bag).

    Computes AV: Runs the Allan Variance node.

    Visualizes: Opens the Log-Log plot and saves results to CSV.

## Configuration (config/imu.yaml)

Ensure your configuration file matches your hardware settings:
```YAML

imu_topic: "/camera/imu"
imu_rate: 400
measure_rate: 100 # Rate to which imu data is subsampled
sequence_time: 10800 # 3 hours in seconds

```

## Outputs

After the script finishes, check the data/ folder for:

    allan_variance_results.csv: Contains the computed sigma values.

    Plots: Visual graphs of the deviation.

Interpretation:

    White Noise: Value at slope -0.5

    Random Walk: Value at slope +0.5

## Important Note on Permissions

If the script doesn't run, make it executable inside the container:
```Bash
chmod +x script/run_analysis.sh
```
