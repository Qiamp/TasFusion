# UWB/GPS-IMU Fusion System for Multi-Sensor Navigation

## Overview
**UWB/GPS-IMU Fusion System** is a comprehensive ROS package for tightly-coupled sensor fusion that integrates UWB/GPS positioning with IMU data using factor graph optimization. Developed at The Hong Kong Polytechnic University (PolyU), this system provides both high-frequency IMU-rate pose estimates and optimized trajectory estimates using sliding window optimization with marginalization. The package is designed for robotics applications requiring robust localization in both indoor (UWB) and outdoor (GPS) environments.

- [Test Dataset](https://www.dropbox.com/scl/fi/3kk7myf0k0mucuvynifst/wxri_round.bag?rlkey=ybkmuouk8rozjjf1laqgtnsst&dl=0)
- [Tutorial in PDF](https://github.com/weisongwen/GNSSBoard/blob/main/support_files/toySLAM_Tutorial.pdf)

## Features
- **Dual-Mode Positioning**: Seamlessly switch between UWB (indoor) and GPS/GNSS (outdoor) positioning sources
- **High-Rate State Estimation**: Publishes IMU-propagated poses at full IMU frequency (up to 400Hz)
- **Factor Graph Optimization**: Employs Ceres Solver for robust sliding window optimization
- **IMU Preintegration**: Efficient computation of IMU factors between keyframes using RK4 integration
- **Marginalization**: Implements Schur complement-based marginalization for computational efficiency
- **Multi-GNSS Support**: Compatible with INSPVAX, GnssPVTSolnMsg, and Odometry message formats
- **Online Bias Estimation**: Real-time estimation and correction of IMU accelerometer and gyroscope biases
- **Comprehensive Visualization**: Real-time trajectory and error visualization in RViz
- **CSV Data Logging**: Automatic saving of trajectory and detailed state information to CSV files

## System Architecture

### Core Components
1. **High-Frequency IMU Propagation**: Continuous state propagation at IMU rate for real-time applications
2. **Keyframe-Based Optimization**: Sliding window factor graph optimization at 10Hz
3. **Measurement Processing**: Unified interface for UWB/GPS measurements with consistency checking
4. **Marginalization Module**: Efficient information retention from marginalized states

### Key Nodes
- **uwb_imu_fusion_node**: Main fusion node handling both UWB and GPS modes
- **State Propagator**: Real-time IMU integration with bias correction
- **Factor Graph Optimizer**: Ceres-based optimization with multiple factor types
- **Visualization Publisher**: Comprehensive trajectory and error visualization

### Factor Graph Structure
- **IMU Factors**: Connect consecutive keyframes via preintegrated measurements
- **Position Factors**: GPS/UWB measurements with full covariance matrices
- **Velocity Factors**: GPS velocity measurements (when available)
- **Marginalization Prior**: Information retention from marginalized states

### Optimization Strategy
- **Sliding Window**: Maintains fixed-size state window (default: 10 keyframes)
- **Schur Complement**: Efficient marginalization of oldest states
- **Adaptive Constraints**: Dynamic weighting based on measurement consistency

## Performance
- **IMU Processing**: Up to 400Hz real-time propagation
- **Optimization**: 10Hz sliding window optimization
- **Memory**: ~15 seconds of IMU buffer maintained
- **CPU Usage**: Optimized for embedded systems (tested on Intel i7)

## Installation

### Dependencies
- **[ROS](https://www.ros.org/)** (Noetic recommended)
- **Eigen3** (Linear algebra library)
- **Ceres Solver 2.0+** (Non-linear optimization)
- **PCL** (Point cloud processing)
- **novatel_msgs** (For INSPVAX GPS messages)
- **gnss_comm** (For GNSS PVT solutions)

### Build Instructions

```bash
# Clone the repository
cd ~/catkin_ws/src
git clone https://github.com/weisongwen/GNSSBoard.git

# Install dependencies
sudo apt-get install ros-noetic-navigation ros-noetic-tf2-ros
sudo apt-get install libeigen3-dev libceres-dev

# Build the package
cd ~/catkin_ws
catkin_make

# Source the workspace
source devel/setup.bash
```
## Usage

### Quick Start - GPS Mode

```bash
# Launch with GPS/IMU fusion
roslaunch toyslam batch_board.launch

# Play test dataset
rosbag play wxri_round.bag
```

## Docker Development Environment (Recommended)

This section describes how to use the containerized Docker environment for consistent and reproducible builds.

### Prerequisites
- **Docker Engine**: Must be installed on your Linux system
- **Git**: Required to clone the repository

### Docker Quick Start Guide

#### Step 1: Create Workspace and Clone Repository
```bash
# Create the workspace directories
mkdir -p ~/catkin_ws/src

# Navigate into the src directory
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/weisongwen/GNSSBoard.git .

# Navigate back to the workspace root
cd ~/catkin_ws
```

#### Step 2: Build Docker Image
Build the development environment Docker image:
```bash
# Run from the catkin_ws/ directory
docker build --no-cache -f src/.docker/Dockerfile -t gnssboard-dev .
```

#### Step 3: Set Script Permissions
Make the startup script executable:
```bash
# Run from the catkin_ws/ directory
chmod +x src/.docker/start_docker.sh
```

#### Step 4: Run Development Environment
Start the container with the development environment:
```bash
# Run from the catkin_ws/ directory
./src/.docker/start_docker.sh
```

### Development Workflow
- Your local `catkin_ws/` folder is mounted inside the container
- Edit code on your host machine using your preferred editor
- Compile inside the container using `catkin_make`
- Run ROS nodes (e.g., `roslaunch`, `rosrun`) directly from the container's terminal (Hints: We use the is_board.launch file for testing)
- All terminals inside the container are automatically sourced with the ROS environment
- Create a data folder in your local computer, and put rosbags inside, and then you can use `rosbag play` to test the code

### Container Features
- Pre-installed ROS Noetic and all dependencies
- Persistent workspace mounting
- Automatic environment sourcing
- GPU support (if available)
- Shared network with host

## CSV Data Logging

The system automatically saves trajectory and state data to CSV files for post-processing and analysis.

### Output Files

Two types of CSV files are generated:

**File Path :** $HOME/Documents/is_board_data/csv_logs

1. **Trajectory CSV** (`gnss_imu_trajectory_YYYYMMDD_HHMMSS.csv`)
   - Contains basic trajectory information
   - Updated at each optimization step
   
2. **State CSV** (`gnss_imu_state_YYYYMMDD_HHMMSS.csv`)
   - Contains detailed system state and diagnostic information
   - Includes IMU biases, optimization statistics, and performance metrics

### Configuration Parameters

Control CSV logging through launch file parameters:

```xml
<!-- CSV output parameters -->
<param name="save_trajectory_csv" value="true"/>
<param name="save_state_csv" value="true"/>
```

### File Management

- Files are automatically timestamped to prevent overwrites
- CSV headers are written immediately upon system initialization
- Data is flushed to disk every 10 optimization cycles for data safety
- Container restart creates new files with fresh timestamps

## Acknowledgments

This work was developed at The Hong Kong Polytechnic University (PolyU) with support from:

- **Department of Aeronautical and Aviation Engineering**, The Hong Kong Polytechnic University
- **TAS Lab** (Trustworthy Autonomous Systems Laboratory) at PolyU
- Built on top of the [ROS (Robot Operating System)](https://www.ros.org/) ecosystem - an open-source set of software libraries and tools for building robot applications. 
- Optimization powered by **Google's Ceres Solver** for non-linear least squares problems
- The ROS community and Open Robotics for maintaining the robust robotics middleware framework

Special thanks to the contributors and maintainers of the open-source libraries that make this project possible.
