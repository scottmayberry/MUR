# MUR Model ROS Package

![MUR Logo](path_to_logo_image) <!-- Replace with the actual path to your logo image -->

## Table of Contents

- [MUR Model ROS Package](#mur-model-ros-package)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
    - [Key Directories and Files](#key-directories-and-files)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
  - [Dependencies](#dependencies)
  - [Configuration](#configuration)
    - [ArUco Marker Configuration](#aruco-marker-configuration)
    - [Model Information](#model-information)
  - [Usage](#usage)
    - [Launching the MUR Model](#launching-the-mur-model)
    - [Additional Launch Options](#additional-launch-options)
  - [Components](#components)
    - [Camera](#camera)
      - [Scripts](#scripts)
      - [Configuration](#configuration-1)
    - [Localization](#localization)
      - [Scripts](#scripts-1)
      - [Configuration](#configuration-2)
    - [Thruster](#thruster)
      - [Scripts](#scripts-2)
      - [Configuration](#configuration-3)
    - [Static TF2 Broadcaster](#static-tf2-broadcaster)
    - [Testing](#testing)
      - [Scripts](#scripts-3)
      - [Usage](#usage-1)
  - [License](#license)

## Overview

The **MUR Model** ROS package provides a comprehensive simulation and control framework for the **Miniature Underwater Robot (MUR)**. It encompasses various modules responsible for camera management, localization, thruster control, and transformation broadcasting. This package facilitates the integration of sensors, actuators, and control algorithms to enable autonomous operation and precise navigation of the MUR in underwater environments.

## Features

- **Camera Management**: Handles multiple camera streams, converts video feeds to ROS Image messages, and publishes them for processing and visualization.
- **Localization**: Processes IMU and pressure sensor data to determine the robot's orientation and depth, ensuring accurate spatial awareness.
- **Thruster Control**: Manages thruster configurations, command conversions, and visualization tools to control the robot's movement.
- **Transformation Broadcasting**: Maintains and broadcasts static and dynamic TF2 frames to accurately represent the robot's structure and movements.
- **Testing Tools**: Includes scripts for testing ESC configurations, visualizing thruster values, and validating thruster performance.

### Key Directories and Files

- **config/**: Contains YAML configuration files for ArUco markers and model information.
- **launch/**: Includes ROS launch files to initiate the various nodes and services.
- **src/**: Houses all the source scripts organized into subdirectories based on functionality.
  - **camera/**: Scripts related to camera stream management and ROS integration.
  - **localization/**: Scripts for processing sensor data to determine the robot's position and orientation.
  - **thruster/**: Scripts for controlling and visualizing thruster operations.
  - **test/**: Testing scripts for validating thruster configurations and behaviors.
  - **static_tf2_broadcaster.py**: Script for broadcasting static TF2 frames.

## Installation

### Prerequisites

- **ROS**: Ensure that you have a compatible version of ROS installed (e.g., ROS Noetic, ROS Melodic).
- **Python Dependencies**: The package relies on several Python libraries. Install them using `pip` or your preferred package manager.

## Dependencies

- **ROS Packages**:
  - `rospy`
  - `sensor_msgs`
  - `std_msgs`
  - `geometry_msgs`
  - `tf2_ros`
  - `cv_bridge`
- **Python Libraries**:
  - `opencv-python`
  - `numpy`
  - `json`
  - `socket`
  - `threading`
  - `queue`

*Ensure all dependencies are installed before running the package.*

## Configuration

### ArUco Marker Configuration

The `aruco_info.yaml` file defines the ArUco dictionary and the specific markers used for control identification and positioning.

- **dictionary**: Specifies which ArUco dictionary to use (e.g., `DICT_4X4_50`).
- **tags**: Lists each ArUco marker with its unique ID, position, and orientation relative to the robot's frames.

*Example Configuration:*

```yaml
aruco_info:
  dictionary: DICT_4X4_50
  
  tags:
    - id: 1
      pos: [0.5, 0.0, 0.2]
      orientation: [0.0, 0.0, 0.0]
    - id: 2
      pos: [-0.5, 0.0, 0.2]
      orientation: [0.0, 0.0, 3.14159]
```

### Model Information

The `model_info.yaml` file contains detailed information about the robot's structure, physical properties, control inputs, thruster configurations, ESC settings, and camera setups.

*Refer to the [model_info.yaml](config/model_info.yaml) for detailed comments and explanations.*

## Usage

### Launching the MUR Model

To start the MUR Model, use the provided ROS launch file:

```bash
roslaunch mur_model mur_model.launch
```

This launch file initializes all necessary nodes, including camera streams, localization modules, thruster controllers, and TF broadcasters.

### Additional Launch Options

You can customize the launch behavior by modifying the launch file or passing parameters directly:

```bash
roslaunch mur_model mur_model.launch camera:=true localization:=true thrusters:=true
```

*Ensure that all required parameters are correctly set in the configuration files before launching.*

## Components

### Camera

#### Scripts

- **camera_boot_server_udp.py**: Handles the initialization of camera streams by sending UDP boot requests to camera modules. It constructs JSON-formatted messages based on camera configurations and transmits them at specified intervals.
  
- **camera_udp_to_ros.py**: Converts incoming UDP camera streams into ROS Image messages. It captures video frames from camera streams, applies any necessary image transformations (e.g., flipping), and publishes the frames to designated ROS topics.

#### Configuration

- **Config File**: `config/aruco_info.yaml` and `config/model_info.yaml`
- **Key Parameters**:
  - Stream URLs
  - Frame rates (FPS)
  - Image resolutions
  - Flip settings for image orientation

### Localization

#### Scripts

- **imu_baselink_updater.py**: Processes IMU data to update the robot's orientation in the TF tree. It listens to IMU sensor topics, computes the corrected orientation, and broadcasts the updated transform to maintain accurate spatial relationships.
  
- **pressure_to_depth.py**: Converts pressure sensor readings into depth estimations. It subscribes to pressure topics, calibrates zero-depth pressure, calculates depth based on pressure changes, and updates the robot's transform accordingly.

#### Configuration

- **Config File**: `config/model_info.yaml`
- **Key Parameters**:
  - Sensor configurations (IMU, pressure sensors)
  - Environmental parameters (fluid density, gravity)
  - TF frame relationships

### Thruster

#### Scripts

- **ros_thruster_command_conversion.py**: Converts high-level thruster commands into ESC-specific commands based on the thruster configurations.
  
- **ros_thruster_microsecond_udp.py**: Sends thruster commands over UDP with microsecond precision for fine control.
  
- **ros_thruster_to_id_microsecond_udp.py**: Maps ROS thruster commands to specific thruster IDs and sends them via UDP.
  
- **ros_thruster_wrench_comparison_visualizer.py**: Visualizes the difference between commanded and actual thruster wrenches to assess performance.
  
- **ros_thruster_wrench_generator.py**: Generates wrench (force and torque) commands for thrusters based on control inputs.

#### Configuration

- **Config File**: `config/model_info.yaml`
- **Key Parameters**:
  - Thruster positions and orientations
  - ESC configurations
  - Board plug IDs
  - Thrust force formulas

### Static TF2 Broadcaster

- **static_tf2_broadcaster.py**: Broadcasts static TF2 frames defined in `model_info.yaml` to maintain fixed spatial relationships between different parts of the robot.

### Testing

#### Scripts

- **esc_loader_test.py**: Tests the loading and configuration of Electronic Speed Controllers (ESCs).
  
- **thruster_value_plot.py**: Plots thruster values over time for analysis and debugging.
  
- **thruster_visualizer_test.py**: Visualizes thruster operations to ensure correct behavior and responses.

#### Usage

Run test scripts individually to validate specific components:

```bash
rosrun mur_model test/esc_loader_test.py
rosrun mur_model test/thruster_value_plot.py
rosrun mur_model test/thruster_visualizer_test.py
```

*Ensure that the ROS master and necessary nodes are running before executing test scripts.*

## License

This project is licensed under the [MIT License](LICENSE).
