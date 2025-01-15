# MUR ROS Package

![MUR Logo](path_to_logo_image) <!-- Replace with the actual path to your logo image -->

## Table of Contents

- [MUR ROS Package](#mur-ros-package)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [Usage](#usage)
    - [Launching the Entire System](#launching-the-entire-system)
  - [Configuration](#configuration)
    - [Configuration Files](#configuration-files)
  - [Dependencies](#dependencies)
    - [ROS Packages](#ros-packages)
    - [Python Libraries](#python-libraries)
  - [Customization](#customization)
    - [Including Additional Modules](#including-additional-modules)
    - [Adjusting Control Parameters](#adjusting-control-parameters)
    - [Changing Operational Environment](#changing-operational-environment)
  - [License](#license)

## Overview

The **MUR ROS Package** serves as the central hub for controlling and managing the **Miniature Underwater Robot (MUR)** system. It orchestrates the initialization and coordination of various sub-packages, including sensor management, modeling, and control systems. By utilizing a single launch file, users can effortlessly start all necessary components, ensuring seamless and simplified operation of the MUR.

## Features

- **Unified Launch System**: Start all essential MUR components with a single command using the `mur.launch` file.
- **Modular Architecture**: Integrates multiple sub-packages (`mur_sensors`, `mur_model`, `mur_control`) for comprehensive robot management.
- **Configurable Environments**: Easily switch between different operational environments (e.g., freshwater, seawater) through configuration files.
- **Scalable Design**: Supports the addition of new modules (e.g., `bluebuzz`) by simply uncommenting relevant lines in the launch file.
- **Emergency Handling**: Incorporates mechanisms for emergency stops and system resets to ensure safe operations.

## Usage

### Launching the Entire System

The `mur` package provides a comprehensive launch file that initializes all necessary sub-packages and configurations required for the MUR system. By launching `mur.launch`, you start the sensor modules, modeling components, and control systems in one streamlined process.

```bash
roslaunch mur mur.launch
```

This command performs the following actions:

1. **Loads Configuration Parameters**:
   - `global_config.yaml`: Defines module information and communication parameters.
   - `environment_info.yaml`: Specifies environmental parameters such as fluid type and density.

2. **Includes Sub-Launch Files**:
   - `mur_sensors.launch`: Initializes all sensor-related nodes.
   - `mur_model.launch`: Sets up the robot's modeling and simulation components.
   - `mur_control.launch`: Starts the control systems for maneuvering and stabilization.

   *Note: Additional modules like `bluebuzz` can be included by uncommenting the respective lines in the `mur.launch` file.*

## Configuration

### Configuration Files

The `mur` package utilizes YAML configuration files to manage various parameters essential for the robot's operation. These files are located in the `config/` directory.

- **.gitignore**: Specifies files and directories to be ignored by Git.
- **CMakeLists.txt**: Defines the build configuration for the package.
- **config/**: Contains YAML configuration files for environmental and module parameters.
  - `environment_info.yaml`: Defines environmental parameters like fluid type and density.
  - `global_config.yaml`: Specifies module information and communication settings.
- **launch/**: Holds launch files to initialize ROS nodes and configurations.
  - `mur.launch`: Main launch file that starts all necessary sub-packages and loads configurations.
- **LICENSE**: Contains the licensing information for the package.
- **msg/**: Defines custom ROS message types.
  - `DepthSensor.msg`: Custom message type for depth sensor data.
- **package.xml**: Describes the package and its dependencies.
- **README.md**: Documentation file for the package (this file).

## Dependencies

The `mur` package relies on several ROS packages and Python libraries to function correctly. Ensure that all dependencies are installed before running the control scripts.

### ROS Packages

- `rospy`: Python client library for ROS.
- `tf2_ros`: TF2 library for ROS transformations.
- `geometry_msgs`: Standard ROS messages for geometric data.
- `std_msgs`: Standard ROS messages.
- `mur_sensors`: Custom package for sensor management.
- `mur_model`: Custom package for robot modeling and simulation.
- `mur_control`: Custom package for control systems.
- `bluebuzz`: (Optional) Additional module for specialized functionalities.

### Python Libraries

- `pygame`: Library for creating multimedia applications (used here for capturing keyboard inputs).
- `simple_pid`: PID controller library.
- `tf`: ROS package for handling transformations.
- `numpy`: Fundamental package for scientific computing with Python.
- `math`: Standard Python library for mathematical operations.
- `time`: Time-related functions.

*Install Python dependencies using `pip` or your preferred package manager:*

```bash
pip install rospy pygame simple-pid tf2_ros numpy
```

*Note: Some dependencies may require ROS-specific installation steps.*

## Customization

### Including Additional Modules

The `mur.launch` file is designed to be easily customizable. To include additional modules, simply uncomment the relevant lines in the launch file.

```xml
<launch>
  <rosparam command="load" file="$(find mur)/config/global_config.yaml" />
  <rosparam command="load" file="$(find mur)/config/environment_info.yaml" />
  <!-- Launch nodes or other launch files -->
  <include file="$(find mur_sensors)/launch/mur_sensors.launch"/>
  <include file="$(find mur_model)/launch/mur_model.launch"/>
  <include file="$(find mur_control)/launch/mur_control.launch"/>
</launch>
```

### Adjusting Control Parameters

Modify the `control_info.yaml` file to adjust PID gains, control loop rates, and control modes for each degree of freedom (DOF). This allows for fine-tuning the robot's responsiveness and stability.

### Changing Operational Environment

Update the `environment_info.yaml` file to switch between different operational environments (e.g., freshwater, seawater) by selecting the appropriate fluid type and density.

## License

This project is licensed under the [MIT License](LICENSE).
