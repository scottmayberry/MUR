# MUR ROS Packages

![MUR Logo](path_to_logo_image) <!-- Replace with the actual path to your logo image -->

## Table of Contents

- [MUR ROS Packages](#mur-ros-packages)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [Usage](#usage)
    - [Launching the System](#launching-the-system)
  - [Configuration](#configuration)
    - [Configuration Files](#configuration-files)
  - [Packages](#packages)
    - [mur](#mur)
    - [mur\_control](#mur_control)
    - [mur\_model](#mur_model)
    - [mur\_sensors](#mur_sensors)
  - [Dependencies](#dependencies)
    - [ROS Packages](#ros-packages)
    - [Python Libraries](#python-libraries)
  - [Customization](#customization)
    - [Including Additional Modules](#including-additional-modules)
    - [Adjusting Control Parameters](#adjusting-control-parameters)
    - [Changing Operational Environment](#changing-operational-environment)
  - [License](#license)

## Overview

The **MUR ROS Packages** form a comprehensive suite designed to control and manage the **Miniature Underwater Robot (MUR)**. These interconnected packages work collaboratively to handle various aspects of the robot's operation, including control mechanisms, modeling, sensor management, and overall system orchestration. By leveraging the modular architecture of ROS (Robot Operating System), the MUR system ensures scalability, maintainability, and efficient performance in underwater environments.

## Features

- **Unified Control System**: Integrates manual and PID-based control mechanisms for precise maneuvering.
- **Modular Architecture**: Divided into specialized packages (`mur`, `mur_control`, `mur_model`, `mur_sensors`) for scalability and maintainability.
- **Comprehensive Sensor Management**: Handles various sensors essential for underwater navigation and operation.
- **Environment Configurability**: Easily switch between different operational environments (e.g., freshwater, seawater) through configuration files.
- **Emergency Handling**: Implements emergency stop and reset functionalities to ensure safe operations.
- **Simplified Launch Process**: Utilize a single launch file to initialize all necessary components, streamlining deployment.


## Usage

### Launching the System

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

The `mur` and `mur_control` packages utilize YAML configuration files to manage various parameters essential for the robot's operation. These files are located in the `config/` directories of their respective packages.

- **global_config.yaml** (`mur/config/global_config.yaml`): Specifies module information, communication settings, and other system-wide configurations.
- **environment_info.yaml** (`mur/config/environment_info.yaml`): Defines environmental parameters such as the type and density of the operating fluid and the gravitational constant.
- **control_info.yaml** (`mur_control/config/control_info.yaml`): Contains PID gains, control loop rates, and control modes for each degree of freedom (DOF).

*Ensure that these configuration files are properly set up before launching the system to guarantee optimal performance.*

## Packages

### mur

The `mur` package serves as the central orchestrator for the MUR system. It manages the overall system configuration, initializes all sub-packages, and ensures cohesive operation.

- **Key Components**:
  - **Launch File**: `mur.launch` includes and initializes all necessary sub-packages.
  - **Configuration Files**: `global_config.yaml` and `environment_info.yaml` define system-wide settings and environmental parameters.
  - **Custom Messages**: `DepthSensor.msg` defines a custom message type for depth sensor data.

- **Usage**:
  
  Launch the entire MUR system using the provided launch file:
  
  ```bash
  roslaunch mur mur.launch
  ```

### mur_control

The `mur_control` package handles the control mechanisms of the MUR, including both manual and PID-based controls. It processes input commands, computes control efforts, and publishes wrench commands to the thrusters.

- **Key Components**:
  - **Launch File**: `mur_control.launch` initializes all control-related nodes.
  - **Configuration File**: `control_info.yaml` defines PID gains, control modes, and control loop rates.
  - **Scripts**:
    - `blended_manual_and_pid_setpoint_controller.py`: Combines manual inputs with PID-based control.
    - `manual_with_PID_setpoint_generator.py`: Generates pose setpoints based on manual inputs.
    - `ros_manual_thruster_driver.py`: Allows manual control of thrusters via keyboard.
    - `ros_pid_setpoint_control.py`: Implements PID-based stabilization control.
    - `ros_pose_setpoint_generator.py`: Generates and publishes pose setpoints.

- **Usage**:
  
  The `mur_control` package is automatically launched via the main `mur.launch` file. Ensure that the `control_info.yaml` is properly configured before launching.

### mur_model

The `mur_model` package is responsible for the robot's modeling and simulation components. It creates a realistic model of the MUR, enabling simulation and testing of control strategies.

- **Key Components**:
  - **Launch File**: `mur_model.launch` initializes all modeling and simulation nodes.
  - **Scripts and Models**: Contains scripts for simulation and 3D models representing the MUR.

- **Usage**:
  
  The `mur_model` package is automatically launched via the main `mur.launch` file. Ensure that all necessary models and simulation parameters are correctly set in the configuration files.

### mur_sensors

The `mur_sensors` package manages all sensor-related functionalities of the MUR, including data acquisition, processing, and communication.

- **Key Components**:
  - **Launch File**: `mur_sensors.launch` initializes all sensor nodes.
  - **Sensors**: Includes drivers and interfaces for various sensors (e.g., depth sensors, IMUs, cameras).

- **Usage**:
  
  The `mur_sensors` package is automatically launched via the main `mur.launch` file. Ensure that all sensor parameters are correctly configured in the `global_config.yaml` before launching.

## Dependencies

The `mur` and `mur_control` packages rely on several ROS packages and Python libraries to function correctly. Ensure that all dependencies are installed before running the control scripts.

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

The `mur.launch` file is designed to be easily customizable. To include additional modules such as `bluebuzz`, simply uncomment the relevant lines in the launch file.

```xml
<launch>
  <rosparam command="load" file="$(find mur)/config/global_config.yaml" />
  <rosparam command="load" file="$(find mur)/config/environment_info.yaml" />
  <!-- Launch nodes or other launch files -->
  <include file="$(find mur_sensors)/launch/mur_sensors.launch"/>
  <!-- <include file="$(find bluebuzz)/launch/bluebuzz.launch"/> -->
  <include file="$(find mur_model)/launch/mur_model.launch"/>
  <include file="$(find mur_control)/launch/mur_control.launch"/>
</launch>
```

*To include the `bluebuzz` module, remove the comment tags:*

```xml
<include file="$(find bluebuzz)/launch/bluebuzz.launch"/>
```

### Adjusting Control Parameters

Modify the `control_info.yaml` file within the `mur_control` package to adjust PID gains, control loop rates, and control modes for each degree of freedom (DOF). This allows for fine-tuning the robot's responsiveness and stability.

### Changing Operational Environment

Update the `environment_info.yaml` file within the `mur` package to switch between different operational environments (e.g., freshwater, seawater) by selecting the appropriate fluid type and density.

## License

This project is licensed under the [MIT License](mur/LICENSE).
