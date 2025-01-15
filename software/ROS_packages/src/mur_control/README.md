# MUR Control ROS Package

![MUR Logo](path_to_logo_image) <!-- Replace with the actual path to your logo image -->

## Table of Contents

- [MUR Control ROS Package](#mur-control-ros-package)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [Usage](#usage)
    - [Launching the Control System](#launching-the-control-system)
  - [Configuration](#configuration)
    - [control\_info.yaml](#control_infoyaml)
      - [Key Configuration Parameters](#key-configuration-parameters)
  - [Components](#components)
    - [Python Scripts](#python-scripts)
      - [blended\_manual\_and\_pid\_setpoint\_controller.py](#blended_manual_and_pid_setpoint_controllerpy)
      - [manual\_with\_PID\_setpoint\_generator.py](#manual_with_pid_setpoint_generatorpy)
      - [ros\_manual\_thruster\_driver.py](#ros_manual_thruster_driverpy)
      - [ros\_pid\_setpoint\_control.py](#ros_pid_setpoint_controlpy)
      - [ros\_pose\_setpoint\_generator.py](#ros_pose_setpoint_generatorpy)
  - [Dependencies](#dependencies)
  - [License](#license)

## Overview

The **MUR Control ROS Package** is a critical component of the **Miniature Underwater Robot (MUR)** ecosystem. It provides a suite of control tools and scripts designed to manage the robot's thrusters, stabilize its movement, and handle both manual and automated pose setpoints. This package leverages ROS (Robot Operating System) to facilitate seamless integration, communication, and control within the MUR system.

## Features

- **Blended Control**: Combines manual keyboard inputs with PID-based automated control for stabilization.
- **Pose Setpoint Generation**: Generates and publishes desired poses for autonomous navigation or testing.
- **Thruster Management**: Directly controls thrusters based on user inputs and automated commands.
- **Emergency Handling**: Implements emergency stop and reset functionalities for enhanced safety.
- **Configurable Parameters**: Easily adjustable control parameters through YAML configuration files.

## Usage

### Launching the Control System

The `mur_control` package provides a launch file to initialize all necessary nodes and configurations.

```bash
roslaunch mur_control mur_control.launch
```

This launch file initializes the control nodes, sets up the necessary parameters, and ensures that all components are ready for operation.

## Configuration

### control_info.yaml

The `control_info.yaml` file resides in the `config/` directory and contains all the configurable parameters for the control system. This includes PID gains, control loop rates, and control modes for each degree of freedom (DOF).

#### Key Configuration Parameters

- **manual_thruster_update_hz**: Determines how frequently (in Hertz) manual thruster commands are sent. A higher value allows for more responsive manual control.

- **stabilization**:
  - **rate**: Sets the frequency (in Hertz) at which the stabilization control loop operates.
  
  - **gains**: Defines the PID (Proportional, Integral, Derivative) gains for each degree of freedom (x, y, z, roll, pitch, yaw). These gains are crucial for tuning the robot's responsiveness and stability.
  
  - **control_mode**: Specifies whether each DOF is controlled manually or via PID controllers. Options are:
    - `manual`: User inputs directly control the DOF.
    - `pid`: Automated PID controllers manage the DOF based on setpoints.

## Components

### Python Scripts

The `src/` directory contains several Python scripts, each responsible for different aspects of the control system.

#### blended_manual_and_pid_setpoint_controller.py

**Purpose**: Implements a blended control strategy that combines manual keyboard inputs with PID-based automated control for managing the robot's pose.

**Key Functionalities**:

- Captures keyboard inputs to allow manual adjustments to the robot's position and orientation.
- Utilizes PID controllers to stabilize or guide the robot towards desired setpoints.
- Publishes combined wrench commands to thrusters for precise maneuvering.
- Handles emergency stop and reset functionalities for enhanced safety.

**Usage**:

```bash
rosrun mur_control blended_manual_and_pid_setpoint_controller.py
```

#### manual_with_PID_setpoint_generator.py

**Purpose**: Allows manual generation of pose setpoints based on keyboard inputs, facilitating real-time adjustments to the robot's desired position and orientation.

**Key Functionalities**:

- Captures specific key presses to incrementally adjust pose setpoints.
- Converts Euler angles to quaternions for proper orientation representation.
- Publishes `PoseStamped` messages containing the updated setpoints at a specified rate.

**Usage**:

```bash
rosrun mur_control manual_with_PID_setpoint_generator.py
```

#### ros_manual_thruster_driver.py

**Purpose**: Enables manual control of the robot's thrusters using keyboard inputs, allowing users to directly influence the robot's movement and orientation.

**Key Functionalities**:

- Maps keyboard inputs to specific force and torque commands for thrusters.
- Scales input values to appropriate magnitudes for thruster actuation.
- Publishes `WrenchStamped` messages containing the computed force and torque to control thrusters.
- Implements emergency stop functionality for immediate cessation of thruster commands.

**Usage**:

```bash
rosrun mur_control ros_manual_thruster_driver.py
```

#### ros_pid_setpoint_control.py

**Purpose**: Implements a PID-based stabilization controller that manages the robot's pose by minimizing the error between current state and desired setpoints.

**Key Functionalities**:

- Subscribes to pose setpoints and updates internal setpoints accordingly.
- Retrieves the robot's current pose using TF2 transformations.
- Calculates errors between current pose and setpoints.
- Applies PID control to compute wrench commands.
- Publishes wrench commands to the appropriate ROS topic for thruster actuation.

**Usage**:

```bash
rosrun mur_control ros_pid_setpoint_control.py
```

#### ros_pose_setpoint_generator.py

**Purpose**: Generates and publishes smooth pose setpoints for the robot, enabling autonomous navigation or predefined movement trajectories.

**Key Functionalities**:

- Creates `PoseStamped` messages that define desired positions and orientations.
- Converts Euler angles to quaternions for accurate orientation representation.
- Publishes pose setpoints to the `pose_setpoint` ROS topic at a specified rate.

**Usage**:

```bash
rosrun mur_control ros_pose_setpoint_generator.py
```

## Dependencies

- **ROS Packages**:
  - `rospy`: Python client library for ROS.
  - `tf2_ros`: TF2 library for ROS transformations.
  - `geometry_msgs`: Standard ROS messages for geometric data.
  - `std_msgs`: Standard ROS messages.
  - `simple_pid`: PID controller library.
  - `tf`: ROS package for handling transformations.

- **Python Libraries**:
  - `pygame`: Library for creating multimedia applications (used here for capturing keyboard inputs).
  - `numpy`: Fundamental package for scientific computing with Python.
  - `math`: Standard Python library for mathematical operations.
  - `time`: Time-related functions.

*Ensure all dependencies are installed before running the control scripts.*

## License

This project is licensed under the [MIT License](LICENSE).
