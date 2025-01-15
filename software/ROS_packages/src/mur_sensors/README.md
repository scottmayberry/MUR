# MUR Sensors Package

![MUR Sensors](./images/mur_sensors.png)

## Table of Contents

- [Overview](#overview)
- [Features](#features)
- [File Structure](#file-structure)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Scripts and Modules](#scripts-and-modules)
- [Retired Scripts](#retired-scripts)
- [Contributing](#contributing)
- [License](#license)

## Overview

The **MUR Sensors Package** is a component of the **Miniature Underwater Robot (MUR)** project. This package is responsible for managing and processing sensor data, enabling accurate environmental monitoring and navigation capabilities essential for underwater exploration. It leverages the Robot Operating System (ROS) framework to facilitate communication between hardware sensors and software algorithms.

## Features

- **Sensor Integration**: Supports a variety of sensors including IMUs, pressure sensors, magnetometers, temperature and humidity sensors.
- **Dynamic Network Configuration**: Automatically manages IP addresses and communication ports for sensor modules.
- **Sensor Fusion**: Implements sensor fusion algorithms to enhance data accuracy and reliability.
- **Modular Design**: Easily extensible and customizable to accommodate additional sensors and functionalities.
- **Real-Time Data Processing**: Efficiently handles incoming sensor data streams and publishes processed information to ROS topics.


### Directory Breakdown

using CMake.
- **config/**: Contains configuration files.
  - **sensor_config.yaml**: Defines sensor parameters, topics, and fusion settings.
- **launch/**: Contains ROS launch files.
  - **mur_sensors.launch**: Launches all necessary nodes and configurations for the sensors package.
- **LICENSE**: Specifies the licensing information for the package.
- **package.xml**: Defines the package dependencies and metadata for ROS.
- **src/**: Contains the source code scripts.
  - **ros_imu_sensor_fusion_dyn.py**: Dynamically fuses IMU sensor data.
  - **ros_sensor_udp_auto_network.py**: Manages automatic network configuration via UDP.
  - **ros_sensor_udp_server_dyn.py**: Handles dynamic UDP server functionalities for sensors.
  - **sensor_publisher_constructor.py**: Initializes and manages ROS publishers for sensors.
- **CMakeLists.txt**: Configuration file for building the ROS package 

## Installation

### Prerequisites

- **ROS**: Ensure that you have ROS (Robot Operating System) installed. The package is compatible with ROS Noetic and later versions.
- **Python 3**: The scripts are written in Python 3 and require corresponding dependencies.

## Configuration

### Sensor Configuration

The `sensor_config.yaml` file located in the `config/` directory defines all sensor parameters, including topics, message types, and sensor fusion settings.

#### Example Configuration

```yaml
sensor_info:
  comm_port: 51585
  broadcast_port: 51584
  server_id_message: "server"
  # Additional sensor configurations...
```

#### Editing the Configuration

1. **Open the Configuration File**

   ```bash
   nano config/sensor_config.yaml
   ```

2. **Modify Sensor Parameters**

   Adjust the parameters as needed to match your hardware setup and network configurations.

3. **Save and Close**

   Press `CTRL + O` to save and `CTRL + X` to exit the editor.

## Usage

### Launching the Sensors Package

Use the provided ROS launch file to start all necessary nodes and configurations:

```bash
roslaunch mur_sensors mur_sensors.launch
```

### Running Individual Scripts

While the launch file handles the initialization of all components, you can also run individual scripts manually for testing or debugging purposes.

#### Example: Running the UDP Auto Network Script

```bash
rosrun mur_sensors ros_sensor_udp_auto_network.py
```

## Scripts and Modules

### `ros_imu_sensor_fusion_dyn.py`

Handles the dynamic fusion of IMU data from multiple sensors. It subscribes to accelerometer, gyroscope, and magnetometer topics, processes the data using sensor fusion algorithms, and publishes the fused IMU data to a ROS topic.

**Key Features:**
- Subscribes to different sensor message types (`Twist`, `Vector3`).
- Utilizes sensor fusion algorithms (`Offset`, `Ahrs`).
- Publishes fused data as `Imu` messages.

### `ros_sensor_udp_auto_network.py`

Manages automatic network configuration by listening for UDP broadcast messages requesting IP address and communication port assignments. It updates module information and responds to requesting modules to ensure seamless network integration.

**Key Features:**
- Listens for UDP broadcasts on a specified port.
- Parses and processes incoming JSON messages.
- Updates ROS parameters with new network configurations.
- Sends response messages to requesting modules.

### `ros_sensor_udp_server_dyn.py`

Implements a dynamic UDP server that listens for incoming sensor data packets, processes them, and publishes the data to corresponding ROS topics. It supports multiple sensor types and ensures real-time data distribution.

**Key Features:**
- Initializes ROS node and UDP socket.
- Handles incoming UDP packets with sensor data.
- Publishes processed data to ROS topics based on sensor type.

### `sensor_publisher_constructor.py`

Initializes ROS publishers for all configured sensors based on the sensor configuration parameters. It ensures that each sensor has an associated ROS topic and publisher for data dissemination.

**Key Features:**
- Reads sensor configurations from ROS parameters.
- Creates ROS publishers for each sensor instance.
- Supports various message types (`Twist`, `Imu`, `Vector3`, `Float32`, `String`, `FluidPressure`).


## License

This project is licensed under the [MIT License](./LICENSE).
