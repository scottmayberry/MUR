# MUR Embedded Code

The `mur_embedded` folder contains the Arduino code responsible for managing the various sensors, thrusters, and communication modules in the Miniature Underwater Robot (MUR). This document outlines the structure of the code, its components, and how they interact to operate the MUR effectively.

## Table of Contents

- [MUR Embedded Code](#mur-embedded-code)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Features](#features)
  - [Code Structure](#code-structure)
  - [Module Descriptions](#module-descriptions)
    - [Config (`config.h`)](#config-configh)
    - [EthernetManager (`EthernetManager.cpp` \& `EthernetManager.h`)](#ethernetmanager-ethernetmanagercpp--ethernetmanagerh)
    - [ThrusterManager (`ThrusterManager.cpp` \& `ThrusterManager.h`)](#thrustermanager-thrustermanagercpp--thrustermanagerh)
    - [SensorManager (`SensorManager.cpp` \& `SensorManager.h`)](#sensormanager-sensormanagercpp--sensormanagerh)
  - [Process Flow](#process-flow)
    - [Initialization](#initialization)
    - [Sensor Data Collection](#sensor-data-collection)
    - [Thruster Control](#thruster-control)
    - [Ethernet Communication](#ethernet-communication)
  - [Hardware Integration](#hardware-integration)
    - [Supported Hardware Components](#supported-hardware-components)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
    - [Setup Instructions](#setup-instructions)
  - [Usage](#usage)
  - [Troubleshooting](#troubleshooting)
  - [Future Improvements](#future-improvements)
  - [Contributing](#contributing)

## Overview

The MUR's embedded code is designed to handle real-time operations, including sensor data acquisition, thruster control, and communication with external systems. The code is modularized into different managers that handle specific aspects of the robot's functionality, ensuring scalability and maintainability.

## Features

- **Real-Time Sensor Data Acquisition**: Interfaces with multiple sensors to collect vital data for navigation and operation.
- **Thruster Control**: Manages thruster operations based on received commands to maneuver the robot underwater.
- **Ethernet Communication**: Facilitates robust communication with external systems using Ethernet and UDP protocols.
- **Modular Design**: Organized into distinct modules for easy maintenance and scalability.
- **Error Handling & Safety Mechanisms**: Implements checks and failsafes to ensure reliable operation even in adverse conditions.

## Code Structure

The codebase is organized into the following key components:

- **`config.h`**: Contains configuration settings and hardware definitions.
- **`EthernetManager.h` / `EthernetManager.cpp`**: Manages Ethernet communication.
- **`ThrusterManager.h` / `ThrusterManager.cpp`**: Controls the thrusters based on received commands.
- **`SensorManager.h` / `SensorManager.cpp`**: Interfaces with and processes data from various sensors.
- **`mur_embedded.ino`**: The main Arduino file that integrates all modules and orchestrates their interactions.

## Module Descriptions

### Config (`config.h`)

The `config.h` file defines essential configuration parameters and hardware-specific settings. It includes pin definitions, board types, and constants used across the codebase.

- **Board Type**: The code supports different board versions (`BOARD_COMPUTE` and `BOARD_FUSELAGE`). The appropriate board type must be defined to ensure correct operation.
- **UUID Configuration**: Unique identifiers for the robot are stored and managed here.
- **Ethernet and SPI Pins**: Configures the pins for Ethernet communication.

### EthernetManager (`EthernetManager.cpp` & `EthernetManager.h`)

The `EthernetManager` module handles all Ethernet communication, including UDP messaging and server requests. It supports both DHCP and static IP configurations.

- **Initialization**: Sets up the Ethernet shield and establishes a connection.
- **UDP Communication**: Sends and receives JSON-formatted messages via UDP.
- **Broadcasting**: Periodically sends out broadcast messages to discover the server IP.
- **Error Handling**: Implements retries and resets for robust connectivity.

### ThrusterManager (`ThrusterManager.cpp` & `ThrusterManager.h`)

The `ThrusterManager` module is responsible for controlling the MUR's thrusters. It processes commands and converts them into PWM signals to drive the thrusters.

- **Setup**: Initializes the thruster pins and servos.
- **Processing**: Monitors incoming commands and updates thruster states accordingly.
- **Safety**: Shuts down thrusters if no command is received within a defined period.
- **Command Handling**: Supports both unit-scaled and microsecond-based thruster commands.

### SensorManager (`SensorManager.cpp` & `SensorManager.h`)

The `SensorManager` module manages the MUR's sensors, including accelerometers, gyroscopes, magnetometers, pressure sensors, and more.

- **Initialization**: Sets up communication protocols (I2C, SPI) and initializes each sensor.
- **Data Processing**: Collects sensor data and packages it into JSON for transmission.
- **Interrupts**: Handles events like leak detection and magnetic switch activation.
- **Error Handling**: Disables sensors if initialization fails and logs errors for debugging.

## Process Flow

### Initialization

At startup, each module initializes its respective hardware components:

1. **EthernetManager** initializes the Ethernet shield and attempts to establish a connection.
2. **ThrusterManager** sets up the thruster servos and prepares them for command input (Fuselage board only).
3. **SensorManager** initializes all sensors and sets up communication protocols.

### Sensor Data Collection

The `SensorManager` continuously monitors the sensors and updates the system with new data:

1. Sensors are polled at specified intervals.
2. Data from each sensor is processed and added to a JSON document.
3. The JSON document is prepared for transmission over Ethernet.

### Thruster Control

The `ThrusterManager` listens for commands that adjust the MUR's thrusters:

1. Commands are received either in unit-scaled values or microseconds.
2. The commands are converted to PWM signals and sent to the appropriate thrusters.
3. Thrusters are periodically checked to ensure they are receiving valid commands.
4. Implements safety shutdown if no commands are received within the defined timeframe.

### Ethernet Communication

The `EthernetManager` handles communication between the MUR and external systems:

1. Periodic broadcast messages are sent to identify the server.
2. Incoming UDP messages are processed to extract commands and configuration updates.
3. Outgoing messages, including sensor data and status reports, are sent to the server.
4. Manages retries and resets to maintain robust connectivity.

## Hardware Integration

The code integrates tightly with the MUR's hardware, including:

- **ESC Control**: Manages the escs driving the thrusters using PWM signals.
- **Sensor Interfaces**: Connects with a variety of sensors, each using specific communication protocols (I2C, SPI).
- **Ethernet Shield**: Facilitates network communication for remote control and data transmission.
- **CAN Bus**: Utilizes FlexCAN for communication between different modules on the robot. (not currently implemented but availble)
- **Interrupts**: Handles hardware interrupts for events like leak detection and magnetometer switches.

### Supported Hardware Components

- **Sensors**:
  - Accelerometers: LSM6DS3TR-C, MPU6500, KXTJ3-1057
  - Gyroscopes: LSM6DS3TR-C, MPU6500
  - Magnetometers: LIS2MDL, HSCDTD008A, BNO055
  - Pressure Sensors: DPS310, MS5837
  - Humidity & Temperature: AHT20, HSCDTD008A
- **Communication Modules**:
  - Ethernet Shield (W5500)
  - CC1101 Radio Module
  - CAN Bus (FlexCAN_T4)
- **Actuators**:
  - Thruster Servos

## Installation

### Prerequisites

Ensure you have the following installed and set up before deploying the MUR embedded code:

- **Arduino IDE**: Version 1.8.13 or later.
- **Teensyduino**: Extension for the Arduino IDE to support Teensy boards.
- **Libraries**: The following Arduino libraries must be installed. You can install them via the Arduino Library Manager or download them from GitHub.
  - [Adafruit Sensor Library](https://github.com/adafruit/Adafruit_Sensor)
  - [Adafruit LSM6DS3TR-C Library](https://github.com/adafruit/Adafruit_LSM6DS)
  - [Adafruit LIS2MDL Library](https://github.com/adafruit/Adafruit_LIS2MDL)
  - [Adafruit BNO055 Library](https://github.com/adafruit/Adafruit_BNO055)
  - [Adafruit AHTX0 Library](https://github.com/adafruit/Adafruit_AHTX0)
  - [Adafruit DPS310 Library](https://github.com/adafruit/Adafruit_DPS310)
  - [MPU6500_WE Library](https://github.com/wollewald/MPU9250_WE)
  - [KXTJ3-1057 Library](https://github.com/KMotionX/KXTJ3-1057)
  - [HSCDTD008A Library](https://github.com/bobveringa/HSCDTD008A-Library)
  - [MS5837 Library](https://github.com/bluerobotics/BlueRobotics_MS5837_Library)
  - [RadioLib](https://github.com/jgromes/RadioLib)
  - [FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4)
  - [ArduinoJson](https://github.com/bblanchon/ArduinoJson)

### Setup Instructions

1. **Clone the Repository**:


2. **Open the Project in Arduino IDE**:

   - Launch the Arduino IDE.
   - Navigate to `File` > `Open` and select the `mur_embedded.ino` file from the cloned repository.

3. **Configure the Board Type**:

   - Open the `config.h` file.
   - Define the appropriate board type by uncommenting either `#define BOARD_COMPUTE` or `#define BOARD_FUSELAGE`. Ensure that only one is defined at a time.

4. **Connect Hardware Components**:

   - Connect the Teensy board to your computer via USB.
   - Ensure all sensors, thrusters, and communication modules are connected to the correct pins as defined in `config.h` and the respective manager modules.

5. **Install Required Libraries**:

   - Use the Arduino Library Manager (`Sketch` > `Include Library` > `Manage Libraries`) to install all necessary libraries listed in the [Prerequisites](#prerequisites) section.

6. **Upload the Code**:

   - Select the correct Teensy board and port from `Tools` > `Board` and `Tools` > `Port`.
   - Click the `Upload` button to compile and upload the code to the Teensy board.

## Usage

Once the code is successfully uploaded and the hardware is connected:

1. **Monitor Serial Output**:

   - Open the Serial Monitor (`Tools` > `Serial Monitor`) at `115200 baud` to view debug messages and status updates.

2. **Operation Flow**:

   - **Initialization**: The robot initializes Ethernet, sensors, and thrusters (if applicable).
   - **Communication**: The robot attempts to establish Ethernet connectivity and discover the server IP.
   - **Data Collection**: Sensors continuously collect data, which is packaged into JSON format.
   - **Thruster Control**: Receives thruster commands via Ethernet and adjusts thrusters accordingly.
   - **Data Transmission**: Sends sensor data and status updates to the server via UDP.

3. **Interacting with the Robot**:

   - Use the designated server interface to send thruster commands in either unit-scaled values or microseconds.
   - Monitor sensor data and thruster statuses through the server.

## Troubleshooting

If you encounter issues while deploying or operating the MUR, consider the following troubleshooting steps:

- **Ethernet Initialization Fails**:
  - Ensure the Ethernet shield is properly connected to the Teensy board.
  - Check network connectivity and DHCP server availability.

- **Sensor Initialization Fails**:
  - Confirm all sensors are correctly installed.
  - Verify I2C and SPI connections.
  - Check for library compatibility and updates.

- **Thruster Control Issues**:
  - Ensure thrusters are properly connected to the defined signal pins.
  - Verify PWM signal ranges and adjust thresholds if necessary.
  - Check for any physical obstructions or issues with thruster motors.

- **Communication Problems**:
  - Confirm that the server IP is correctly set and reachable.
  - Check firewall settings that might block UDP traffic.
  - Ensure that JSON messages are properly formatted and within buffer size limits.

- **General Debugging**:
  - Use the Serial Monitor to view debug messages and identify where the process might be failing.
  - Ensure all libraries are up-to-date and correctly installed.

## Future Improvements

Potential enhancements for the MUR's embedded code include:

- **Optimized Communication**: Implement more efficient communication protocols to reduce latency and increase reliability.
- **Advanced Sensor Fusion**: Integrate data from multiple sensors for improved navigation, stability, and environmental awareness.
- **Enhanced Error Handling**: Develop more sophisticated error detection and recovery mechanisms, especially in network communication and sensor data processing.
- **Power Management**: Implement power-saving modes to extend the robot's operational time underwater.
- **User Interface**: Develop a more comprehensive user interface for real-time monitoring and control of the robot.
- **Firmware Updates**: Enable over-the-air firmware updates to simplify maintenance and upgrades.

## Contributing

Contributions are welcome! If you'd like to improve the MUR embedded code, please follow these guidelines:

1. **Fork the Repository**: Click the `Fork` button at the top right of the repository page to create a personal copy.

2. **Create a New Branch**: Use descriptive names for branches, such as `feature/sensor-integration` or `bugfix/ethernet-connection`.

   ```bash
   git checkout -b feature/sensor-integration
   ```

3. **Make Your Changes**: Implement your improvements or fixes, ensuring that the code remains well-documented and adheres to the project's coding standards.

4. **Commit Your Changes**: Write clear and concise commit messages that describe the changes made.

   ```bash
   git commit -m "Add detailed comments to SensorManager for better clarity"
   ```

5. **Push to Your Fork**:

   ```bash
   git push origin feature/sensor-integration
   ```

6. **Submit a Pull Request**: Navigate to the original repository and submit a pull request detailing the changes you've made.

7. **Review and Feedback**: Collaborate with the maintainers to refine your contributions.