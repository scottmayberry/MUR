# MUR Embedded Code

The `mur_embedded` folder contains the Arduino code responsible for managing the various sensors, thrusters, and communication modules in the Miniature Underwater Robot (MUR). This document outlines the structure of the code, its components, and how they interact to operate the MUR.

## Table of Contents

- [MUR Embedded Code](#mur-embedded-code)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
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
  - [Future Improvements](#future-improvements)

## Overview

The MUR's embedded code is designed to handle real-time operations, including sensor data acquisition, thruster control, and communication with external systems. The code is modularized into different managers that handle specific aspects of the robot's functionality.

## Code Structure

The codebase is structured as follows:

- **config.h**: Contains configuration settings and hardware definitions.
- **EthernetManager.h/.cpp**: Manages Ethernet communication.
- **ThrusterManager.h/.cpp**: Controls the thrusters based on received commands.
- **SensorManager.h/.cpp**: Interfaces with and processes data from various sensors.
- **mur_embedded.ino**: The main Arduino file that integrates all modules.

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

### ThrusterManager (`ThrusterManager.cpp` & `ThrusterManager.h`)

The `ThrusterManager` module is responsible for controlling the MUR's thrusters. It processes commands and converts them into PWM signals to drive the thrusters.

- **Setup**: Initializes the thruster pins and servos.
- **Processing**: Monitors incoming commands and updates thruster states accordingly.
- **Safety**: Shuts down thrusters if no command is received within a defined period.

### SensorManager (`SensorManager.cpp` & `SensorManager.h`)

The `SensorManager` module manages the MUR's sensors, including accelerometers, gyroscopes, magnetometers, pressure sensors, and more.

- **Initialization**: Sets up communication protocols (I2C, SPI) and initializes each sensor.
- **Data Processing**: Collects sensor data and packages it into JSON for transmission.
- **Interrupts**: Handles events like leak detection and magnetic switch activation.

## Process Flow

### Initialization

At startup, each module initializes its respective hardware components:

1. **EthernetManager** initializes the Ethernet shield and attempts to establish a connection.
2. **ThrusterManager** sets up the thruster servos and prepares them for command input.
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

### Ethernet Communication

The `EthernetManager` handles communication between the MUR and external systems:

1. Periodic broadcast messages are sent to identify the server.
2. Incoming UDP messages are processed to extract commands and configuration updates.
3. Outgoing messages, including sensor data and status reports, are sent to the server.

## Hardware Integration

The code integrates tightly with the MUR's hardware, including:

- **Servo Control**: Manages the servos driving the thrusters.
- **Sensor Interfaces**: Connects with a variety of sensors, each using specific communication protocols.
- **Ethernet Shield**: Facilitates network communication.

## Future Improvements

Potential enhancements for the MUR's embedded code include:

- **Optimized Communication**: Implement more efficient communication protocols to reduce latency.
- **Advanced Sensor Fusion**: Integrate data from multiple sensors for improved navigation and stability.
- **Error Handling**: Improve error detection and recovery mechanisms, especially in network communication.