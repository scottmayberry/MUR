# Miniature Underwater Robot (MUR) Control System

Welcome to the **Miniature Underwater Robot (MUR) Hardware PCB** repository. This documentation provides an overview of the integrated hardware components designed to control and monitor the operations of the MUR. The system is composed of three primary printed circuit boards (PCBs), each serving specialized functions to ensure efficient and reliable underwater exploration and operations.

## Table of Contents

- [Miniature Underwater Robot (MUR) Control System](#miniature-underwater-robot-mur-control-system)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Components](#components)
    - [1. MUR Compute Module Mini](#1-mur-compute-module-mini)
      - [Key Features](#key-features)
      - [Functionality](#functionality)
    - [2. MUR Sensor and ESC Control PCB](#2-mur-sensor-and-esc-control-pcb)
      - [Key Features](#key-features-1)
      - [Functionality](#functionality-1)
    - [3. MUR ESC to Thruster Control PCB](#3-mur-esc-to-thruster-control-pcb)
      - [Key Features](#key-features-2)
      - [Functionality](#functionality-2)
  - [System Integration](#system-integration)
  - [Applications](#applications)
  - [Getting Started](#getting-started)
  - [Contributing](#contributing)
  - [License](#license)

## Overview

The **MUR Control System** is a modular and scalable hardware architecture designed for controlling and monitoring miniature underwater robots. The system facilitates robust data processing, precise sensor management, efficient motor control, and reliable communication, enabling advanced functionalities such as autonomous navigation, environmental monitoring, and real-time data transmission.

The system architecture comprises the following core components:

1. **MUR Compute Module Mini**: Acts as the central processing unit, handling computation, communication, and overall system coordination.
2. **MUR Sensor and ESC Control PCB**: Manages sensor data acquisition and electronic speed controller (ESC) operations for precise motor control.
3. **MUR ESC to Thruster Control PCB**: Interfaces ESCs with thrusters and manages power distribution to propulsion systems.

Each component is designed to integrate with the others, providing a cohesive and efficient control system for underwater robotics applications.

## Components

### 1. MUR Compute Module Mini

<p align="center">
  <img src="./MUR_Compute_Module_Mini/images/compute_module_mini_front.png" alt="Compute Module Mini Front" width="250"/>
  <img src="./MUR_Compute_Module_Mini/images/compute_module_mini_back.png" alt="Compute Module Mini Back" width="250"/>
</p>

The **MUR Compute Module Mini** serves as the brain of the underwater robot, providing robust computational capabilities and versatile connectivity options necessary for complex underwater tasks.

#### Key Features

- **Raspberry Pi Compute Module 4 Integration**: Leverages the power and flexibility of the Raspberry Pi CM4 for efficient processing and widespread software support.
- **Teensy 4.0 Microcontroller**: Provides real-time processing capabilities for time-critical tasks and low-level hardware control.
- **CC1101 Radio Module Slot**: Enables low-power, long-range wireless communication for remote control and data transmission.
- **6-Port USB 2.0 Hub**: Offers ample connectivity for peripherals such as cameras, additional sensors, and storage devices.
- **3-Port Ethernet Switch**: Facilitates high-speed wired communication between onboard systems and external networks.
- **I2C Interfaces**:
  - *1 Port for CM4*: Allows integration of various I2C-compatible sensors and devices with the compute module.
  - *2 Ports for Teensy 4.0*: Provides additional connectivity for sensor expansion and specialized peripherals.
- **HDMI Port**: Supports high-definition video output for monitoring and debugging purposes.
- **5V Fan Port**: Ensures adequate cooling for the CM4 and other critical components during intensive operations.
- **USB OTG Port**: Simplifies flashing and updating the CM4 firmware.
- **Leak Detection Sensors**: Includes 3 leak detection interfaces (2 JST connectors and 1 pin header) for early detection of water ingress and system protection.
  
#### Functionality

The Compute Module Mini orchestrates the overall operation of the MUR by:

- Processing sensor data and executing control algorithms.
- Managing communication between various subsystems and external controllers.
- Providing interfaces for system monitoring, debugging, and updates.
- Ensuring system stability and reliability through effective power and thermal management.

### 2. MUR Sensor and ESC Control PCB

<p align="center">
  <img src="./MUR_Sensor_and_ESC_Control_PCB/images/sensor_and_esc_control_board_front.png" alt="Sensor and ESC Control PCB Front" width="250"/>
  <img src="./MUR_Sensor_and_ESC_Control_PCB/images/sensor_and_esc_control_board_back.png" alt="Sensor and ESC Control PCB Back" width="250"/>
</p>

The **MUR Sensor and ESC Control PCB** is dedicated to precise sensor data acquisition and efficient motor control, ensuring accurate environmental awareness and responsive maneuverability.

#### Key Features

- **Integrated Sensors**:
  - *Inertial Measurement Units (IMUs)*: Multiple IMUs (MPU6500, LSM6DS3TR-C, BNO055) provide comprehensive orientation and motion data.
  - *Magnetometers*: HSCDTD008A and LIS2MDLTR sensors enable accurate heading and orientation detection.
  - *Accelerometer*: KXTJ3-1057 accelerometer measures linear acceleration for movement analysis.
  - *Environmental Sensors*: AHT20 for temperature and humidity, DPS310 for barometric pressure, and ATGM332D-5N31 GPS module for positioning.
- **I/O Interfaces**:
  - *3 I2C Ports*: Allow for additional sensor and peripheral integrations.
  - *2 Serial Ports*: Support communication with various serial devices and modules.
  - *3 CAN Bus Interfaces*: Enable robust and reliable communication with motor controllers and other critical subsystems.
  - *8 PWM Outputs*: Provide precise control over servos and other actuators for complex movements.
- **Ethernet Connectivity**:
  - *W5500 Ethernet Controller*: Ensures stable and high-speed network communication.
  - *RJ45 Connector and Passthrough*: Facilitate easy and reliable wired connections for control and data exchange.
- **Power Management**:
  - *XT60 and XT30 Connectors*: Distribute power efficiently to various subsystems and external components.
- **Leak Detection**:
  - *2 JST Connectors and 3 Header Pins*: Provide multiple points for integrating leak detection sensors to monitor hull integrity.

#### Functionality

This PCB enhances the MUR's capabilities by:

- Collecting and processing critical environmental and positional data for informed decision-making and autonomous operations.
- Managing ESCs to control thruster speeds accurately, enabling precise and agile movements.
- Facilitating communication between sensors, actuators, and the central compute module.
- Ensuring system safety through proactive leak detection and responsive control measures.

### 3. MUR ESC to Thruster Control PCB

<p align="center">
  <img src="./MUR_ESC_to_Thruster_Control_PCB/images/esc_to_thruster_pcb_front.png" alt="ESC to Thruster Control PCB Front" width="250"/>
  <img src="./MUR_ESC_to_Thruster_Control_PCB/images/esc_to_thruster_pcb_back.png" alt="ESC to Thruster Control PCB Back" width="250"/>
</p>

The **MUR ESC to Thruster Control PCB** serves as the power distribution hub for the MUR's propulsion system, ensuring reliable and efficient delivery of power to the thrusters.

#### Key Features

- **ESC Connectivity**:
  - *8 XT30 Connectors*: Provide secure and efficient connections to Electronic Speed Controllers (ESCs) for each thruster.
- **Thruster Connections**:
  - *Spring Terminals*: Allow for quick and tool-less connection and disconnection of thrusters, facilitating easy maintenance and replacement.
- **Power Distribution**:
  - *XT60 Connectors*: Supply robust power output for high-demand components and systems.
- **Compact and Robust Design**: Ensures minimal space usage while providing durable and reliable connections suitable for harsh underwater environments.

#### Functionality

This PCB ensures effective propulsion control by:

- Distributing power efficiently from the main power source to all thrusters through ESCs.
- Providing reliable and maintainable connections between ESCs and thrusters, reducing downtime and simplifying repairs.
- Supporting high-current demands of the propulsion system while maintaining safety and performance standards.

## System Integration

The three PCBs are designed to work cohesively, providing a comprehensive control and monitoring solution for the MUR:

1. **Data Flow**: Sensor data collected by the **Sensor and ESC Control PCB** is transmitted to the **Compute Module Mini** for processing. Based on this data, control signals are generated and sent back to the **Sensor and ESC Control PCB** and **ESC to Thruster Control PCB** to adjust actuator outputs accordingly.

2. **Communication**: The **Compute Module Mini** manages both wired and wireless communications, interfacing with external control stations, data logging systems, and other networked devices, ensuring seamless command and control capabilities.

3. **Power Management**: Efficient power distribution is managed across all boards, with the **ESC to Thruster Control PCB** handling the high-current demands of propulsion, while the other boards manage power for processing, sensors, and communication modules.

4. **Safety and Reliability**: Integrated leak detection and thermal management systems across the boards ensure operational safety and longevity, critical for underwater applications.

<p align="center">
  <img src="./images/system_integration_diagram.png" alt="System Integration Diagram" width="600"/>
</p>

## Applications

The **MUR Control System** is suitable for a wide range of underwater applications, including but not limited to:

- **Environmental Monitoring**: Collecting data on water quality, temperature, and other environmental parameters.
- **Marine Research**: Conducting studies related to marine biology, geology, and oceanography.
- **Underwater Inspection**: Inspecting submerged structures such as pipelines, cables, and ship hulls.
- **Search and Rescue**: Assisting in locating objects or individuals underwater.
- **Aquaculture**: Monitoring and managing fish farms and other aquatic cultivation systems.

## Getting Started

To get started with the **MUR Control System**:

1. **Hardware Setup**:
   - Assemble the PCBs according to the provided schematics and connect them following the system integration guidelines.
   - Connect sensors, actuators, and power sources as specified.

2. **Software Configuration**:
   - Flash the Raspberry Pi Compute Module 4 and Teensy 4.0 with the appropriate firmware and software.
   - Configure network settings and communication protocols as needed.

3. **Testing and Calibration**:
   - Perform initial tests to ensure all components are functioning correctly.
   - Calibrate sensors and actuators for accurate performance.

4. **Deployment**:
   - Integrate the system into the underwater robot chassis.
   - Conduct field tests to validate system performance in real-world conditions.

For detailed instructions and documentation, please refer to the respective folders and manuals for each PCB.

## Contributing

We welcome contributions from the community to enhance and improve the **MUR Control System**. If you have ideas, suggestions, or improvements, please feel free to submit pull requests or open issues.

## License

This project is licensed under the [MIT License](LICENSE), allowing for open collaboration and sharing. Please review the license terms before use.

---

Thank you for your interest in the **MUR Control System**. We hope this documentation provides a clear and comprehensive understanding of the system's capabilities and applications. For further information, support, or collaboration opportunities, please contact our development team or visit our project website.

<p align="center">
  <img src="./images/mur_logo.png" alt="MUR Logo" width="150"/>
</p>