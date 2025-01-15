# MUR Firmware

The `firmware` directory contains all the essential code needed to operate the Miniature Underwater Robot (MUR). This includes both the core embedded code for running the robot's hardware (`mur_embedded`) and the testing scripts for communication systems (`embedded_communication_tests`). The folder is structured to facilitate development, testing, and deployment of the MUR's software and communication protocols.

## Table of Contents

- [MUR Firmware](#mur-firmware)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Folder Structure](#folder-structure)
  - [mur\_embedded](#mur_embedded)
  - [embedded\_communication\_tests](#embedded_communication_tests)
  - [Usage](#usage)
  - [Contributing](#contributing)

## Overview

The `firmware` folder is designed to house all necessary code for the MUR. The core functionalities, such as sensor management, thruster control, and Ethernet communication, are found in the `mur_embedded` folder. Complementary to this are the `embedded_communication_tests`, which provide scripts for testing and validating the robot's communication capabilities.

## Folder Structure

The `firmware` folder is organized as follows:

- **mur_embedded/**: Contains the core Arduino code responsible for running the MUR's hardware components, such as sensors, thrusters, and communication modules.
- **embedded_communication_tests/**: Includes Python scripts used to test the communication systems of the MUR, ensuring reliable data transmission and reception.

## mur_embedded

The `mur_embedded` directory holds the main embedded code for the MUR. It includes modules for:

- **Sensor Management**: Interfaces with various sensors (IMUs, pressure sensors, etc.) and processes their data.
- **Thruster Control**: Manages the PWM signals sent to the thrusters to control the robot's movement.
- **Ethernet Communication**: Handles the network communication between the MUR and external systems, enabling remote control and data transmission.

For detailed information on the structure and functionalities of the `mur_embedded` folder, refer to the [README](mur_embedded/README.md) within the folder.

## embedded_communication_tests

The `embedded_communication_tests` folder contains a series of Python scripts used to validate the MUR's communication systems. These tests are essential for ensuring that the robot can correctly send and receive data over its network interfaces.

Key scripts include:

- **UDP Transmission and Reception**: Tests for sending and receiving UDP packets between the MUR and other networked devices.
- **IMU Fusion Processing**: A script that processes IMU data to calculate orientation using the IMUFusion library.
- **Server ID Broadcasting**: Dynamically announces the server's IP address to the MUR, removing the need for hardcoded network settings.

For more details on how to use these scripts, refer to the [README](embedded_communication_tests/README.md) within the `embedded_communication_tests` folder.

## Usage

1. **Set Up the MUR**:
   - Begin by setting up the MUR with the `mur_embedded` firmware. Upload the Arduino code to the MUR’s microcontroller to manage its sensors, thrusters, and communication modules.

2. **Test Communication**:
   - Use the scripts in `embedded_communication_tests` to ensure that the MUR is correctly transmitting and receiving data. This is particularly important before deploying the robot in any mission-critical scenario.

3. **Development and Debugging**:
   - The structure allows developers to easily modify and test both the embedded code and the communication protocols. Make changes in the `mur_embedded` codebase and validate them with the `embedded_communication_tests`.

## Contributing

Contributions to the MUR firmware are welcome. Please follow these steps:

1. **Fork the Repository**: Start by forking the repository to your GitHub account.
2. **Create a New Branch**: Develop your feature or fix in a new branch.
3. **Test Your Changes**: Use the `embedded_communication_tests` to validate your changes.
4. **Submit a Pull Request**: Once your changes are tested and validated, submit a pull request for review.