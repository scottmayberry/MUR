# MUR Model Camera Module

![MUR Logo](path_to_logo_image) <!-- Replace with the actual path to your logo image -->

## Table of Contents

- [MUR Model Camera Module](#mur-model-camera-module)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Components](#components)
    - [camera\_boot\_server\_udp.py](#camera_boot_server_udppy)
      - [Purpose](#purpose)
      - [How It Works](#how-it-works)
      - [Key Functions](#key-functions)
      - [Data Flow](#data-flow)
    - [camera\_udp\_to\_ros.py](#camera_udp_to_rospy)
      - [Purpose](#purpose-1)
      - [How It Works](#how-it-works-1)
      - [Key Functions](#key-functions-1)
      - [Data Flow](#data-flow-1)
  - [Installation](#installation)
    - [Prerequisites](#prerequisites)
      - [Python Libraries](#python-libraries)
  - [Usage](#usage)
    - [Launching the Camera Module](#launching-the-camera-module)
      - [Using the ROS Launch File](#using-the-ros-launch-file)
      - [Direct Execution](#direct-execution)
  - [Configuration](#configuration)
    - [Camera Configuration](#camera-configuration)
      - [`model_info.yaml`](#model_infoyaml)
      - [`aruco_info.yaml`](#aruco_infoyaml)
  - [Dependencies](#dependencies)
  - [License](#license)

## Overview

The **MUR Model Camera Module** is a crucial part of the **Miniature Underwater Robot (MUR)** ROS package, responsible for managing camera operations. It consists of two primary Python scripts:

1. **camera_boot_server_udp.py**: Handles the initialization and booting of camera modules via UDP communication.
2. **camera_udp_to_ros.py**: Bridges the camera streams to ROS by converting UDP-based video feeds into ROS Image messages.

Together, these scripts ensure that the MUR's cameras are correctly initialized, managed, and integrated into the ROS ecosystem for real-time image processing and visualization.

Note, the camera setup currently is for using a tethered computer externally, and for booting up the cameras for live-feed over the ethernet tether. This is why udp boot and streaming is required.

**This requires the camera streaming service to be running on the CM4. (see the service_setup_guide.md for all CM4 setup).**

## Components

### camera_boot_server_udp.py

#### Purpose

The `camera_boot_server_udp.py` script serves as a UDP-based boot server for the MUR's camera modules. Its primary role is to periodically send boot requests to each camera, ensuring they are initialized or reset as needed.

#### How It Works

1. **Initialization**:
   - **UDP Socket Setup**: Initializes a UDP socket configured to reuse local addresses, facilitating reliable communication with camera modules.
   - **ROS Node Initialization**: Initializes a ROS node named `camera_boot_server` to interact with the ROS ecosystem.

2. **Configuration Retrieval**:
   - **Camera Data**: Retrieves camera configurations from the ROS parameter server under `/model_info/camera_data`, including details like `seconds_between_camera_boot_requests` and individual camera settings.
   - **Compute Module Information**: Continuously retrieves module configurations from `/module_info/modules` to identify the compute module responsible for handling camera boot requests.

3. **Boot Request Construction and Transmission**:
   - **JSON Message Construction**: For each camera, constructs a JSON-formatted message containing necessary boot information using the `construct_camera_udp_json` function.
   - **UDP Transmission**: Sends the JSON message over UDP to the compute module's IP address and designated port at intervals defined by `secondsBetweenPings`.

4. **Graceful Shutdown**:
   - Handles ROS interrupt exceptions to ensure the node shuts down cleanly without errors.

#### Key Functions

- **construct_camera_udp_json(camera)**:
  - **Description**: Serializes camera configuration details into a JSON-formatted byte string suitable for UDP transmission.
  - **Args**:
    - `camera` (dict): Camera configuration details.
  - **Returns**:
    - `bytes`: Encoded JSON message.

- **udp_camera_boot_server(compute_module, transmit_port, cameras, secondsBetweenPings)**:
  - **Description**: Manages the UDP socket and sends boot requests to each camera at specified intervals.
  - **Args**:
    - `compute_module` (dict): Compute module configuration, including IP address.
    - `transmit_port` (int): UDP port for transmitting boot requests.
    - `cameras` (list): List of camera configurations.
    - `secondsBetweenPings` (float): Interval between consecutive boot requests.

#### Data Flow

1. **Input**:
   - Camera configurations from `/model_info/camera_data`.
   - Compute module information from `/module_info/modules`.

2. **Process**:
   - Constructs JSON messages for each camera.
   - Sends these messages over UDP to the compute module.

3. **Output**:
   - Boot requests transmitted to each camera via UDP.
   - Cameras receive boot requests and initialize or reset accordingly.

### camera_udp_to_ros.py

#### Purpose

The `camera_udp_to_ros.py` script bridges the MUR's UDP-based camera streams with ROS by converting incoming video feeds into ROS Image messages. This integration allows for real-time processing and visualization of camera data within the ROS ecosystem.

#### How It Works

1. **Initialization**:
   - **ROS Node Initialization**: Initializes a ROS node named `cameras` to handle camera operations.
   - **Configuration Retrieval**: Fetches camera configurations from `/model_info/camera_data`, including stream URLs, frame rates, resolutions, and image transformation settings.

2. **Camera Stream Management**:
   - **Stream URL Construction**: Constructs HTTP stream URLs for each camera based on the compute module's IP address and the camera's designated stream port.
   - **Threading**: Launches separate threads for each camera to handle their respective video streams concurrently, ensuring efficient processing without blocking.

3. **Frame Capture and Publishing**:
   - **Video Capture**: Each thread attempts to open a video stream using OpenCV's `VideoCapture`. If successful, it proceeds to capture frames; otherwise, it retries after a delay.
   - **Image Transformation**: Applies any specified image transformations (e.g., horizontal or vertical flipping) to the captured frames.
   - **ROS Image Conversion**: Utilizes `CvBridge` to convert OpenCV images to ROS Image messages.
   - **Publishing**: Publishes the converted Image messages to designated ROS topics (e.g., `/camera_radial_left/image_raw`) for further processing and visualization.

4. **Graceful Shutdown**:
   - Handles ROS interrupt exceptions to ensure all camera streams are closed and threads are properly terminated upon shutdown.

#### Key Functions

- **open_stream(camera)**:
  - **Description**: Attempts to open a video stream for a specific camera and initiates the publishing of frames to a ROS topic upon successful connection.
  - **Args**:
    - `camera` (dict): Camera configuration details.

- **launch_camera_node(cam_id, cap, target_fps, cam_frame_tf2, flip_horizontal=False, flip_vertical=False)**:
  - **Description**: Continuously reads frames from an OpenCV VideoCapture object, applies transformations, converts them to ROS Image messages, and publishes them to a ROS topic.
  - **Args**:
    - `cam_id` (int): Unique identifier for the camera.
    - `cap` (cv2.VideoCapture): OpenCV VideoCapture object.
    - `target_fps` (float): Target frame rate for publishing.
    - `cam_frame_tf2` (str): Frame ID for TF2 transformations.
    - `flip_horizontal` (bool, optional): Whether to flip the image horizontally.
    - `flip_vertical` (bool, optional): Whether to flip the image vertically.

- **load_camera_streams(compute_module_ip, cameras)**:
  - **Description**: Constructs stream URLs for each camera and starts separate threads to handle each camera's video stream concurrently.
  - **Args**:
    - `compute_module_ip` (str): IP address of the compute module.
    - `cameras` (list): List of camera configurations.

#### Data Flow

1. **Input**:
   - Camera configurations from `/model_info/camera_data`.
   - Video streams accessed via constructed HTTP URLs.

2. **Process**:
   - Captures video frames using OpenCV.
   - Applies image transformations as configured.
   - Converts frames to ROS Image messages using `CvBridge`.
   - Publishes Image messages to ROS topics.

3. **Output**:
   - ROS Image messages published to topics such as `/camera_radial_left/image_raw`, accessible by other ROS nodes for processing and visualization.

## Installation

### Prerequisites

- **ROS**: Ensure that you have a compatible version of ROS installed (e.g., ROS Noetic, ROS Melodic).
- **Python Dependencies**: The camera module relies on several Python libraries. Install them using `pip` or your preferred package manager.

#### Python Libraries

```bash
pip install rospy opencv-python cv_bridge
```

*Note: Some dependencies like `cv_bridge` may require additional ROS-specific installation steps. Refer to the [ROS documentation](http://wiki.ros.org/cv_bridge) for detailed instructions.*

## Usage

### Launching the Camera Module

To start the camera module, you can launch it using the provided ROS launch file or run the scripts directly.

#### Using the ROS Launch File

The `mur_model.launch` file initializes all necessary nodes, including the camera boot server and camera stream handlers.

```bash
roslaunch mur_model mur_model.launch
```

#### Direct Execution

Alternatively, you can run the scripts individually if you prefer more granular control.

1. **Start the Camera Boot Server**

   ```bash
   rosrun mur_model camera_boot_server_udp.py
   ```

2. **Start the Camera to ROS Bridge**

   ```bash
   rosrun mur_model camera_udp_to_ros.py
   ```

*Ensure that the ROS master is running and that all necessary parameters are correctly set in the configuration files before executing the scripts.*

## Configuration

### Camera Configuration

The camera module relies on YAML configuration files to define camera properties and operational parameters. These configurations are stored in the `config/` directory.

#### `model_info.yaml`

Contains comprehensive information about the robot's structure, including camera settings.

- **Camera Settings**:
  - `stream_port`: UDP port used for streaming video.
  - `fps`: Frames per second for video capture.
  - `width` & `height`: Resolution of the video stream.
  - `flip_horizontal` & `flip_vertical`: Flags to flip the image orientation.
  - `pos` & `orientation`: Position and orientation of the camera relative to the robot's frame.

*Refer to the [model_info.yaml](config/model_info.yaml) for detailed comments and explanations.*

#### `aruco_info.yaml`

Defines the ArUco dictionary and specific markers used for control identification and positioning.

- **dictionary**: Specifies which ArUco dictionary to use (e.g., `DICT_4X4_50`).
- **tags**: Lists each ArUco marker with its unique ID, position, and orientation relative to the robot's frames.

*Refer to the [aruco_info.yaml](config/aruco_info.yaml) for detailed comments and explanations.*

## Dependencies

- **ROS Packages**:
  - `rospy`: Python client library for ROS.
  - `sensor_msgs`: Standard ROS messages for sensor data.
  - `std_msgs`: Standard ROS messages.
  - `geometry_msgs`: Standard ROS messages for geometric data.
  - `tf2_ros`: TF2 library for ROS.
  - `cv_bridge`: ROS package to convert between ROS Image messages and OpenCV images.

- **Python Libraries**:
  - `opencv-python`: OpenCV library for computer vision tasks.
  - `numpy`: Fundamental package for scientific computing with Python.
  - `json`: Module for JSON serialization and deserialization.
  - `socket`: Low-level networking interface for UDP communication.
  - `threading`: Module for creating and managing threads.
  - `queue`: Module for creating queue data structures.

*Ensure all dependencies are installed before running the camera module.*

## License

This project is licensed under the [MIT License](LICENSE).
