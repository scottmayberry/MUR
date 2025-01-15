#!/usr/bin/env python
"""
camera_boot_server_udp.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script serves as a UDP-based boot server for camera modules in the Miniature Underwater Robot (MUR).
    It periodically sends boot requests to specified camera modules to initialize or reset their states.
    The script retrieves camera configurations from the ROS parameter server, constructs JSON-formatted
    messages for each camera, and transmits these messages over UDP to the designated compute module.
    
    Key Functions:
        1. Initializes a UDP socket for communication.
        2. Retrieves camera configuration and compute module information from ROS parameters.
        3. Constructs JSON-formatted messages containing camera boot information.
        4. Sends boot requests to each camera at specified intervals.
    
Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model camera_boot_server_udp.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - socket
    - json
    - time
    - std_msgs.msg
    - geometry_msgs.msg
    - sensor_msgs.msg

License:
    MIT License
"""

import socket
import time
import rospy
import json
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

def construct_camera_udp_json(camera):
    """
    Constructs a JSON-formatted UDP message for a given camera.

    Args:
        camera (dict): A dictionary containing camera configuration details.

    Returns:
        bytes: JSON-formatted camera data encoded in UTF-8.
    """
    camera_json = json.dumps(camera)  # Serialize the camera dictionary to a JSON-formatted string
    encoded_camera_json = camera_json.encode('utf-8')  # Encode the JSON string to bytes
    return encoded_camera_json  # Return the encoded JSON message

def udp_camera_boot_server(compute_module, transmit_port, cameras, secondsBetweenPings):
    """
    Initializes the UDP camera boot server and sends boot requests to cameras at specified intervals.

    Args:
        compute_module (dict): A dictionary containing the compute module's configuration, including IP address.
        transmit_port (int): The UDP port number on which the compute module listens for camera boot requests.
        cameras (list): A list of dictionaries, each containing configuration details for a camera.
        secondsBetweenPings (float): The interval in seconds between consecutive boot request transmissions.
    """
    # Create a UDP socket for communication
    sock = socket.socket(socket.AF_INET,  # Use IPv4 addressing
                        socket.SOCK_DGRAM)  # Use UDP protocol
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of local addresses

    rospy.init_node('camera_boot_server', anonymous=True)  # Initialize the ROS node with a unique name

    rate = rospy.Rate(1 / secondsBetweenPings)  # Calculate the ROS rate based on the desired ping interval

    while not rospy.is_shutdown():  # Continue running until ROS is shut down
        for camera in cameras:  # Iterate over each camera configuration
            udp_camera_encoded_json = construct_camera_udp_json(camera)  # Construct the JSON message for the camera
            sock.sendto(udp_camera_encoded_json, (compute_module['ipaddress'], transmit_port))  # Send the UDP message to the compute module
            time.sleep(0.1)  # Short delay to prevent message collision or overwhelming the network
        rate.sleep()  # Sleep to maintain the desired ping rate


# The following line is temporarily commented out and can be used for retrieving the host IP address
# _, UDP_IP = get_Host_name_IP()

if __name__ == "__main__":
    """
    Entry point for the Camera Boot Server UDP script.

    This section retrieves camera configurations and compute module information from ROS parameters,
    determines the appropriate transmission port, and starts the UDP camera boot server.
    """
    camera_data = rospy.get_param("/model_info/camera_data")  # Retrieve camera-related configuration from ROS parameters

    try:
        secondsBetweenPings = camera_data["seconds_between_camera_boot_requests"]  # Get the ping interval from parameters
    except KeyError:
        secondsBetweenPings = 15  # Default ping interval if not specified

    modules = compute_module = None  # Initialize variables for module information

    while True:
        modules = rospy.get_param("/module_info/modules")  # Retrieve all module configurations from ROS parameters
        compute_module = [module for module in modules if module["name"] == "mur_compute_module"][0]  # Find the compute module configuration

        if 'ipaddress' in compute_module.keys():  # Check if the compute module has an IP address configured
            break  # Exit the loop if the compute module is properly configured

        time.sleep(5)  # Wait for 5 seconds before retrying if the compute module is not yet configured

    print("Starting Camera Service")  # Informational print statement indicating the start of the camera service

    transmit_port = compute_module['ports']['camera']  # Retrieve the UDP port number for camera communication from the compute module's configuration
    cameras = camera_data['cameras']  # Retrieve the list of camera configurations from the camera data

    try:
        udp_camera_boot_server(compute_module, transmit_port, cameras, secondsBetweenPings)  # Start the UDP camera boot server
    except rospy.ROSInterruptException:
        pass  # Gracefully handle ROS interrupt exceptions (e.g., when the node is shut down)
