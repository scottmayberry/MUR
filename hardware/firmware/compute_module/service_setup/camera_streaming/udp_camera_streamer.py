#!/usr/bin/env python
"""
mjpg_streamer_launcher.py

Author: Scott Mayberry
Date: 2025-03-02

Description:
    This script listens for UDP messages containing configuration parameters to launch
    the mjpg_streamer process for video devices. When a UDP message is received in JSON format,
    the script extracts parameters such as the unique device identifier (ID_PATH_TAG), desired
    resolution, frame rate, stream port, and autofocus setting. It then locates video devices
    matching the provided ID_PATH_TAG using system commands and launches mjpg_streamer with the
    specified settings. The script also supports disabling the camera's autofocus if requested.
    
    Key Functionalities:
        1. Determines the script's directory to locate required plugins.
        2. Queries the system for video devices matching a given ID_PATH_TAG.
        3. Retrieves camera control settings using system utilities.
        4. Disables autofocus on the camera when needed.
        5. Constructs and launches the mjpg_streamer process with the provided parameters.
        6. Listens on a UDP port for incoming JSON messages containing configuration parameters.
        7. Prevents duplicate launches by tracking already processed ID_PATH_TAGs.

Dependencies:
    - socket: For creating and binding UDP sockets.
    - json: For parsing incoming JSON messages.
    - subprocess: For running system commands and launching external processes.
    - time: For introducing delays to reduce CPU usage.
    - os: For handling file paths and environment variables.

License:
    MIT License
"""

import socket
import json
import subprocess
import time
import os

# ---------------------------------------------------------------------------------
# Function: get_script_dir
# ---------------------------------------------------------------------------------
# Returns the directory where this script is located.
# This is useful for constructing paths relative to the script location.
def get_script_dir():
    return os.path.dirname(os.path.realpath(__file__))

# ---------------------------------------------------------------------------------
# Function: get_video_device_by_id_path_tag
# ---------------------------------------------------------------------------------
# Searches for video devices that match a specified ID_PATH_TAG.
# It iterates over possible video device file names (/dev/video0 to /dev/video9)
# and uses the 'udevadm' command to query device information.
# If the output contains the matching ID_PATH_TAG, the device is added to the list.
def get_video_device_by_id_path_tag(id_path_tag):
    devices = []
    for i in range(10):  # Assuming a maximum of 10 video devices
        device = f"/dev/video{i}"
        try:
            # Run the 'udevadm' command to get detailed device info
            result = subprocess.run(
                ["udevadm", "info", "--query=all", "--name=" + device],
                capture_output=True, text=True, check=True
            )
            # Check if the device info contains the desired ID_PATH_TAG
            if f"ID_PATH_TAG={id_path_tag}" in result.stdout:
                devices.append(device)
        except subprocess.CalledProcessError:
            # If udevadm fails (device does not exist or another error), continue to next device
            continue
    return devices

# ---------------------------------------------------------------------------------
# Function: get_camera_control
# ---------------------------------------------------------------------------------
# Checks if a specific camera control exists for a given device and retrieves its current value.
# Uses the 'v4l2-ctl' command to query the control.
def get_camera_control(device, control_name):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--get-ctrl", control_name],
            capture_output=True, text=True, check=True
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        # Return None if the control is not available or the command fails
        return None

# ---------------------------------------------------------------------------------
# Function: disable_autofocus
# ---------------------------------------------------------------------------------
# Attempts to disable the camera's autofocus by setting the 'focus_automatic_continuous' control to 0.
# If the command fails, it prints an error message but continues execution.
def disable_autofocus(device):
    autofocus_control = "focus_automatic_continuous"
    
    try:
        # Attempt to disable autofocus using v4l2-ctl
        subprocess.run(
            ["v4l2-ctl", "-d", device, "-c", f"{autofocus_control}=0"],
            check=True
        )
        print(f"Autofocus disabled on {device}")
    except subprocess.CalledProcessError:
        # Print a warning if disabling autofocus fails, but do not stop execution
        print(f"Failed to disable autofocus on {device}. Continuing without changes.")

# ---------------------------------------------------------------------------------
# Function: launch_mjpg_streamer
# ---------------------------------------------------------------------------------
# Launches the mjpg_streamer process for a given video device with specified parameters:
# resolution (width and height), frame rate (fps), stream port, and autofocus setting.
# It verifies the existence of the necessary input and output plugins,
# disables autofocus if requested, constructs the command for mjpg_streamer,
# and starts the process in the background.
def launch_mjpg_streamer(device, width, height, fps, stream_port, autofocus=True):
    # Get the directory where this script is located
    script_dir = get_script_dir()
    # Define the plugin directory based on the script's location
    plugin_dir = os.path.join(script_dir, "mjpg-streamer/mjpg-streamer-experimental")
    # Define paths to the input and output plugins
    input_plugin = os.path.join(plugin_dir, "input_uvc.so")
    output_plugin = os.path.join(plugin_dir, "output_http.so")
    # Define the directory for the web interface files
    www_folder = os.path.join(plugin_dir, "www")
    
    # Verify that the input plugin exists
    if not os.path.exists(input_plugin):
        print(f"ERROR: {input_plugin} does not exist")
        return False
    # Verify that the output plugin exists
    if not os.path.exists(output_plugin):
        print(f"ERROR: {output_plugin} does not exist")
        return False

    # If autofocus is not desired, disable it on the device
    if not autofocus:
        disable_autofocus(device)

    # Construct the command to launch mjpg_streamer with appropriate parameters
    command = [
        "/usr/local/bin/mjpg_streamer",
        "-i", f"{input_plugin} -d {device} -r {width}x{height} -f {fps}",
        "-o", f"{output_plugin} -w {www_folder} -p {stream_port}"
    ]
    # Copy the current environment and update LD_LIBRARY_PATH to include the plugin directory
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = plugin_dir
    try:
        # Launch the mjpg_streamer process in the background
        subprocess.Popen(command, env=env)
        print(f"Successfully launched mjpg_streamer on {device}")
        return True
    except Exception as e:
        # Handle any exceptions that occur during process launch
        print(f"Failed to launch mjpg_streamer on {device}: {e}")
        return False

# ---------------------------------------------------------------------------------
# Function: main
# ---------------------------------------------------------------------------------
# The main function sets up a UDP socket to listen for incoming messages on a specified port.
# Upon receiving a message, it parses the JSON to extract camera parameters and checks if the
# device corresponding to the provided ID_PATH_TAG has already been launched.
# If not, it finds the matching video devices and attempts to launch mjpg_streamer with the
# provided settings. The loop includes a short sleep to reduce CPU usage.
def main():
    UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
    UDP_PORT = 5005     # Port number for receiving UDP messages

    launched_tags = set()  # Set to keep track of ID_PATH_TAGs that have been processed

    # Create and bind the UDP socket to listen for incoming messages
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP messages on {UDP_IP}:{UDP_PORT}")

    # Main loop to process incoming UDP messages continuously
    while True:
        # Receive data from the UDP socket (buffer size of 1024 bytes)
        data, addr = sock.recvfrom(1024)
        try:
            # Decode the incoming data from bytes to string and parse as JSON
            message = json.loads(data.decode('utf-8'))
            # Extract configuration parameters from the JSON message
            id_path_tag = message["id_path_tag"]
            width = message["width"]
            height = message["height"]
            fps = message["fps"]
            stream_port = message["stream_port"]
            autofocus = message.get("autofocus", False)  # Default to turning off autofocus if not provided

            # If this ID_PATH_TAG has already been processed, skip to avoid duplicate launches
            if id_path_tag in launched_tags:
                print(f"Ignoring message for already launched ID_PATH_TAG {id_path_tag}")
                continue

            # Retrieve a list of video devices that match the specified ID_PATH_TAG
            devices = get_video_device_by_id_path_tag(id_path_tag)
            if not devices:
                print(f"No devices found for ID_PATH_TAG {id_path_tag}")
                continue

            # Attempt to launch mjpg_streamer for each matching device until successful
            for device in devices:
                if launch_mjpg_streamer(device, width, height, fps, stream_port, autofocus):
                    # Mark this ID_PATH_TAG as processed if a launch is successful
                    launched_tags.add(id_path_tag)
                    break

        except json.JSONDecodeError:
            # Handle the case where the received message is not valid JSON
            print("Received invalid JSON message")

        # Short sleep to reduce CPU usage in the loop
        time.sleep(0.05)

# ---------------------------------------------------------------------------------
# Script Entry Point
# ---------------------------------------------------------------------------------
if __name__ == "__main__":
    main()
