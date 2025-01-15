#!/usr/bin/env python
"""
camera_udp_to_ros.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script bridges UDP-based camera boot requests with ROS-based image publishing for the Miniature Underwater Robot (MUR).
    It retrieves camera configurations from the ROS parameter server, constructs stream URLs for each camera, and initiates
    separate threads to handle each camera's video stream. The captured frames are converted to ROS Image messages and
    published to corresponding ROS topics for further processing and visualization within the ROS ecosystem.

    Key Functions:
        1. Constructs JSON-formatted UDP messages for camera boot requests.
        2. Initializes and manages UDP camera boot servers to send boot requests to cameras.
        3. Launches camera nodes that capture video streams and publish them as ROS Image messages.
        4. Manages multiple camera streams concurrently using threading.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model camera_udp_to_ros.py
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
    - std_msgs.msg (Header)
    - geometry_msgs.msg
    - sensor_msgs.msg (Image)
    - cv2 (OpenCV)
    - threading
    - cv_bridge

License:
    MIT License
"""

import time
import rospy
import cv2
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

def open_stream(camera):
    """
    Opens a video stream for a given camera and initiates publishing frames to a ROS topic.

    Args:
        camera (dict): A dictionary containing camera configuration details such as stream URL, FPS, ID, frame transformations, and flip settings.
    """
    stream_url = camera['stream_url']  # URL of the camera's video stream
    fps = camera['fps']  # Frames per second for publishing
    cam_id = camera['id']  # Unique identifier for the camera
    # Determine the frame ID for TF2; use 'camera_<id>' if 'name' is not specified
    cam_frame_tf2 = f'camera_{cam_id}' if 'name' not in camera.keys() else camera['name']
    flip_horizontal = camera['flip_horizontal']  # Flag to flip the image horizontally
    flip_vertical = camera['flip_vertical']  # Flag to flip the image vertically

    # Continuously attempt to open the video stream until ROS is shut down
    while not rospy.is_shutdown():
        cap = cv2.VideoCapture(stream_url)  # Attempt to open the video stream
        if cap.isOpened():
            rospy.loginfo(f"Successfully opened stream for camera_{cam_id} at {fps} FPS")  # Log successful stream opening
            # Launch a camera node with the appropriate index and configurations
            launch_camera_node(cam_id, cap, fps, cam_frame_tf2, flip_horizontal, flip_vertical)
            break  # Exit the loop after successfully launching the camera node
        else:
            rospy.logwarn(f"Failed to open stream for camera_{cam_id}. Retrying in 10 seconds...")  # Log warning on failure
            time.sleep(10)  # Wait for 10 seconds before retrying
        cap.release()  # Release the video capture object before retrying

def launch_camera_node(cam_id, cap, target_fps, cam_frame_tf2, flip_horizontal=False, flip_vertical=False):
    """
    Publishes video frames from a camera to a ROS Image topic.

    Args:
        cam_id (int): Unique identifier for the camera.
        cap (cv2.VideoCapture): OpenCV VideoCapture object for the camera stream.
        target_fps (float): Target frames per second for publishing.
        cam_frame_tf2 (str): Frame ID for TF2 transformations.
        flip_horizontal (bool, optional): Whether to flip the image horizontally. Defaults to False.
        flip_vertical (bool, optional): Whether to flip the image vertically. Defaults to False.
    """
    # Initialize a ROS publisher for the camera's image topic
    pub = rospy.Publisher(f'/camera_{cam_id}/image_raw', Image, queue_size=10)
    rate = rospy.Rate(target_fps)  # Set the publishing rate based on target FPS
    bridge = CvBridge()  # Initialize CvBridge for converting OpenCV images to ROS Image messages

    header = Header()  # Initialize the ROS Header
    header.frame_id = cam_frame_tf2  # Set the frame ID for the image

    # Continuously read frames from the video stream and publish them until ROS is shut down or the stream is closed
    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()  # Read a frame from the video stream
        if ret:
            # Apply flipping based on the configuration flags
            if flip_horizontal and flip_vertical:
                frame = cv2.flip(frame, -1)  # Flip both horizontally and vertically
            elif flip_horizontal:
                frame = cv2.flip(frame, 1)  # Flip horizontally
            elif flip_vertical:
                frame = cv2.flip(frame, 0)  # Flip vertically
            try:
                # Convert the OpenCV frame to a ROS Image message
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                header.stamp = rospy.Time.now()  # Update the timestamp in the header
                image_msg.header = header  # Assign the header to the Image message
                pub.publish(image_msg)  # Publish the Image message to the ROS topic
            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert frame for camera_{cam_id}: {e}")  # Log conversion errors
        else:
            rospy.logwarn(f"Failed to read frame from camera_{cam_id}")  # Log warning if frame reading fails
        
        rate.sleep()  # Sleep to maintain the publishing rate

def load_camera_streams(compute_module_ip, cameras):
    """
    Constructs stream URLs for each camera and starts separate threads to handle each camera's stream.

    Args:
        compute_module_ip (str): IP address of the compute module to send boot requests.
        cameras (list): A list of dictionaries, each containing configuration details for a camera.
    """
    # Construct stream URLs for each camera based on the compute module's IP and camera's stream port
    for camera in cameras:
        stream_port = camera['stream_port']  # UDP port for the camera's stream
        camera['stream_url'] = f"http://{compute_module_ip}:{stream_port}/?action=stream"  # Construct the stream URL

    # Initialize a list to keep track of camera threads
    threads = []
    # Start a separate thread for each camera to handle its video stream concurrently
    for index, camera in enumerate(cameras):
        t = threading.Thread(target=open_stream, args=(camera,))  # Create a new thread for the camera
        t.start()  # Start the thread
        threads.append(t)  # Add the thread to the list

    # Wait for all camera threads to finish execution
    for t in threads:
        t.join()

if __name__ == "__main__":
    """
    Entry point for the Camera UDP to ROS bridge script.

    This section initializes the ROS node, retrieves camera and compute module configurations from ROS parameters,
    constructs stream URLs, and starts the UDP camera boot server and camera streaming threads.
    """
    rospy.init_node('cameras', anonymous=True)  # Initialize the ROS node with a unique name
    camera_data = rospy.get_param("/model_info/camera_data")  # Retrieve camera-related configuration from ROS parameters
    modules = compute_module = None  # Initialize variables for module information

    # Continuously attempt to retrieve the compute module's IP address until it is available
    while True:
        modules = rospy.get_param("/module_info/modules")  # Retrieve all module configurations from ROS parameters
        # Find the compute module configuration by name
        compute_module = [module for module in modules if module["name"] == "mur_compute_module"][0]
        if 'ipaddress' in compute_module.keys():  # Check if the compute module has an IP address configured
            break  # Exit the loop if the compute module is properly configured
        time.sleep(5)  # Wait for 5 seconds before retrying if the compute module is not yet configured

    compute_module_ip = compute_module['ipaddress']  # Retrieve the compute module's IP address
    cameras = camera_data['cameras']  # Retrieve the list of camera configurations from the camera data

    try:
        load_camera_streams(compute_module_ip, cameras)  # Start loading and handling camera streams
    except rospy.ROSInterruptException:
        pass  # Gracefully handle ROS interrupt exceptions (e.g., when the node is shut down)
