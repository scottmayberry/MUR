#!/usr/bin/env python
"""
sensor_publisher_constructor.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script is responsible for initializing ROS publishers for all sensors
    configured in the Miniature Underwater Robot (MUR). It reads sensor configurations
    from ROS parameters, sets up the appropriate ROS topics, and prepares publishers
    for each sensor instance based on its data type and queue size.

Usage:
    This script is typically imported and used by other modules that require
    access to the initialized sensor publishers. It abstracts the publisher
    initialization process, allowing for easy management and scalability of sensors.

Dependencies:
    - rospy
    - std_msgs.msg (Float32, String)
    - geometry_msgs.msg (Twist, Vector3)
    - sensor_msgs.msg (Imu, FluidPressure)

License:
    [Specify your license here, e.g., MIT License]
"""

import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import *

def initialize_sensor_publishers():
    """
    Initializes ROS publishers for each sensor based on the sensor configuration.

    This function performs the following steps:
        1. Retrieves the sensor configurations from ROS parameters.
        2. Iterates through each sensor and its instances.
        3. Creates ROS publishers for each sensor instance based on its message type.
        4. Stores the publishers in a dictionary keyed by their respective ROS topics.

    Returns:
        tuple:
            - pubs (dict): A dictionary mapping ROS topics to their corresponding publishers.
            - sensors_config (dict): The sensor configuration retrieved from ROS parameters.
    """
    # Retrieve sensor configurations from the ROS parameter server
    sensors_config = rospy.get_param('/sensor_info/sensors')

    # Initialize an empty dictionary to hold ROS publishers
    pubs = {}

    # Iterate over each sensor defined in the configuration
    for sensor_name, sensor_data in sensors_config.items():
        topic_base = sensor_data['topic']  # Base name for the ROS topic
        msg_type = sensor_data['type']     # Message type for the ROS topic

        # Iterate over each instance of the current sensor
        for instance in sensor_data['instances']:
            module = instance['module']  # Module name associated with the sensor instance
            queue_size = instance['queue']  # Queue size for the ROS publisher

            # Construct the full ROS topic name using module and topic base
            topic = f"/sensors/{module}/{topic_base}"

            # Initialize the appropriate ROS publisher based on the message type
            if msg_type == "Twist":
                # Publisher for Twist messages (e.g., for IMUs like MPU6500, LSM6DS3)
                pubs[topic] = rospy.Publisher(topic, Twist, queue_size=queue_size)
            elif msg_type == "Imu":
                # Publisher for Imu messages (e.g., for BNO055)
                pubs[topic] = rospy.Publisher(topic, Imu, queue_size=queue_size)
            elif msg_type == "Vector3":
                # Publisher for Vector3 messages (e.g., for accelerometers, magnetometers)
                pubs[topic] = rospy.Publisher(topic, Vector3, queue_size=queue_size)
            elif msg_type == "Float32":
                # Publisher for Float32 messages (e.g., for temperature, humidity, pressure)
                pubs[topic] = rospy.Publisher(topic, Float32, queue_size=queue_size)
            elif msg_type == "String":
                # Publisher for String messages (if needed for textual data)
                pubs[topic] = rospy.Publisher(topic, String, queue_size=queue_size)
            elif msg_type == "FluidPressure":
                # Publisher for FluidPressure messages (e.g., for MS5837)
                pubs[topic] = rospy.Publisher(topic, FluidPressure, queue_size=queue_size)
            else:
                # Log a warning if an unsupported message type is encountered
                rospy.logwarn(f"Unsupported message type '{msg_type}' for sensor '{sensor_name}' on topic '{topic}'.")

    # Return the dictionary of publishers and the sensor configuration
    return pubs, sensors_config
