#!/usr/bin/env python
"""
ros_sensor_udp_server_dyn.py

Description:
    This script serves as a dynamic UDP server for the Miniature Underwater Robot (MUR) sensors.
    It listens for incoming UDP packets containing sensor data, processes the data, and
    publishes it to corresponding ROS (Robot Operating System) topics. The server handles
    multiple sensor types, including IMUs, magnetometers, pressure sensors, and more.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing
    this script. It initializes ROS nodes, sets up UDP sockets, and continuously listens for
    incoming sensor data to distribute to appropriate ROS topics.

Dependencies:
    - rospy
    - std_msgs
    - geometry_msgs
    - sensor_msgs
    - json
    - sensor_publisher_constructor (custom module)
    
"""

import socket
import sys
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import json
import sensor_publisher_constructor

def reload_module_info():
    """
    Reloads the module information from ROS parameters.

    Returns:
        dict: A dictionary containing module information.
    """
    return rospy.get_param('/module_info/modules')

def get_has_booted():
    """
    Retrieves the boot status of the modules from ROS parameters.

    Returns:
        bool: True if modules have booted, False otherwise.
    """
    return rospy.get_param('/module_info/hasBooted')

def set_has_booted(hasBooted: bool):
    """
    Sets the boot status of the modules in ROS parameters.

    Args:
        hasBooted (bool): The boot status to set.
    """
    rospy.set_param('/module_info/hasBooted', hasBooted)

def udp_server_listener_and_distributor():
    """
    Initializes the UDP server, listens for incoming sensor data, processes it,
    and publishes it to the appropriate ROS topics.

    The function performs the following steps:
        1. Initializes the ROS node.
        2. Sets up UDP socket parameters.
        3. Loads sensor and module configurations.
        4. Initializes ROS publishers for each sensor.
        5. Enters a loop to continuously listen for and process incoming UDP packets.
    """
    # Initialize the ROS node named 'udp_server'
    rospy.init_node('udp_server', anonymous=True)

    # Retrieve the initial boot status of the modules
    hasBooted = get_has_booted()

    # Set up UDP socket parameters
    UDP_IP = ''  # Listen on all available interfaces
    UDP_PORT_IN = int(rospy.get_param('/sensor_info/comm_port'))  # Port for incoming UDP data
    sock = socket.socket(socket.AF_INET,  # IPv4
                         socket.SOCK_DGRAM)  # UDP protocol
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow address reuse
    sock.bind((UDP_IP, UDP_PORT_IN))  # Bind the socket to the specified IP and port
    sock.settimeout(0.005)  # Set a timeout for blocking socket operations

    # Initialize IMU covariance matrix (identity matrix scaled by zero)
    imu_covariance = [0] * 9

    # Reload module information from ROS parameters
    module_info = reload_module_info()

    # Initialize ROS publishers for sensors using a custom constructor
    pubs, sensors_config = sensor_publisher_constructor.initialize_sensor_publishers()

    # Main loop to listen for and process incoming UDP packets
    while not rospy.is_shutdown():
        try:
            # Attempt to receive data from the UDP socket
            data, addr = sock.recvfrom(2048)  # Buffer size set to 2048 bytes
        except:
            # If no data is received within the timeout, continue the loop
            continue

        try:
            # Decode the received bytes to a UTF-8 string
            datastring = data.decode('utf8')
            # Parse the JSON-formatted string into a Python dictionary
            dataJson = json.loads(datastring)
            # Ignore messages that are server requests for IP
            if "serverRequestForIP" in dataJson:
                continue
        except:
            # If decoding or JSON parsing fails, skip to the next iteration
            continue

        try:
            # Extract the sender's IP address from the UDP packet
            sender_ip = addr[0]
            # Find the module name that matches the sender's IP address
            module_name = next((m['name'] for m in module_info if 'ipaddress' in m.keys() and m['ipaddress'] == sender_ip), None)
            if not module_name:
                # If the module name is not found, reload module info and continue
                module_info = reload_module_info()
                continue

            # Remove the 'id' field from the data, defaulting to 255 if not present
            module_id = dataJson.pop('id', 255)
            # Remove the 'isThrust' field to check if thrusters are working, defaulting to True
            are_thrusters_working = dataJson.pop('isThrust', True)

            if not are_thrusters_working:
                # If thrusters are not working, log an error and raise an exception
                print("THRUSTERS NOT CONNECTED...")
                raise ValueError("THRUSTERS NOT CONNECTED...")

            if not hasBooted:
                # If the modules have not booted yet, update the boot status
                hasBooted = True
                set_has_booted(hasBooted)
        except:
            # If any exception occurs during processing, skip to the next iteration
            continue

        try:
            # Iterate over each sensor's data in the JSON payload
            for sensor_name, info in dataJson.items():
                # Construct the ROS topic name based on sensor and module
                topic_base = sensors_config[sensor_name]['topic']
                topic = f"/sensors/{module_name}/{topic_base}"
                # Define the frame ID for the sensor's data
                frame_id = f"{sensor_name}_{module_name}"

                if topic in pubs:
                    # Publish data based on the sensor type
                    if sensor_name == "BNO055":
                        # Create Quaternion, Angular Velocity, and Linear Acceleration messages
                        orientation = Quaternion(info['q']['x'], info['q']['y'], info['q']['z'], info['q']['w'])
                        ang_vel = Vector3(info['av']['x'], info['av']['y'], info['av']['z'])
                        lin_acc = Vector3(info['la']['x'], info['la']['y'], info['la']['z'])
                        # Create a header with the current timestamp and frame ID
                        header = Header()
                        header.frame_id = frame_id
                        header.stamp = rospy.Time.now()
                        # Construct the Imu message with covariance
                        dataToSend = Imu(header, orientation, imu_covariance, ang_vel, imu_covariance, lin_acc, imu_covariance)
                        # Publish the Imu message to the corresponding ROS topic
                        pubs[topic].publish(dataToSend)

                    elif sensor_name in ["MPU6500", "LSM6DS3"]:
                        # Create Vector3 messages for accelerometer and gyroscope data
                        accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                        gyro = Vector3(info['gyr_x'], info['gyr_y'], info['gyr_z'])
                        # Construct the Twist message with accelerometer and gyroscope data
                        dataToSend = Twist(accel, gyro)
                        # Publish the Twist message to the corresponding ROS topic
                        pubs[topic].publish(dataToSend)

                    elif sensor_name == "KXTJ3":
                        # Create a Vector3 message for accelerometer data
                        accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                        dataToSend = accel
                        # Publish the accelerometer data to the corresponding ROS topic
                        pubs[topic].publish(dataToSend)

                    elif sensor_name in ["LIS2MDL", "HSCDTD"]:
                        # Create a Vector3 message for magnetometer data
                        mag = Vector3(info['x'], info['y'], -info['z'])  # Note: Inverting the z-axis
                        dataToSend = mag
                        # Publish the magnetometer data to the corresponding ROS topic
                        pubs[topic].publish(dataToSend)

                    elif sensor_name == "AHT20":
                        # Create Float32 messages for humidity and temperature
                        humid = Float32(info['humidity'])
                        temp = Float32(info['temp'])
                        # Publish humidity data to the corresponding ROS topic
                        pubs[topic].publish(humid)
                        # Uncomment the following line to publish temperature data
                        # pubs[f"{topic}_temp"].publish(temp)

                    elif sensor_name == "DPS310":
                        # Create Float32 messages for pressure and temperature
                        pressure = Float32(info['pressure'])
                        temp = Float32(info['temp'])
                        # Publish pressure data to the corresponding ROS topic
                        pubs[topic].publish(pressure)

                    elif sensor_name == "MS5837":
                        # Create a header with the current timestamp and frame ID
                        header = Header()
                        header.frame_id = frame_id
                        header.stamp = rospy.Time.now()
                        # Create a FluidPressure message with the pressure data
                        pressure = FluidPressure(header, info['pressure'], 0)  # Assuming zero temperature covariance
                        # Publish the FluidPressure message to the corresponding ROS topic
                        pubs[topic].publish(pressure)

        except Exception as e:
            # Log errors encountered during packet processing
            print(f"[ERROR]: packet processing failed due to {e}")
            print(f"[ERROR]: topic with error {topic}")
            continue

if __name__ == "__main__":
    try:
        # Start the UDP server listener and distributor
        udp_server_listener_and_distributor()
    except rospy.ROSInterruptException:
        # Handle ROS interrupt exceptions gracefully
        pass
