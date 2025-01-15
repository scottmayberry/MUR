#!/usr/bin/env python
"""
ros_thruster_wrench_generator.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script generates and publishes smooth wrench commands (forces and torques) for the Miniature Underwater Robot (MUR).
    It creates `WrenchStamped` messages with sinusoidal force components and publishes them to the `requested_thruster_wrench` ROS topic.
    The generated commands can be used for testing, calibration, or simulation purposes to evaluate the robot's thruster response
    and overall control algorithms.

Usage:
    Ensure that the ROS master is running before executing this script. Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model ros_thruster_wrench_generator.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - numpy
    - geometry_msgs.msg (WrenchStamped)

License:
    MIT License
"""

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped

def generate_smooth_command(t):
    """
    Generates a smooth wrench command based on the current time.

    This function creates sinusoidal force components for the x-axis while keeping other components static.
    It scales the generated forces and torques to achieve the desired command magnitudes.

    Args:
        t (float): The current time in seconds.

    Returns:
        WrenchStamped: The generated wrench command message.
    """
    # Generate sinusoidal force commands for testing purposes
    x = 1 * np.sin(t)
    y = 0  # 3 * np.sin(t + np.pi/3)  # Temporarily commented out for static y-force
    z = 0  # 4 * np.sin(t + 2*np.pi/3)  # Temporarily commented out for static z-force
    roll = 0  # 2 * np.sin(t + np.pi)    # Temporarily commented out for static roll torque
    pitch = 0
    yaw = 0  # 10 * np.sin(t + 4*np.pi/3)  # Temporarily commented out for static yaw torque

    scale = 2  # Scaling factor to adjust the magnitude of forces and torques

    # Scale the generated force and torque components
    x, y, z, roll, pitch, yaw = [k * scale for k in (x, y, z, roll, pitch, yaw)]

    # Create a WrenchStamped message with the generated force and torque
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()  # Timestamp of the message
    wrench_msg.header.frame_id = "com"           # Reference frame (center of mass)
    wrench_msg.wrench.force.x = x
    wrench_msg.wrench.force.y = y
    wrench_msg.wrench.force.z = z
    wrench_msg.wrench.torque.x = roll
    wrench_msg.wrench.torque.y = pitch  # Pitch is not used in this configuration
    wrench_msg.wrench.torque.z = yaw

    return wrench_msg

if __name__ == '__main__':
    """
    Entry point for the Thruster Wrench Generator script.

    Initializes the ROS node, sets up the publisher for thruster wrench commands,
    and enters a loop to continuously publish generated wrench messages at a specified rate.
    """
    rospy.init_node('body_frame_command_tester')  # Initialize the ROS node with a unique name

    # Set up the publisher to send WrenchStamped messages to the 'requested_thruster_wrench' topic
    pub = rospy.Publisher('requested_thruster_wrench', WrenchStamped, queue_size=10)

    hz = 10.0  # Frequency of message publication (10Hz)
    rate = rospy.Rate(hz)  # ROS rate object to control the loop rate
    t = 0.0  # Initialize time variable

    while not rospy.is_shutdown():
        wrench_msg = generate_smooth_command(t)  # Generate a new wrench command
        pub.publish(wrench_msg)  # Publish the wrench command to the ROS topic

        t += 1/hz  # Increment time by the period (0.1s) for the next iteration
        rate.sleep()  # Sleep to maintain the loop rate
