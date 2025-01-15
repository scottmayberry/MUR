#!/usr/bin/env python
"""
ros_pose_setpoint_generator.py

Author: Scott Mayberry
Date: 2025-01-10

Description:
    This script generates and publishes pose setpoints for the Miniature Underwater Robot (MUR)
    within the ROS ecosystem. It creates PoseStamped messages that define desired positions
    and orientations (setpoints) for the robot to achieve. This script is about creating a smooth 
    set of setpoints to move the robot.

    Key Functionalities:
        1. Generates smooth pose setpoints based on time or predefined trajectories.
        2. Converts Euler angles (roll, pitch, yaw) to quaternions for proper orientation representation.
        3. Publishes PoseStamped messages to the 'pose_setpoint' ROS topic at a specified rate.

Dependencies:
    - rospy: Python client library for ROS.
    - numpy: Fundamental package for scientific computing with Python.
    - geometry_msgs.msg: ROS message types for geometric primitives such as points, vectors, and poses.
    - tf.transformations: Utility functions for transforming between different representations (e.g., Euler angles to quaternions).
    - cv_bridge: ROS package to convert between ROS Image messages and OpenCV images.

License:
    MIT License
"""

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def generate_smooth_pose(t):
    """
    Generates a PoseStamped message with smooth, sinusoidal trajectories for position and orientation.

    Args:
        t (float): Current time in seconds.

    Returns:
        PoseStamped: A ROS message containing the desired pose (position and orientation).
    """
    # Initialize position and orientation variables
    x = y = z = pitch = yaw = roll = 0

    # Uncomment and modify the following lines to define dynamic trajectories
    # Example of generating sinusoidal trajectories for smooth movement
    # roll = np.pi  # Static roll angle
    # x = 10 * np.sin(t)  # Oscillate x between -10 and 10 meters
    # y = 3 * np.sin(t + np.pi/3)  # Oscillate y with phase shift
    # z = 4 * np.sin(t + 2*np.pi/3)  # Oscillate z with different phase shift
    # roll = 2 * np.sin(t/10 + np.pi) + roll  # Oscillate roll
    # pitch = 0  # Static pitch angle
    # yaw = np.pi  # Static yaw angle
    # yaw = 2 * np.sin(t/10 + 4*np.pi/3)  # Alternatively, oscillate yaw

    # Convert roll, pitch, yaw angles to a quaternion for orientation
    quat = quaternion_from_euler(roll, pitch, yaw)

    # Create and populate a PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()  # Current time stamp
    pose_msg.header.frame_id = "world"  # Reference frame for the pose

    # Set the desired position
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z

    # Set the desired orientation using the quaternion
    pose_msg.pose.orientation = Quaternion(*quat)

    return pose_msg  # Return the populated PoseStamped message

def main():
    """
    Main function that initializes the ROS node, sets up the publisher, and enters a loop to publish pose setpoints.
    """
    rospy.init_node('pose_setpoint_generator')  # Initialize the ROS node with the name 'pose_setpoint_generator'

    # Publisher setup: publishes PoseStamped messages to the 'pose_setpoint' topic
    pub = rospy.Publisher('pose_setpoint', PoseStamped, queue_size=10)

    hz = 10.0  # Define the publishing rate (10 Hz)
    rate = rospy.Rate(hz)  # Create a Rate object to maintain the loop at 10 Hz
    t = 0.0  # Initialize time variable

    while not rospy.is_shutdown():  # Continue looping until ROS is shut down
        pose_msg = generate_smooth_pose(t)  # Generate the current pose setpoint based on time 't'
        pub.publish(pose_msg)  # Publish the PoseStamped message to the 'pose_setpoint' topic

        t += 1/hz  # Increment time by the duration of one loop iteration (0.1 seconds)
        rate.sleep()  # Sleep to maintain the loop rate at 10 Hz

if __name__ == '__main__':
    try:
        main()  # Execute the main function
    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exceptions gracefully (e.g., when the node is shut down)
