#!/usr/bin/env python
"""
ros_manual_thruster_driver.py

Author: Scott Mayberry
Date: 2025-01-10

Description:
    This script allows manual control of the Miniature Underwater Robot (MUR) thrusters using keyboard inputs.
    It captures keyboard events to generate wrench (force and torque) commands, which are then published
    to the `/model/requested_thruster_wrench` ROS topic. The wrench commands enable manual maneuvering
    of the robot by adjusting its position and orientation based on user inputs.

    Key Functionalities:
        1. Initializes a Pygame window to capture keyboard events.
        2. Maps specific keyboard keys to thruster commands for controlling movement along and rotation around
           the x, y, and z axes.
        3. Scales the input values to appropriate magnitudes for thruster actuation.
        4. Publishes `WrenchStamped` messages containing the computed force and torque to control the thrusters.
        5. Handles graceful shutdown upon receiving quit events or ROS interruptions.

Dependencies:
    - rospy: Python client library for ROS.
    - pygame: Library for creating multimedia applications (used here for capturing keyboard inputs).
    - geometry_msgs.msg: ROS message types for geometric data.
    - std_msgs.msg: Standard ROS message types.

License:
    MIT License
"""

import rospy  # ROS Python client library
import pygame  # Library for handling multimedia (keyboard inputs)
from geometry_msgs.msg import WrenchStamped  # ROS message type for wrench (force and torque)

# Initialize Pygame to capture keyboard events
pygame.init()
screen = pygame.display.set_mode((200, 200))  # Create a small Pygame window (required for event handling)

def get_keyboard_command():
    """
    Captures the current state of the keyboard and maps specific keys to thruster commands.

    Returns:
        WrenchStamped: A ROS message containing the computed force and torque based on keyboard inputs.
    """
    # Initialize command values for each degree of freedom (DOF)
    x, y, z, roll, pitch, yaw = 0, 0, 0, 0, 0, 0
    scale = 11.0  # Scale factor to adjust the magnitude of the commands

    # Get the current state of all keyboard keys
    keys = pygame.key.get_pressed()

    # Map specific keys to movement commands
    if keys[pygame.K_w]:
        x = 1  # Move forward along the x-axis
    if keys[pygame.K_s]:
        x = -1  # Move backward along the x-axis
    if keys[pygame.K_a]:
        y = -1  # Move left along the y-axis
    if keys[pygame.K_d]:
        y = 1  # Move right along the y-axis
    if keys[pygame.K_q]:
        z = 1  # Move upward along the z-axis
    if keys[pygame.K_e]:
        z = -1  # Move downward along the z-axis
    if keys[pygame.K_i]:
        roll = 1  # Roll clockwise
    if keys[pygame.K_k]:
        roll = -1  # Roll counter-clockwise
    if keys[pygame.K_j]:
        yaw = 1  # Yaw clockwise
    if keys[pygame.K_l]:
        yaw = -1  # Yaw counter-clockwise

    # Apply the scale factor to each command
    x, y, z, roll, pitch, yaw = [k * scale for k in (x, y, z, roll, pitch, yaw)]
    roll /= 8  # Reduce the roll command magnitude
    yaw /= 8  # Reduce the yaw command magnitude

    # Create a WrenchStamped message to hold the force and torque commands
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()  # Set the current timestamp
    wrench_msg.header.frame_id = "fossen_com"  # Reference frame for the wrench commands

    # Assign the computed forces to the wrench message
    wrench_msg.wrench.force.x = x
    wrench_msg.wrench.force.y = y
    wrench_msg.wrench.force.z = z

    # Assign the computed torques to the wrench message
    wrench_msg.wrench.torque.x = roll
    wrench_msg.wrench.torque.y = pitch  # Pitch is not used in this implementation
    wrench_msg.wrench.torque.z = yaw

    # The following lines are temporarily commented out and can be used for formatted output
    # print(f"Manual Thruster Wrench: Force - x: {wrench_msg.wrench.force.x:10.4f} y: {wrench_msg.wrench.force.y:10.4f} z: {wrench_msg.wrench.force.z:10.4f} "
    #       f"Torque - x: {wrench_msg.wrench.torque.x:10.4f} y: {wrench_msg.wrench.torque.y:10.4f} z: {wrench_msg.wrench.torque.z:10.4f}")

    return wrench_msg  # Return the populated wrench message

if __name__ == '__main__':
    """
    Entry point for the ROS Manual Thruster Driver script.

    This section initializes the ROS node, sets up the publisher for wrench commands, and enters a loop
    to continuously capture keyboard inputs, generate wrench messages, and publish them at the specified rate.
    It also handles Pygame events to allow graceful shutdown of the script.
    """
    rospy.init_node('body_frame_command_tester')  # Initialize the ROS node with the name 'body_frame_command_tester'

    # Publisher setup: publishes WrenchStamped messages to the '/model/requested_thruster_wrench' topic
    pub = rospy.Publisher('/model/requested_thruster_wrench', WrenchStamped, queue_size=10)

    try:
        # Attempt to retrieve the publishing rate from ROS parameters
        hz = rospy.get_param('/control_info/manual_thruster_update_hz')
    except:
        hz = 10.0  # Default to 10 Hz if the parameter is not set
    rate = rospy.Rate(hz)  # Create a Rate object to maintain the loop at the desired frequency

    try:
        while not rospy.is_shutdown():  # Continue looping until ROS is shut down
            # Process Pygame events to handle window closure and other events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown('Quit event received')  # Gracefully shut down the ROS node

            wrench_msg = get_keyboard_command()  # Generate the current wrench command based on keyboard inputs
            pub.publish(wrench_msg)  # Publish the wrench command to the thruster control topic
            rate.sleep()  # Sleep to maintain the loop rate

    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exceptions gracefully (e.g., when the node is shut down)
    finally:
        pygame.quit()  # Ensure that Pygame quits properly upon exiting the script
