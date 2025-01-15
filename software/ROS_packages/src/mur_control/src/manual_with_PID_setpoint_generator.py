#!/usr/bin/env python
"""
manual_with_PID_setpoint_generator.py

Author: Scott Mayberry
Date: 2025-01-10

Description:
    This script allows manual generation of pose setpoints for the Miniature Underwater Robot (MUR)
    using keyboard inputs. It captures specific key presses to adjust the robot's desired position
    and orientation in real-time. The script publishes `PoseStamped` messages to the `/control/pose_setpoint`
    ROS topic, which can be used by PID controllers or other control mechanisms to maneuver the robot.

    Key Functionalities:
        1. Initializes a Pygame window to capture keyboard events.
        2. Maps specific keyboard keys to incremental adjustments in position and orientation.
        3. Maintains and updates current pose setpoints based on user inputs.
        4. Converts Euler angles to quaternions for proper orientation representation.
        5. Publishes `PoseStamped` messages containing the updated setpoints at a specified rate.
        6. Handles graceful shutdown upon receiving quit events or ROS interruptions.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_control manual_with_PID_setpoint_generator.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_control mur_control.launch
        ```

Dependencies:
    - rospy: Python client library for ROS.
    - pygame: Library for creating multimedia applications (used here for capturing keyboard inputs).
    - geometry_msgs.msg: ROS message types for geometric data.
    - tf: ROS package for handling transformations between coordinate frames.
    - math: Standard Python library for mathematical operations.

License:
    MIT License
"""

import rospy  # ROS Python client library
import pygame  # Library for handling multimedia (keyboard inputs)
from geometry_msgs.msg import PoseStamped, Quaternion  # ROS message types for poses
import math  # Standard Python math library
import tf  # ROS package for handling transformations

# Initialize Pygame to capture keyboard events
pygame.init()
screen = pygame.display.set_mode((200, 200))  # Create a small Pygame window (required for event handling)

# Initialize the current pose setpoint
current_x, current_y, current_z, current_yaw = 0.0, 0.0, 0.0, 0.0  # Starting at origin with zero yaw

def get_keyboard_command():
    """
    Captures the current state of the keyboard and maps specific keys to pose adjustments.

    Returns:
        PoseStamped: A ROS message containing the updated pose setpoint based on keyboard inputs.
    """
    global current_x, current_y, current_z, current_yaw  # Access and modify global setpoint variables

    # Initialize command increments for position and orientation
    x_inc, y_inc, z_inc, yaw_inc = 0, 0, 0, 0
    scale = 0.1  # Scale factor for position increments (meters)
    yaw_scale = 0.1  # Scale factor for yaw increment (radians)

    # Get the current state of all keyboard keys
    keys = pygame.key.get_pressed()

    # Map specific keys to position and orientation commands
    if keys[pygame.K_w]:
        x_inc = 1  # Increase x-position (move forward)
    if keys[pygame.K_s]:
        x_inc = -1  # Decrease x-position (move backward)
    if keys[pygame.K_a]:
        y_inc = -1  # Decrease y-position (move left)
    if keys[pygame.K_d]:
        y_inc = 1  # Increase y-position (move right)
    if keys[pygame.K_q]:
        z_inc = 1  # Increase z-position (move upward)
    if keys[pygame.K_e]:
        z_inc = -1  # Decrease z-position (move downward)
    if keys[pygame.K_j]:
        yaw_inc = 1  # Increase yaw (rotate clockwise)
    if keys[pygame.K_l]:
        yaw_inc = -1  # Decrease yaw (rotate counter-clockwise)

    # Apply scaling to the increments to control the magnitude of adjustments
    x_inc, y_inc, z_inc = [k * scale for k in (x_inc, y_inc, z_inc)]  # Scale position increments
    yaw_inc *= yaw_scale  # Scale yaw increment

    # Update the current pose setpoint based on the scaled increments
    current_x += x_inc
    current_y += y_inc
    current_z += z_inc
    current_yaw += yaw_inc

    # Create a PoseStamped message to hold the updated setpoint
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()  # Timestamp the message with the current ROS time
    pose_msg.header.frame_id = "fossen_world"  # Reference frame for the pose

    # Set the desired position in the pose message
    pose_msg.pose.position.x = current_x
    pose_msg.pose.position.y = current_y
    pose_msg.pose.position.z = current_z

    # Convert Euler angles (roll, pitch, yaw) to a quaternion for orientation
    # In this script, only yaw is being adjusted; roll and pitch remain zero
    pose_msg.pose.orientation.x = 0.0  # No roll
    pose_msg.pose.orientation.y = 0.0  # No pitch
    pose_msg.pose.orientation.z = math.sin(current_yaw / 2.0)  # Quaternion z-component for yaw
    pose_msg.pose.orientation.w = math.cos(current_yaw / 2.0)  # Quaternion w-component for yaw

    # Convert quaternion to Euler angles for printing (optional)
    quaternion = (
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)  # Convert quaternion to Euler angles

    # Format and print the pose setpoint for debugging and visualization
    print(f"Pose Setpoint: Position - x: {pose_msg.pose.position.x:10.4f} y: {pose_msg.pose.position.y:10.4f} z: {pose_msg.pose.position.z:10.4f} "
          f"\nOrientation (quaternion) - x: {pose_msg.pose.orientation.x:10.4f} y: {pose_msg.pose.orientation.y:10.4f} z: {pose_msg.pose.orientation.z:10.4f} w: {pose_msg.pose.orientation.w:10.4f} "
          f"\nOrientation (Euler) - roll: {euler[0]:10.4f} pitch: {euler[1]:10.4f} yaw: {euler[2]:10.4f}")

    return pose_msg  # Return the populated PoseStamped message

if __name__ == '__main__':
    """
    Entry point for the Manual Pose Setpoint Generator script.

    This section initializes the ROS node, sets up the publisher for pose setpoints, and enters a loop
    to continuously capture keyboard inputs, generate pose messages, and publish them at the specified rate.
    It also handles Pygame events to allow graceful shutdown of the script.
    """
    rospy.init_node('manual_pose_setpoint_publisher')  # Initialize the ROS node with the name 'manual_pose_setpoint_publisher'

    # Publisher setup: publishes PoseStamped messages to the '/control/pose_setpoint' topic
    pub = rospy.Publisher('/control/pose_setpoint', PoseStamped, queue_size=10)

    hz = 20.0  # Define the publishing rate (20 Hz)
    rate = rospy.Rate(hz)  # Create a Rate object to maintain the loop at 20 Hz

    try:
        while not rospy.is_shutdown():  # Continue looping until ROS is shut down
            # Process Pygame events to detect window closure and other events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown('Quit event received')  # Gracefully shut down the ROS node

            pose_msg = get_keyboard_command()  # Generate the current pose setpoint based on keyboard inputs
            pub.publish(pose_msg)  # Publish the PoseStamped message to the setpoint topic
            rate.sleep()  # Sleep to maintain the loop rate

    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exceptions gracefully (e.g., when the node is shut down)
    finally:
        pygame.quit()  # Ensure that Pygame quits properly upon exiting the script
