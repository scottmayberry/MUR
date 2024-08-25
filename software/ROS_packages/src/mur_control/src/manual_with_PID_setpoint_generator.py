#!/usr/bin/env python

import rospy
import pygame
from geometry_msgs.msg import PoseStamped
import math
import tf

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((200, 200))  # Small window

# Initialize the current pose setpoint
current_x, current_y, current_z, current_yaw = 0.0, 0.0, 0.0, 0.0

def get_keyboard_command():
    global current_x, current_y, current_z, current_yaw

    # Initialize command increments
    x_inc, y_inc, z_inc, yaw_inc = 0, 0, 0, 0
    scale = 0.1  # Scale factor for position values
    yaw_scale = 0.1  # Scale factor for yaw

    # Get the set of keys pressed
    keys = pygame.key.get_pressed()

    # Map keys to commands
    if keys[pygame.K_w]:
        x_inc = 1
    if keys[pygame.K_s]:
        x_inc = -1
    if keys[pygame.K_a]:
        y_inc = -1
    if keys[pygame.K_d]:
        y_inc = 1
    if keys[pygame.K_q]:
        z_inc = 1
    if keys[pygame.K_e]:
        z_inc = -1
    if keys[pygame.K_j]:
        yaw_inc = 1
    if keys[pygame.K_l]:
        yaw_inc = -1

    # Apply scale
    x_inc, y_inc, z_inc = [k * scale for k in (x_inc, y_inc, z_inc)]
    yaw_inc *= yaw_scale

    # Update the current pose setpoint
    current_x += x_inc
    current_y += y_inc
    current_z += z_inc
    current_yaw += yaw_inc

    # Create a PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "fossen_world"
    pose_msg.pose.position.x = current_x
    pose_msg.pose.position.y = current_y
    pose_msg.pose.position.z = current_z
    pose_msg.pose.orientation.x = 0.0
    pose_msg.pose.orientation.y = 0.0
    pose_msg.pose.orientation.z = math.sin(current_yaw / 2.0)
    pose_msg.pose.orientation.w = math.cos(current_yaw / 2.0)

    # Convert quaternion to Euler angles for printing
    quaternion = (
        pose_msg.pose.orientation.x,
        pose_msg.pose.orientation.y,
        pose_msg.pose.orientation.z,
        pose_msg.pose.orientation.w
    )
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Format the output for aligned columns
    print(f"Pose Setpoint: Position - x: {pose_msg.pose.position.x:10.4f} y: {pose_msg.pose.position.y:10.4f} z: {pose_msg.pose.position.z:10.4f} "
          f"\nOrientation (quaternion) - x: {pose_msg.pose.orientation.x:10.4f} y: {pose_msg.pose.orientation.y:10.4f} z: {pose_msg.pose.orientation.z:10.4f} w: {pose_msg.pose.orientation.w:10.4f} "
          f"\nOrientation (Euler) - roll: {euler[0]:10.4f} pitch: {euler[1]:10.4f} yaw: {euler[2]:10.4f}")

    return pose_msg

if __name__ == '__main__':
    rospy.init_node('manual_pose_setpoint_publisher')

    pub = rospy.Publisher('/control/pose_setpoint', PoseStamped, queue_size=10)

    hz = 20.0
    rate = rospy.Rate(hz)  # 10Hz

    try:
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown('Quit event received')

            pose_msg = get_keyboard_command()
            pub.publish(pose_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
