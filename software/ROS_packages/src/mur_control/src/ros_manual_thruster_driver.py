#!/usr/bin/env python

import rospy
import pygame
from geometry_msgs.msg import WrenchStamped

# Initialize Pygame
pygame.init()
screen = pygame.display.set_mode((200, 200))  # Small window

def get_keyboard_command():
    # Initialize command values
    x, y, z, roll, pitch, yaw = 0, 0, 0, 0, 0, 0
    scale = 11.0  # Scale factor for command values

    # Get the set of keys pressed
    keys = pygame.key.get_pressed()

    # Map keys to commands
    if keys[pygame.K_w]:
        x = 1
    if keys[pygame.K_s]:
        x = -1
    if keys[pygame.K_a]:
        y = -1
    if keys[pygame.K_d]:
        y = 1
    if keys[pygame.K_q]:
        z = 1
    if keys[pygame.K_e]:
        z = -1
    if keys[pygame.K_i]:
        roll = 1
    if keys[pygame.K_k]:
        roll = -1
    if keys[pygame.K_j]:
        yaw = 1
    if keys[pygame.K_l]:
        yaw = -1

    # Apply scale
    x, y, z, roll, pitch, yaw = [k * scale for k in (x, y, z, roll, pitch, yaw)]
    roll /= 8
    yaw /= 8

    # Create a WrenchStamped message
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()
    wrench_msg.header.frame_id = "fossen_com"
    wrench_msg.wrench.force.x = x
    wrench_msg.wrench.force.y = y
    wrench_msg.wrench.force.z = z
    wrench_msg.wrench.torque.x = roll
    wrench_msg.wrench.torque.y = pitch  # Pitch is not used
    wrench_msg.wrench.torque.z = yaw

    # # Format the output for aligned columns
    # print(f"Manual Thruster Wrench: Force - x: {wrench_msg.wrench.force.x:10.4f} y: {wrench_msg.wrench.force.y:10.4f} z: {wrench_msg.wrench.force.z:10.4f} "
    #       f"Torque - x: {wrench_msg.wrench.torque.x:10.4f} y: {wrench_msg.wrench.torque.y:10.4f} z: {wrench_msg.wrench.torque.z:10.4f}")


    return wrench_msg

if __name__ == '__main__':
    rospy.init_node('body_frame_command_tester')

    pub = rospy.Publisher('/model/requested_thruster_wrench', WrenchStamped, queue_size=10)

    try:
        hz = rospy.get_param('/control_info/manual_thruster_update_hz')
    except:
        hz = 10.0
    rate = rospy.Rate(hz)  # 10Hz

    try:
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown('Quit event received')

            wrench_msg = get_keyboard_command()
            pub.publish(wrench_msg)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
