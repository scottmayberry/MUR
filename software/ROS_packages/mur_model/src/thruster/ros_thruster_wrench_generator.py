#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import WrenchStamped

def generate_smooth_command(t):
    # Example: Generate a sinusoidal command for x, y, and z
    x = 1 * np.sin(t)
    y = 0 # 3 * np.sin(t + np.pi/3)
    z = 0 # 4 * np.sin(t + 2*np.pi/3)
    roll = 0 # 2 * np.sin(t + np.pi)
    pitch = 0
    yaw = 0 # 10 * np.sin(t + 4*np.pi/3)

    scale=2

    x, y, z, roll, pitch, yaw = [k*scale for k in (x,y,z,roll,pitch,yaw)]

    # Create a WrenchStamped message
    wrench_msg = WrenchStamped()
    wrench_msg.header.stamp = rospy.Time.now()
    wrench_msg.header.frame_id = "com"
    wrench_msg.wrench.force.x = x
    wrench_msg.wrench.force.y = y
    wrench_msg.wrench.force.z = z
    wrench_msg.wrench.torque.x = roll
    wrench_msg.wrench.torque.y = pitch  # Pitch is not used
    wrench_msg.wrench.torque.z = yaw

    return wrench_msg

if __name__ == '__main__':
    rospy.init_node('body_frame_command_tester')
    
    pub = rospy.Publisher('requested_thruster_wrench', WrenchStamped, queue_size=10)
    
    hz = 10.0
    rate = rospy.Rate(hz)  # 10Hz
    t = 0.0

    while not rospy.is_shutdown():
        wrench_msg = generate_smooth_command(t)
        pub.publish(wrench_msg)
        
        t += 1/hz  # Increment time by 0.1s for the next iteration
        rate.sleep()
