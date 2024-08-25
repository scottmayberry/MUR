#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler

def generate_smooth_pose(t):
    # Generate a sinusoidal command for x, y, z, roll, pitch, and yaw
    x = y = z = pitch = yaw = roll = 0
    # roll = np.pi
    # x = 10 * np.sin(t)
    # y = 3 * np.sin(t + np.pi/3)
    # z = 4 * np.sin(t + 2*np.pi/3)
    # roll = 2 * np.sin(t/10 + np.pi) + roll
    # pitch = 0
    # yaw = np.pi # 2 * np.sin(t/10 + 4*np.pi/3)

    # print(yaw)


    # Convert roll, pitch, yaw to a quaternion
    quat = quaternion_from_euler(roll, pitch, yaw)

    # Create a PoseStamped message
    pose_msg = PoseStamped()
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.header.frame_id = "world"
    pose_msg.pose.position.x = x
    pose_msg.pose.position.y = y
    pose_msg.pose.position.z = z
    pose_msg.pose.orientation = Quaternion(*quat)

    return pose_msg

if __name__ == '__main__':
    rospy.init_node('pose_setpoint_generator')
    
    pub = rospy.Publisher('pose_setpoint', PoseStamped, queue_size=10)
    
    hz = 10.0
    rate = rospy.Rate(hz)  # 10Hz
    t = 0.0

    while not rospy.is_shutdown():
        pose_msg = generate_smooth_pose(t)
        pub.publish(pose_msg)
        
        t += 1/hz  # Increment time by 0.1s for the next iteration
        rate.sleep()
