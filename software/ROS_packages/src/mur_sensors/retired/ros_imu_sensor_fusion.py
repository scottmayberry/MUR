#!/usr/bin/env python
import socket
import sys
import time
import yaml
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import json
import numpy
import imufusion
import tf2_ros
import tf2_geometry_msgs


def mpu6500_data_callback(data):
    global mpu6500_data
    mpu6500_data.append(data)

def lis2mdl_data_callback(data):
    global lis2mdl_data
    lis2mdl_data.append(data)


def imu_fusion(sample_rate=50):
    global mpu6500_data, lis2mdl_data

    # Instantiate algorithms
    offset = imufusion.Offset(sample_rate)
    ahrs = imufusion.Ahrs()

    ahrs.settings = imufusion.Settings(0.5,  # gain
                                    10,  # acceleration rejection
                                    20,  # magnetic rejection
                                    5 * sample_rate)  # rejection timeout = 5 seconds

    gyroscope = numpy.empty((3))
    accelerometer = numpy.empty((3))
    magnetometer = numpy.empty((3))
    deltaTime = numpy.float64(1/sample_rate)
    euler = numpy.empty((3))

    rospy.init_node('imu_fusion', anonymous=True)

    # Initialize TF2 Buffer and Listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Get the transform from LIS2MDL to MPU6500
    transform = None
    while transform is None:
        try:
            transform = tf_buffer.lookup_transform('MPU6500', 'LIS2MDL', rospy.Time(), rospy.Duration(5.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Failed to get initial transform from LIS2MDL to MPU6500. Retrying in 5 seconds")
            print("Failed to get initial transform from LIS2MDL to MPU6500. Retrying in 5 seconds")
            time.sleep(5)
            # sys.exit()

    imu_covariance = [0]*9
    pub = rospy.Publisher('imu_fused', Imu, queue_size=2)
    rospy.Subscriber('MPU6500', Twist, mpu6500_data_callback)
    rospy.Subscriber('LIS2MDL', Vector3, lis2mdl_data_callback)
    rate = rospy.Rate(sample_rate)
    while not rospy.is_shutdown():
        if(len(mpu6500_data) > 0 and len(lis2mdl_data) > 0):
            mpu6500_sample = mpu6500_data.pop()
            lis2mdl_sample = lis2mdl_data.pop()

            # handle the case if too much data is coming in
            if(len(mpu6500_data) > 1):
                mpu6500_data = mpu6500_data[:1]
            if(len(lis2mdl_data) > 1):
                lis2mdl_data = lis2mdl_data[:1]

            # print("Connections: ", pub.get_num_connections())
            if pub.get_num_connections() > 0:
                gyroscope[0] = mpu6500_sample.angular.x
                gyroscope[1] = mpu6500_sample.angular.y
                gyroscope[2] = mpu6500_sample.angular.z

                accelerometer[0] = mpu6500_sample.linear.x
                accelerometer[1] = mpu6500_sample.linear.y
                accelerometer[2] = mpu6500_sample.linear.z

                # Transform the magnetometer readings to the MPU6500 frame
                magnetometer_msg = Vector3Stamped()
                magnetometer_msg.vector = lis2mdl_sample
                transformed_magnetometer_msg = tf2_geometry_msgs.do_transform_vector3(magnetometer_msg, transform)
                lis2mdl_sample = transformed_magnetometer_msg.vector

                magnetometer[0] = lis2mdl_sample.x
                magnetometer[1] = lis2mdl_sample.y
                magnetometer[2] = lis2mdl_sample.z
                gyroscope = offset.update(gyroscope)

                ahrs.update(gyroscope, accelerometer, magnetometer, deltaTime)
                
                orientation = Quaternion(ahrs.quaternion.x,ahrs.quaternion.y,ahrs.quaternion.z,ahrs.quaternion.w)
                ang_vel = Vector3(gyroscope[0], gyroscope[1], gyroscope[2])
                lin_acc = Vector3(accelerometer[0], accelerometer[1], accelerometer[2])
                header = Header()
                header.stamp = rospy.Time.now()
                data_to_pub = Imu(header, orientation, imu_covariance, ang_vel, imu_covariance, lin_acc, imu_covariance)
                pub.publish(data_to_pub)
        rate.sleep()
    
mpu6500_data = []
lis2mdl_data = []

if __name__ == "__main__":
    try:
        sample_rate = int(rospy.get_param("/sensor_info/sensor_fusion_info/sample_rate"))
        print("Sample rate from config: ", sample_rate)
        imu_fusion(sample_rate)
    except rospy.ROSInterruptException:
        pass
