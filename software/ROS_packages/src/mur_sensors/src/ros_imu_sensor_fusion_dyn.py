#!/usr/bin/env python
import threading
import rospy
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header
from geometry_msgs.msg import Twist, Vector3, Vector3Stamped, Quaternion
from sensor_msgs.msg import Imu
import numpy as np
import imufusion
import time

class IMUFusion:
    def __init__(self, fusion_config):
        self.sample_rate = fusion_config['sample_rate']
        self.accel_topic = f"/sensors/{fusion_config['accelerometer_module']}/{fusion_config['accelerometer']}"
        self.gyro_topic = f"/sensors/{fusion_config['gyroscope_module']}/{fusion_config['gyroscope']}"
        self.mag_topic = f"/sensors/{fusion_config['magnetometer_module']}/{fusion_config['magnetometer']}"
        self.fusion_id = fusion_config['id']

        self.accel_frame = f"{fusion_config['accelerometer']}_{fusion_config['accelerometer_module']}"
        self.gyro_frame = f"{fusion_config['gyroscope']}_{fusion_config['gyroscope_module']}"
        self.mag_frame = f"{fusion_config['magnetometer']}_{fusion_config['magnetometer_module']}"

        self.imu_covariance = [0] * 9

        # Instantiate algorithms
        self.offset = imufusion.Offset(self.sample_rate)
        self.ahrs = imufusion.Ahrs()

        self.ahrs.settings = imufusion.Settings(0.5,  # gain
                                                10,  # acceleration rejection
                                                20,  # magnetic rejection
                                                5 * self.sample_rate)  # rejection timeout = 5 seconds

        self.gyroscope = np.empty((3))
        self.accelerometer = np.empty((3))
        self.magnetometer = np.empty((3))
        self.deltaTime = np.float64(1 / self.sample_rate)

        self.accel_data = []
        self.gyro_data = []
        self.mag_data = []

        self.pub = rospy.Publisher(f'imu_fused_{self.fusion_id}', Imu, queue_size=2)

        # Determine the message types for accelerometer and gyroscope
        if fusion_config['accelerometer'] == fusion_config['gyroscope']:
            rospy.Subscriber(self.accel_topic, Twist, self.twist_data_callback)
        else:
            # Check if the topics are using Vector3 or Twist
            accel_type = rospy.get_param(f"/sensor_info/sensors/{fusion_config['accelerometer']}/type")
            gyro_type = rospy.get_param(f"/sensor_info/sensors/{fusion_config['gyroscope']}/type")
            if accel_type == "Vector3":
                rospy.Subscriber(self.accel_topic, Vector3, self.accel_data_callback)
            elif accel_type == "Twist":
                rospy.Subscriber(self.accel_topic, Twist, self.accel_twist_data_callback)

            if gyro_type == "Vector3":
                rospy.Subscriber(self.gyro_topic, Vector3, self.gyro_data_callback)
            elif gyro_type == "Twist":
                rospy.Subscriber(self.gyro_topic, Twist, self.gyro_twist_data_callback)

        rospy.Subscriber(self.mag_topic, Vector3, self.mag_data_callback)

        # Initialize TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = None

    def twist_data_callback(self, data):
        self.accel_data.append(data.linear)
        self.gyro_data.append(data.angular)

    def accel_data_callback(self, data):
        self.accel_data.append(data)

    def accel_twist_data_callback(self, data):
        self.accel_data.append(data.linear)

    def gyro_data_callback(self, data):
        self.gyro_data.append(data)

    def gyro_twist_data_callback(self, data):
        self.gyro_data.append(data.angular)

    def mag_data_callback(self, data):
        self.mag_data.append(data)

    def run(self):
        rate = rospy.Rate(self.sample_rate)

        while not rospy.is_shutdown():
            if self.transform is None:
                try:
                    self.transform = self.tf_buffer.lookup_transform(self.accel_frame, self.mag_frame, rospy.Time(), rospy.Duration(5.0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rospy.logwarn(f"Failed to get initial transform from {self.mag_frame} to {self.accel_frame}. Retrying in 5 seconds")
                    time.sleep(5)
                    continue

            if self.accel_data and self.gyro_data and self.mag_data:
                accel_sample = self.accel_data.pop()
                gyro_sample = self.gyro_data.pop()
                mag_sample = self.mag_data.pop()

                # handle the case if too much data is coming in
                if len(self.accel_data) > 1:
                    self.accel_data = self.accel_data[:1]
                if len(self.gyro_data) > 1:
                    self.gyro_data = self.gyro_data[:1]
                if len(self.mag_data) > 1:
                    self.mag_data = self.mag_data[:1]

                if self.pub.get_num_connections() > 0:
                    self.gyroscope[0] = gyro_sample.x
                    self.gyroscope[1] = gyro_sample.y
                    self.gyroscope[2] = gyro_sample.z

                    self.accelerometer[0] = accel_sample.x
                    self.accelerometer[1] = accel_sample.y
                    self.accelerometer[2] = accel_sample.z

                    # Transform the magnetometer readings to the accelerometer frame
                    mag_msg = Vector3Stamped()
                    mag_msg.vector = mag_sample
                    transformed_mag_msg = tf2_geometry_msgs.do_transform_vector3(mag_msg, self.transform)
                    mag_sample = transformed_mag_msg.vector

                    self.magnetometer[0] = mag_sample.x
                    self.magnetometer[1] = mag_sample.y
                    self.magnetometer[2] = mag_sample.z

                    self.gyroscope = self.offset.update(self.gyroscope)

                    self.ahrs.update(self.gyroscope, self.accelerometer, self.magnetometer, self.deltaTime)
                    
                    orientation = Quaternion(self.ahrs.quaternion.x, self.ahrs.quaternion.y, self.ahrs.quaternion.z, self.ahrs.quaternion.w)
                    ang_vel = Vector3(self.gyroscope[0], self.gyroscope[1], self.gyroscope[2])
                    lin_acc = Vector3(self.accelerometer[0], self.accelerometer[1], self.accelerometer[2])
                    header = Header()
                    header.frame_id = "fossen_base_link"  # Set the frame ID to base_link
                    header.stamp = rospy.Time.now()
                    data_to_pub = Imu(header, orientation, self.imu_covariance, ang_vel, self.imu_covariance, lin_acc, self.imu_covariance)
                    self.pub.publish(data_to_pub)
            rate.sleep()

def run_fusion(fusion):
    fusion.run()

if __name__ == "__main__":
    rospy.init_node('imu_fusion', anonymous=True)
    imu_fusion_configs = rospy.get_param('/sensor_info/imu_sensor_fusion')
    fusion_objects = []

    for idx, config in enumerate(imu_fusion_configs):
        config['id'] = idx
        fusion = IMUFusion(config)
        fusion_objects.append(fusion)

    threads = []
    for fusion in fusion_objects:
        thread = threading.Thread(target=run_fusion, args=(fusion,))
        thread.start()
        threads.append(thread)

    for thread in threads:
        thread.join()
