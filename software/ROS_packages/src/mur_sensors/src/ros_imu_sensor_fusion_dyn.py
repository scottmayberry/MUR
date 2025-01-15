#!/usr/bin/env python
"""
ros_imu_sensor_fusion_dyn.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script handles the fusion of Inertial Measurement Unit (IMU) data for the Miniature Underwater Robot (MUR).
    It subscribes to accelerometer, gyroscope, and magnetometer sensor topics, processes the incoming data using
    sensor fusion algorithms, and publishes the fused IMU data as ROS topics. The fusion process improves the
    accuracy and reliability of orientation and motion tracking for the robot.

Usage:
    Ensure that the ROS master is running and the necessary sensor topics are being published before executing
    this script. The script initializes ROS nodes, sets up subscribers for sensor data, and continuously performs
    sensor fusion in a multi-threaded environment.

Dependencies:
    - rospy
    - tf2_ros
    - tf2_geometry_msgs
    - std_msgs.msg (Header)
    - geometry_msgs.msg (Twist, Vector3, Vector3Stamped, Quaternion)
    - sensor_msgs.msg (Imu)
    - numpy
    - imufusion (custom module)
    - time

License: MIT License
"""

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
    """
    IMUFusion class handles the fusion of accelerometer, gyroscope, and magnetometer data to produce
    a single fused IMU message. It subscribes to the respective sensor topics, processes the data using
    sensor fusion algorithms, and publishes the fused IMU data.
    """
    def __init__(self, fusion_config):
        """
        Initializes the IMUFusion instance with the provided configuration.

        Args:
            fusion_config (dict): Configuration parameters for sensor fusion, including sensor topics,
                                  sample rate, and module identifiers.
        """
        # Set the sample rate for sensor fusion
        self.sample_rate = fusion_config['sample_rate']
        
        # Construct ROS topic names for accelerometer, gyroscope, and magnetometer based on module
        self.accel_topic = f"/sensors/{fusion_config['accelerometer_module']}/{fusion_config['accelerometer']}"
        self.gyro_topic = f"/sensors/{fusion_config['gyroscope_module']}/{fusion_config['gyroscope']}"
        self.mag_topic = f"/sensors/{fusion_config['magnetometer_module']}/{fusion_config['magnetometer']}"
        
        # Unique identifier for the fusion instance
        self.fusion_id = fusion_config['id']

        # Define frame IDs for each sensor based on sensor type and module
        self.accel_frame = f"{fusion_config['accelerometer']}_{fusion_config['accelerometer_module']}"
        self.gyro_frame = f"{fusion_config['gyroscope']}_{fusion_config['gyroscope_module']}"
        self.mag_frame = f"{fusion_config['magnetometer']}_{fusion_config['magnetometer_module']}"

        # Initialize IMU covariance matrix (identity matrix scaled by zero)
        self.imu_covariance = [0] * 9

        # Instantiate sensor fusion algorithms
        self.offset = imufusion.Offset(self.sample_rate)  # Handles sensor bias offset
        self.ahrs = imufusion.Ahrs()  # Attitude and Heading Reference System (AHRS) for orientation

        # Configure AHRS settings: gain, acceleration rejection, magnetic rejection, and rejection timeout
        self.ahrs.settings = imufusion.Settings(
            0.5,                # gain
            10,                 # acceleration rejection threshold
            20,                 # magnetic rejection threshold
            5 * self.sample_rate  # rejection timeout in seconds
        )

        # Initialize arrays to store gyroscope, accelerometer, and magnetometer data
        self.gyroscope = np.empty((3))
        self.accelerometer = np.empty((3))
        self.magnetometer = np.empty((3))
        self.deltaTime = np.float64(1 / self.sample_rate)  # Time interval between samples

        # Lists to store incoming sensor data
        self.accel_data = []
        self.gyro_data = []
        self.mag_data = []

        # Publisher for the fused IMU data
        self.pub = rospy.Publisher(f'imu_fused_{self.fusion_id}', Imu, queue_size=2)

        # Subscribe to sensor topics based on sensor type and configuration
        if fusion_config['accelerometer'] == fusion_config['gyroscope']:
            # If accelerometer and gyroscope are the same sensor type, subscribe to Twist messages
            rospy.Subscriber(self.accel_topic, Twist, self.twist_data_callback)
        else:
            # Determine the message type for accelerometer and subscribe accordingly
            accel_type = rospy.get_param(f"/sensor_info/sensors/{fusion_config['accelerometer']}/type")
            gyro_type = rospy.get_param(f"/sensor_info/sensors/{fusion_config['gyroscope']}/type")
            
            if accel_type == "Vector3":
                rospy.Subscriber(self.accel_topic, Vector3, self.accel_data_callback)
            elif accel_type == "Twist":
                rospy.Subscriber(self.accel_topic, Twist, self.accel_twist_data_callback)

            # Determine the message type for gyroscope and subscribe accordingly
            if gyro_type == "Vector3":
                rospy.Subscriber(self.gyro_topic, Vector3, self.gyro_data_callback)
            elif gyro_type == "Twist":
                rospy.Subscriber(self.gyro_topic, Twist, self.gyro_twist_data_callback)

        # Subscribe to the magnetometer topic (assumed to always use Vector3 messages)
        rospy.Subscriber(self.mag_topic, Vector3, self.mag_data_callback)

        # Initialize TF2 buffer and listener for handling coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.transform = None  # Placeholder for the transformation between frames

    def twist_data_callback(self, data):
        """
        Callback function for Twist messages from combined accelerometer and gyroscope sensors.

        Args:
            data (Twist): Incoming Twist message containing linear (accelerometer) and angular (gyroscope) data.
        """
        self.accel_data.append(data.linear)
        self.gyro_data.append(data.angular)

    def accel_data_callback(self, data):
        """
        Callback function for Vector3 messages from the accelerometer.

        Args:
            data (Vector3): Incoming Vector3 message containing accelerometer data.
        """
        self.accel_data.append(data)

    def accel_twist_data_callback(self, data):
        """
        Callback function for Twist messages from the accelerometer.

        Args:
            data (Twist): Incoming Twist message containing linear (accelerometer) data.
        """
        self.accel_data.append(data.linear)

    def gyro_data_callback(self, data):
        """
        Callback function for Vector3 messages from the gyroscope.

        Args:
            data (Vector3): Incoming Vector3 message containing gyroscope data.
        """
        self.gyro_data.append(data)

    def gyro_twist_data_callback(self, data):
        """
        Callback function for Twist messages from the gyroscope.

        Args:
            data (Twist): Incoming Twist message containing angular (gyroscope) data.
        """
        self.gyro_data.append(data.angular)

    def mag_data_callback(self, data):
        """
        Callback function for Vector3 messages from the magnetometer.

        Args:
            data (Vector3): Incoming Vector3 message containing magnetometer data.
        """
        self.mag_data.append(data)

    def run(self):
        """
        Main loop that performs sensor fusion at the specified sample rate.

        The loop performs the following steps:
            1. Waits for the necessary coordinate transform between accelerometer and magnetometer frames.
            2. Processes incoming sensor data when available.
            3. Updates sensor fusion algorithms with the latest data.
            4. Publishes the fused IMU data as a ROS Imu message.
        """
        rate = rospy.Rate(self.sample_rate)  # Set the loop rate based on sample rate

        while not rospy.is_shutdown():
            # Ensure that the necessary transform is available
            if self.transform is None:
                try:
                    # Lookup the transform between accelerometer and magnetometer frames
                    self.transform = self.tf_buffer.lookup_transform(
                        self.accel_frame,
                        self.mag_frame,
                        rospy.Time(),
                        rospy.Duration(5.0)  # Timeout duration
                    )
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    # Log a warning if the transform lookup fails and retry after 5 seconds
                    rospy.logwarn(f"Failed to get initial transform from {self.mag_frame} to {self.accel_frame}. Retrying in 5 seconds")
                    time.sleep(5)
                    continue  # Skip to the next loop iteration

            # Check if there is available sensor data to process
            if self.accel_data and self.gyro_data and self.mag_data:
                # Retrieve the latest sensor data
                accel_sample = self.accel_data.pop()
                gyro_sample = self.gyro_data.pop()
                mag_sample = self.mag_data.pop()

                # Handle cases where multiple data points have been received to prevent backlog
                if len(self.accel_data) > 1:
                    self.accel_data = self.accel_data[:1]
                if len(self.gyro_data) > 1:
                    self.gyro_data = self.gyro_data[:1]
                if len(self.mag_data) > 1:
                    self.mag_data = self.mag_data[:1]

                # Proceed only if there are active subscribers to the fused IMU topic
                if self.pub.get_num_connections() > 0:
                    # Populate gyroscope data array
                    self.gyroscope[0] = gyro_sample.x
                    self.gyroscope[1] = gyro_sample.y
                    self.gyroscope[2] = gyro_sample.z

                    # Populate accelerometer data array
                    self.accelerometer[0] = accel_sample.x
                    self.accelerometer[1] = accel_sample.y
                    self.accelerometer[2] = accel_sample.z

                    # Transform the magnetometer readings to the accelerometer frame using TF2
                    mag_msg = Vector3Stamped()
                    mag_msg.vector = mag_sample
                    transformed_mag_msg = tf2_geometry_msgs.do_transform_vector3(mag_msg, self.transform)
                    mag_sample = transformed_mag_msg.vector

                    # Populate magnetometer data array
                    self.magnetometer[0] = mag_sample.x
                    self.magnetometer[1] = mag_sample.y
                    self.magnetometer[2] = mag_sample.z

                    # Update gyroscope data with offset calibration
                    self.gyroscope = self.offset.update(self.gyroscope)

                    # Update AHRS algorithm with the latest sensor data
                    self.ahrs.update(
                        self.gyroscope,
                        self.accelerometer,
                        self.magnetometer,
                        self.deltaTime
                    )
                    
                    # Create Quaternion message from AHRS output
                    orientation = Quaternion(
                        self.ahrs.quaternion.x,
                        self.ahrs.quaternion.y,
                        self.ahrs.quaternion.z,
                        self.ahrs.quaternion.w
                    )
                    
                    # Create Vector3 messages for angular velocity and linear acceleration
                    ang_vel = Vector3(
                        self.gyroscope[0],
                        self.gyroscope[1],
                        self.gyroscope[2]
                    )
                    lin_acc = Vector3(
                        self.accelerometer[0],
                        self.accelerometer[1],
                        self.accelerometer[2]
                    )
                    
                    # Create and populate the header for the Imu message
                    header = Header()
                    header.frame_id = "fossen_base_link"  # Reference frame ID
                    header.stamp = rospy.Time.now()  # Current timestamp
                    
                    # Construct the Imu message with the fused data
                    data_to_pub = Imu(
                        header,
                        orientation,
                        self.imu_covariance,  # Orientation covariance
                        ang_vel,
                        self.imu_covariance,  # Angular velocity covariance
                        lin_acc,
                        self.imu_covariance   # Linear acceleration covariance
                    )
                    
                    # Publish the fused Imu message to the ROS topic
                    self.pub.publish(data_to_pub)
            
            # Sleep to maintain the loop rate
            rate.sleep()

def run_fusion(fusion):
    """
    Function to run the sensor fusion process for a given IMUFusion instance.

    Args:
        fusion (IMUFusion): An instance of the IMUFusion class.
    """
    fusion.run()

if __name__ == "__main__":
    """
    Entry point for the IMU fusion script.

    Initializes the ROS node, retrieves IMU fusion configurations, creates IMUFusion instances,
    and starts separate threads for each fusion process to run concurrently.
    """
    rospy.init_node('imu_fusion', anonymous=True)  # Initialize the ROS node named 'imu_fusion'
    
    # Retrieve IMU fusion configurations from ROS parameters
    imu_fusion_configs = rospy.get_param('/sensor_info/imu_sensor_fusion')
    fusion_objects = []  # List to hold IMUFusion instances

    # Create IMUFusion instances based on the retrieved configurations
    for idx, config in enumerate(imu_fusion_configs):
        config['id'] = idx  # Assign a unique ID to each fusion instance
        fusion = IMUFusion(config)
        fusion_objects.append(fusion)

    threads = []  # List to hold threading.Thread objects

    # Start a separate thread for each IMUFusion instance to run concurrently
    for fusion in fusion_objects:
        thread = threading.Thread(target=run_fusion, args=(fusion,))
        thread.start()
        threads.append(thread)

    # Wait for all threads to complete (they run indefinitely until ROS is shut down)
    for thread in threads:
        thread.join()
