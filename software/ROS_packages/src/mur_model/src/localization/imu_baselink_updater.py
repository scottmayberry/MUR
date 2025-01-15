#!/usr/bin/env python
"""
imu_baselink_updater.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script processes IMU (Inertial Measurement Unit) data to update the transform between the 'fossen_base_link'
    and the 'world' frame in the Miniature Underwater Robot (MUR) model. It subscribes to an IMU topic, retrieves
    the current orientation of the robot, and broadcasts an updated transform that accounts for the IMU's measurements.
    This ensures that the robot's base link frame accurately reflects its orientation in the world frame based on
    sensor data.

    The script performs the following key functions:
        1. Initializes a ROS node for broadcasting transforms based on IMU data.
        2. Retrieves sensor configuration parameters from the ROS parameter server.
        3. Subscribes to the specified IMU topic to receive orientation data.
        4. Processes incoming IMU data to calculate the corrected orientation.
        5. Broadcasts the updated transform to maintain accurate spatial relationships within the robot's model.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model imu_baselink_updater.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - tf2_ros
    - tf_conversions
    - sensor_msgs.msg (Imu)
    - geometry_msgs.msg (TransformStamped, Quaternion)
    - tf.transformations
    - numpy

License:
    MIT License
"""

import rospy
import tf2_ros
import tf_conversions
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped, Quaternion
from tf.transformations import quaternion_multiply, quaternion_inverse, euler_from_quaternion
import tf.transformations as tf_transformations


class IMUToTFBroadcaster:
    """
    IMUToTFBroadcaster processes IMU data to update the transform between the 'fossen_base_link'
    and the 'world' frame based on sensor measurements.

    Attributes:
        tf_buffer (tf2_ros.Buffer): Buffer to store and retrieve transforms.
        tf_listener (tf2_ros.TransformListener): Listener to receive transform updates.
        tf_broadcaster (tf2_ros.TransformBroadcaster): Broadcaster to send updated transforms.
        imu_sensor_topic (str): The ROS topic name for the IMU data.
        model_info (dict): Configuration information for the robot's model, including sensor details.
    """
    def __init__(self):
        """
        Initializes the IMUToTFBroadcaster by setting up ROS parameters, TF2 listeners and broadcasters,
        and subscribing to the IMU topic.
        """
        rospy.init_node('imu_to_tf_broadcaster')  # Initialize the ROS node with a unique name

        # Fetch parameters from the sensors namespace on the ROS parameter server
        sensors_config = rospy.get_param('/sensor_info/sensors')

        imu_of_interest = 'BNO055'  # Specify the IMU model to use
        instance_module = 'mur_battery_fuselage'  # Specify the module associated with the IMU

        # Retrieve all configurations related to the specified IMU
        sensor_data_all = sensors_config[imu_of_interest]
        self.sensor_topic_base = sensor_data_all['topic']  # Base topic name for the IMU
        self.sensor_topic_type_string = sensor_data_all['type']  # Data type for the IMU topic

        # Construct the full IMU topic name based on module and base topic
        self.imu_sensor_topic = f"/sensors/{instance_module}/{self.sensor_topic_base}"

        # Initialize TF2 buffer and listener for handling coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Subscribe to the IMU topic to receive orientation data
        rospy.Subscriber(self.imu_sensor_topic, Imu, self.imu_callback)

    # The following imu_callback method is temporarily commented out and can be used for alternative processing
    # def imu_callback(self, imu_msg):
    #     try:
    #         # Step 1: Read the IMU orientation (global measurement in 'world' frame)
    #         imu_orientation = imu_msg.orientation
    #         imu_quat = [imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w]
    #
    #         # Step 2: Get the current transform from world to imu_frame
    #         imu_frame = imu_msg.header.frame_id
    #         try:
    #             world_to_imu = self.tf_buffer.lookup_transform('world', imu_frame, rospy.Time(0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #             rospy.logerr("TF lookup failed for world to imu_frame: {}".format(e))
    #             return
    #
    #         # Step 3: Get the current transform from fossen_base_link to imu_frame
    #         try:
    #             base_link_to_imu = self.tf_buffer.lookup_transform('fossen_base_link', imu_frame, rospy.Time(0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #             rospy.logerr("TF lookup failed for fossen_base_link to imu_frame: {}".format(e))
    #             return
    #
    #         # Step 4: Calculate the relative orientation between base_link and imu_frame
    #         base_link_quat = [
    #             base_link_to_imu.transform.rotation.x,
    #             base_link_to_imu.transform.rotation.y,
    #             base_link_to_imu.transform.rotation.z,
    #             base_link_to_imu.transform.rotation.w
    #         ]
    #         relative_orientation = tf_transformations.quaternion_inverse(base_link_quat)
    #
    #         # Step 5: Combine the IMU orientation with the relative orientation
    #         corrected_orientation = tf_transformations.quaternion_multiply(imu_quat, relative_orientation)
    #
    #         # Step 6: Get the current transform from world to fossen_base_link
    #         try:
    #             world_to_base_link = self.tf_buffer.lookup_transform('world', 'fossen_base_link', rospy.Time(0))
    #         except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #             rospy.logerr("TF lookup failed for world to fossen_base_link: {}".format(e))
    #             return
    #
    #         # Step 7: Create a new transform for predicted_base_link with the corrected orientation
    #         t = TransformStamped()
    #         t.header.stamp = rospy.Time.now()
    #         t.header.frame_id = 'world'
    #         t.child_frame_id = 'predicted_fossen_base_link'
    #
    #         # Use the existing translation from world to base_link
    #         t.transform.translation.x = world_to_base_link.transform.translation.x
    #         t.transform.translation.y = world_to_base_link.transform.translation.y
    #         t.transform.translation.z = world_to_base_link.transform.translation.z
    #         t.transform.rotation.x = corrected_orientation[0]
    #         t.transform.rotation.y = corrected_orientation[1]
    #         t.transform.rotation.z = corrected_orientation[2]
    #         t.transform.rotation.w = corrected_orientation[3]
    #
    #         # Broadcast the new transform
    #         self.tf_broadcaster.sendTransform(t)
    #
    #     except Exception as e:
    #         rospy.logerr("Unexpected error: {}".format(e))

    def imu_callback(self, imu_msg):
        """
        Callback function for processing incoming IMU messages.

        This function receives IMU data, retrieves the necessary transforms, calculates the corrected orientation
        of the robot's base link, and broadcasts the updated transform to maintain accurate spatial relationships
        within the robot's model.

        Args:
            imu_msg (Imu): The incoming IMU message containing orientation data.
        """
        try:
            # Step 1: Read the IMU orientation (global measurement in 'world' frame)
            imu_orientation = imu_msg.orientation
            imu_quat = [imu_orientation.x, imu_orientation.y, imu_orientation.z, imu_orientation.w]

            # Step 2: Get the current transform from world to imu_frame
            imu_frame = imu_msg.header.frame_id
            try:
                world_to_imu = self.tf_buffer.lookup_transform('world', imu_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF lookup failed for world to imu_frame: {}".format(e))
                return

            # Step 3: Get the current transform from fossen_base_link to imu_frame
            try:
                base_link_to_imu = self.tf_buffer.lookup_transform('fossen_base_link', imu_frame, rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF lookup failed for fossen_base_link to imu_frame: {}".format(e))
                return

            # Step 4: Calculate the relative orientation between base_link and imu_frame
            base_link_quat = [
                base_link_to_imu.transform.rotation.x,
                base_link_to_imu.transform.rotation.y,
                base_link_to_imu.transform.rotation.z,
                base_link_to_imu.transform.rotation.w
            ]
            relative_orientation = tf_transformations.quaternion_inverse(base_link_quat)

            # Step 5: Combine the IMU orientation with the relative orientation
            corrected_orientation = tf_transformations.quaternion_multiply(imu_quat, relative_orientation)

            # Step 6: Get the current transform from world to fossen_base_link
            try:
                world_to_base_link = self.tf_buffer.lookup_transform('world', 'fossen_base_link', rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF lookup failed for world to fossen_base_link: {}".format(e))
                return

            # Step 7: Update the transform for fossen_base_link with the corrected orientation
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = 'world'
            t.child_frame_id = 'fossen_base_link'

            # Use the existing translation from world to base_link
            t.transform.translation.x = world_to_base_link.transform.translation.x
            t.transform.translation.y = world_to_base_link.transform.translation.y
            t.transform.translation.z = world_to_base_link.transform.translation.z
            t.transform.rotation.x = corrected_orientation[0]
            t.transform.rotation.y = corrected_orientation[1]
            t.transform.rotation.z = corrected_orientation[2]
            t.transform.rotation.w = corrected_orientation[3]

            # Broadcast the updated transform
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            rospy.logerr("Unexpected error: {}".format(e))

    def run(self):
        """
        Runs the ROS node, keeping it alive to process incoming IMU messages.

        This function enters a loop that keeps the node active until it is shut down,
        allowing continuous processing and broadcasting of transforms based on IMU data.
        """
        rospy.spin()  # Keep the node running and responsive to callbacks


if __name__ == '__main__':
    """
    Entry point for the IMU to Baselink Updater script.

    This section creates an instance of the IMUToTFBroadcaster class and starts the ROS node,
    enabling the processing and broadcasting of transforms based on incoming IMU data.
    """
    imu_to_tf_broadcaster = IMUToTFBroadcaster()  # Instantiate the broadcaster
    imu_to_tf_broadcaster.run()  # Start the ROS node
