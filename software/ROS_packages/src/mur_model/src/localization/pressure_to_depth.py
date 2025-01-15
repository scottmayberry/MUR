#!/usr/bin/env python
"""
pressure_to_depth.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script converts pressure measurements from MS5837 pressure sensors to depth estimations for the Miniature Underwater Robot (MUR).
    It subscribes to pressure topics, processes the incoming pressure data to calculate depth based on the sensor's calibration and environmental parameters,
    and updates the robot's transform in the TF tree to reflect the new depth information.

    The script performs the following key functions:
        1. Initializes a ROS node for converting pressure measurements to depth.
        2. Retrieves sensor and environmental configuration parameters from the ROS parameter server.
        3. Subscribes to specified pressure sensor topics to receive pressure data.
        4. Processes incoming pressure data to calculate depth, considering zero-offset calibration.
        5. Updates the robot's transform in the TF tree to reflect the calculated depth.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model pressure_to_depth.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - tf2_ros
    - tf_conversions
    - sensor_msgs.msg (FluidPressure)
    - geometry_msgs.msg (TransformStamped, Quaternion)
    - tf.transformations
    - numpy

License:
    MIT License
"""

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import tf
import time
from mur.msg import DepthSensor
from queue import Queue
import tf2_ros
import tf.transformations as tf_transformations


def find_topics_ending_with(suffix):
    """
    Finds all currently published ROS topics that end with the specified suffix.

    Args:
        suffix (str): The suffix to match at the end of topic names.

    Returns:
        list: A list of topic names that end with the given suffix.
    """
    # Get a list of all currently published topics
    published_topics = rospy.get_published_topics()

    # Filter the topics to find those that end with the specified suffix
    matching_topics = [topic for topic, _ in published_topics if topic.endswith(suffix)]

    return matching_topics


class PressureToDepthConverter:
    """
    PressureToDepthConverter converts pressure measurements from MS5837 sensors to depth estimations
    and updates the robot's transform in the TF tree accordingly.

    Attributes:
        target_frame_id (str): The frame ID to which the depth update is relative (e.g., 'fossen_com').
        base_link_frame_id (str): The frame ID of the robot's base link (e.g., 'fossen_base_link').
        world_frame_id (str): The frame ID of the world reference frame (e.g., 'fossen_world').
        tf_buffer (tf2_ros.Buffer): Buffer to store and retrieve transforms.
        tf_listener (tf2_ros.TransformListener): Listener to receive transform updates.
        tf_broadcaster (tf2_ros.TransformBroadcaster): Broadcaster to send updated transforms.
        density (float): Density of the working fluid (e.g., water) in kg/m³.
        g (float): Acceleration due to gravity in m/s².
        zero_z_depth_pressure_set (bool): Flag indicating whether the zero-depth pressure has been set.
        zero_z_depth_pressure (float): The pressure reading corresponding to zero depth.
        depth_estimations (Queue): Queue to store calculated depth estimations for processing.
    """

    def __init__(self, depth_topics):
        """
        Initializes the PressureToDepthConverter by setting up ROS parameters, TF2 listeners and broadcasters,
        and subscribing to pressure sensor topics.

        Args:
            depth_topics (list): List of ROS topic names to subscribe to for pressure data.
        """
        # Initialize the ROS node
        rospy.init_node('pressure_to_depth_converter', anonymous=True)

        # Define frame IDs for transform updates
        self.target_frame_id = 'fossen_com'
        self.base_link_frame_id = 'fossen_base_link'
        self.world_frame_id = 'fossen_world'

        # Initialize TF2 buffer, listener, and broadcaster for handling coordinate transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Retrieve environmental parameters from the ROS parameter server
        self.density = rospy.get_param('/environment_info/working_fluid/density')  # Density of the working fluid in kg/m³
        self.g = rospy.get_param('/environment_info/gravity_constant')  # Acceleration due to gravity in m/s²

        # Initialize zero-depth calibration variables
        self.zero_z_depth_pressure_set = False  # Flag to indicate if zero-depth pressure has been set
        self.zero_z_depth_pressure = 0  # Pressure reading at zero depth in mbar

        # Subscribe to MS5837 pressure sensor topics
        for topic in depth_topics:
            rospy.Subscriber(topic, FluidPressure, self.pressure_callback)

        # Subscribe to the topic for resetting zero-depth pressure calibration
        rospy.Subscriber('/model_info/reset_zero_z_depth_pressure', Bool, self.reset_zero_z_depth_pressure_callback)

        # Initialize a queue to store depth estimations for processing
        self.depth_estimations = Queue()

    def reset_zero_z_depth_pressure_callback(self, msg):
        """
        Callback function to reset the zero-depth pressure calibration.

        Args:
            msg (Bool): Message indicating whether to reset the zero-depth pressure.
        """
        self.zero_z_depth_pressure_set = False  # Reset the calibration flag
        rospy.loginfo("Zero z depth pressure has been reset")  # Log the reset action

    def get_z_axis_diff_in_world_frame(self, msg):
        """
        Calculates the difference in the z-axis between the pressure sensor's frame and the target frame.

        Args:
            msg (FluidPressure): The incoming pressure message containing the sensor's frame ID.

        Returns:
            float or None: The z-axis difference in meters, or None if transformation fails.
        """
        frame_id = msg.header.frame_id  # Frame ID of the pressure sensor

        try:
            # Wait for the transform from world to target_frame_id (e.g., 'fossen_com')
            self.listener.waitForTransform(self.world_frame_id, self.target_frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans_target, _) = self.listener.lookupTransform(self.world_frame_id, self.target_frame_id, rospy.Time(0))

            # Wait for the transform from world to the sensor's frame
            self.listener.waitForTransform(self.world_frame_id, frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans_frame, _) = self.listener.lookupTransform(self.world_frame_id, frame_id, rospy.Time(0))

            # Calculate the z-axis difference in the world frame
            z_diff = trans_frame[2] - trans_target[2]  # Difference in meters
            return -z_diff  # Return the negative to represent depth

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", str(e))  # Log any transformation errors
            return None  # Return None if transformation fails

    def pressure_callback(self, msg):
        """
        Callback function for processing incoming pressure messages.

        This function calculates the depth based on the incoming pressure data and updates the robot's transform.

        Args:
            msg (FluidPressure): The incoming pressure message containing the fluid pressure reading.
        """
        z_diff = self.get_z_axis_diff_in_world_frame(msg)  # Get the z-axis difference
        if z_diff is None:
            return  # Exit if transformation fails

        # Calculate the relative pressure change from the change in depth
        relative_pressure_change_mbar = self.relative_mbar_from_delta_z(z_diff)

        original_pressure_mbar = msg.fluid_pressure  # Original pressure reading in mbar
        adjusted_pressure_mbar = original_pressure_mbar + relative_pressure_change_mbar  # Adjusted pressure

        if not self.zero_z_depth_pressure_set:
            self.zero_z_depth_pressure = adjusted_pressure_mbar  # Set the zero-depth pressure calibration
            self.zero_z_depth_pressure_set = True  # Mark calibration as set

        depth = self.calculate_depth(adjusted_pressure_mbar)  # Calculate depth in meters
        # rospy.loginfo("Calculated depth: %f", depth)  # Debugging: Log the calculated depth (temporarily commented out)
        self.depth_estimations.put(depth)  # Add the calculated depth to the queue

    def run(self):
        """
        Runs the main loop to process depth estimations and update transforms.

        This function continuously checks for new depth estimations in the queue and updates the
        transform of the base link frame accordingly.
        """
        rate = rospy.Rate(25)  # Set the loop rate to 25 Hz
        while not rospy.is_shutdown():
            while not self.depth_estimations.empty():
                current_depth = self.depth_estimations.get()  # Retrieve the latest depth estimation
                self.update_transform_z_only(self.base_link_frame_id, current_depth)  # Update the transform with new depth
            rate.sleep()  # Sleep to maintain the loop rate

    def update_transform_z_only(self, frame_id, new_z_value):
        """
        Updates the transform of the specified frame by modifying only the z-axis value.

        Args:
            frame_id (str): The frame ID to update (e.g., 'fossen_base_link').
            new_z_value (float): The new z-axis value in meters.
        """
        try:
            # Wait for and retrieve the current transform from world to the specified frame
            self.listener.waitForTransform(self.world_frame_id, frame_id, rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform(self.world_frame_id, frame_id, rospy.Time(0))

            # Update only the z value of the translation
            trans = (trans[0], trans[1], new_z_value)  # New z-axis value in meters

            # Broadcast the updated transform with the new translation and existing rotation
            self.broadcaster.sendTransform(
                trans,
                rot,
                rospy.Time.now(),
                frame_id,
                self.world_frame_id
            )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", str(e))  # Log any transformation errors

    def relative_mbar_from_delta_z(self, delta_z):
        """
        Calculates the relative pressure change in mbar based on a change in depth.

        Args:
            delta_z (float): Change in depth in meters.

        Returns:
            float: Relative pressure change in mbar.
        """
        # Calculate pressure change in Pascals
        pressure_change_pa = self.density * self.g * delta_z
        # Convert Pascals to mbar (1 mbar = 100 Pascals)
        pressure_change_mbar = pressure_change_pa / 100
        return pressure_change_mbar

    def calculate_depth(self, pressure_mbar):
        """
        Calculates the depth based on the given pressure reading.

        Args:
            pressure_mbar (float): Measured pressure in mbar.

        Returns:
            float: Calculated depth in meters.
        """
        # Calculate the pressure difference from zero depth pressure
        pressure_difference_pa = (pressure_mbar - self.zero_z_depth_pressure) * 100  # Convert mbar to Pascals
        # Calculate depth using the hydrostatic pressure formula
        depth = pressure_difference_pa / (self.density * self.g)
        return depth


if __name__ == "__main__":
    """
    Entry point for the Pressure to Depth Converter script.

    This section initializes the converter by finding the relevant pressure sensor topics,
    creating an instance of the PressureToDepthConverter class, and running the converter
    to continuously process pressure data and update depth estimations.
    """
    try:
        matching_topics = []
        while len(matching_topics) == 0:
            time.sleep(5)  # Wait for 5 seconds before retrying
            suffix = "MS5837"  # Suffix to identify relevant pressure sensor topics
            matching_topics = find_topics_ending_with(suffix)  # Find matching pressure sensor topics

        converter = PressureToDepthConverter(matching_topics)  # Instantiate the converter with found topics
        converter.run()  # Start the converter's main loop

    except rospy.ROSInterruptException:
        pass  # Handle ROS interrupt exceptions gracefully
