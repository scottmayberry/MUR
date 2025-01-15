#!/usr/bin/env python
"""
ros_pid_setpoint_control.py

Author: Scott Mayberry
Date: 2025-01-10

Description:
    This script implements a PID-based stabilization controller for the Miniature Underwater Robot (MUR).
    It subscribes to pose setpoints, retrieves the robot's current pose, computes control efforts using PID
    controllers for each degree of freedom (DOF), and publishes the resulting wrench commands to control the
    robot's thrusters. The controller ensures that the robot maintains or reaches the desired position and
    orientation by minimizing the error between the current state and the setpoints.

Key Functionalities:
    1. Initializes PID controllers for each DOF based on configuration parameters.
    2. Listens to pose setpoint updates and updates internal setpoints accordingly.
    3. Retrieves the robot's current pose using TF2 transformations.
    4. Calculates errors between current pose and setpoints.
    5. Applies PID control to compute wrench commands.
    6. Publishes wrench commands to the appropriate ROS topic for thruster actuation.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_control ros_pid_setpoint_control.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_control mur_control.launch
        ```

Dependencies:
    - rospy: Python client library for ROS.
    - tf2_ros: TF2 library for ROS transformations.
    - geometry_msgs.msg: ROS message types for geometric data.
    - sensor_msgs.msg: ROS message types for sensor data.
    - simple_pid.PID: PID controller class.
    - numpy: Numerical operations.
    - time: Time-related functions.

License:
    MIT License
"""

import rospy
import tf2_ros
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_multiply, quaternion_from_euler
from geometry_msgs.msg import TransformStamped, WrenchStamped, Vector3Stamped, PoseStamped
from tf.transformations import euler_from_quaternion
from simple_pid import PID  # Assuming you have a PID class
import numpy as np
import time


class StabilizationController:
    def __init__(self):
        """
        Initializes the StabilizationController by setting up PID controllers, TF2 listener,
        subscribers, and publishers based on configuration parameters.
        """
        # Load control_info parameters from ROS parameter server
        # control_info = rospy.get_param('/control_info/stabilization')
        self.control_info = rospy.get_param('/control_info')  # Retrieve all control_info parameters
        self.rate = self.control_info['stabilization']['rate']  # Control loop rate in Hz
        self.gains_info = self.control_info['stabilization']['gains']  # PID gains for each DOF

        # Initialize PID controllers for each Degree of Freedom (DOF)
        self.pids = {}
        for dof, params in self.gains_info.items():
            self.pids[dof] = PID(params['kp'], params['ki'], params['kd'], setpoint=0)  # Create PID instance

        # Initialize TF2 listener to retrieve robot's current pose
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize setpoints dictionary to store desired poses
        self.setpoints = {}
        # Example structure:
        # {
        #     'roll': 0,
        #     'pitch': 0,
        #     'yaw': 0,
        #     'x': 0,
        #     'y': 0,
        #     'z': 0
        # }

        # Subscriber to the 'pose_setpoint' topic to receive desired pose updates
        rospy.Subscriber('pose_setpoint', PoseStamped, self.setpoint_callback)

        # Publisher to send computed wrench commands to the thrusters
        self.wrench_pub = rospy.Publisher('/model/requested_thruster_wrench', WrenchStamped, queue_size=10)

    def setpoint_callback(self, msg):
        """
        Callback function that updates the setpoints based on incoming PoseStamped messages.

        Args:
            msg (PoseStamped): The desired pose message containing position and orientation.
        """
        # The following lines are temporarily commented out and can be used for direct setpoint updates
        # # Update orientation setpoints
        # quat = msg.pose.orientation
        # euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        # self.setpoints['roll'] = euler[0]
        # self.setpoints['pitch'] = euler[1]
        # self.setpoints['yaw'] = euler[2]

        # # Update position setpoints
        # self.setpoints['x'] = msg.pose.position.x
        # self.setpoints['y'] = msg.pose.position.y
        # self.setpoints['z'] = msg.pose.position.z

        try:
            # Lookup transform from 'world' frame to the frame specified in the incoming message
            trans = self.tf_buffer.lookup_transform('world', msg.header.frame_id, rospy.Time(0))

            # Extract translation and rotation from the transform
            trans_translation = trans.transform.translation
            trans_rotation = trans.transform.rotation

            # Extract position from the incoming setpoint message
            position = msg.pose.position
            # Transform the position to the 'world' frame by adding the translation
            transformed_position = [
                position.x + trans_translation.x,
                position.y + trans_translation.y,
                position.z + trans_translation.z
            ]

            # Extract and transform orientation using quaternion multiplication
            quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            trans_quat = [trans_rotation.x, trans_rotation.y, trans_rotation.z, trans_rotation.w]
            world_quat = quaternion_multiply(trans_quat, quat)  # Combine rotations
            world_euler = euler_from_quaternion(world_quat)  # Convert to Euler angles

            # Update the setpoints with transformed position and orientation
            self.setpoints = {
                'x': transformed_position[0],
                'y': transformed_position[1],
                'z': transformed_position[2],
                'roll': world_euler[0],
                'pitch': world_euler[1],
                'yaw': world_euler[2]
            }
            # print(self.setpoints)  # Uncomment for debugging

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF lookup failed: {e}")  # Log error if transform lookup fails

    def get_current_orientation_and_position(self):
        """
        Retrieves the current orientation and position of the robot from the TF2 transform.

        Returns:
            dict or None: A dictionary containing current roll, pitch, yaw, x, y, z if successful; otherwise, None.
        """
        try:
            # Lookup transform from 'world' frame to 'fossen_com' frame (robot's center of mass)
            trans = self.tf_buffer.lookup_transform('world', 'fossen_com', rospy.Time(0))
            quat = trans.transform.rotation  # Extract rotation
            euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])  # Convert to Euler angles
            position = trans.transform.translation  # Extract position
            return {
                'roll': euler[0],
                'pitch': euler[1],
                'yaw': euler[2],
                'x': position.x,
                'y': position.y,
                'z': position.z
            }
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None  # Return None if transform lookup fails

    def shortest_angular_difference(current, target):
        """
        Computes the shortest angular difference between two angles.

        Args:
            current (float): Current angle in radians.
            target (float): Target angle in radians.

        Returns:
            float: Shortest difference in radians between current and target angles.
        """
        difference = target - current
        while difference > np.pi:
            difference -= 2 * np.pi
        while difference < -np.pi:
            difference += 2 * np.pi
        return difference

    def control_loop(self):
        """
        Main control loop that runs at the specified rate, computes control efforts using PID controllers,
        and publishes wrench commands to actuate the thrusters.
        """
        rate = rospy.Rate(self.rate)  # Set the loop rate based on configuration
        while not rospy.is_shutdown():  # Continue looping until ROS is shut down
            orientation_and_position = self.get_current_orientation_and_position()  # Get current state
            if orientation_and_position and self.setpoints.keys():
                # Initialize a WrenchStamped message to hold control efforts
                wrench_msg = WrenchStamped()
                wrench_msg.header.stamp = rospy.Time.now()  # Current timestamp
                wrench_msg.header.frame_id = "fossen_com"  # Reference frame for the wrench

                # Iterate over each DOF and compute control output using PID
                for dof, pid in self.pids.items():
                    if dof in ['roll', 'pitch', 'yaw']:
                        # Compute the shortest angular difference for rotational DOFs
                        error = StabilizationController.shortest_angular_difference(
                            orientation_and_position[dof],
                            self.setpoints[dof]
                        )
                        # if dof == 'yaw':
                        #     print(error)  # Uncomment for debugging yaw errors
                        control_output = pid(error)  # Compute PID output
                    else:
                        # Compute positional error for translational DOFs
                        error = orientation_and_position[dof] - self.setpoints[dof]
                        control_output = pid(error)  # Compute PID output
                        # print(orientation_and_position[dof], self.setpoints['z'], control_output)  # Uncomment for debugging
                        # if dof == 'z':
                        #     print(control_output, orientation_and_position[dof])  # Uncomment for debugging z-axis

                    # Assign the control output to the appropriate component of the wrench message
                    if dof == 'roll':
                        wrench_msg.wrench.torque.x = control_output
                    elif dof == 'pitch':
                        wrench_msg.wrench.torque.y = control_output
                    elif dof == 'yaw':
                        wrench_msg.wrench.torque.z = control_output
                    elif dof == 'x':
                        wrench_msg.wrench.force.x = control_output
                    elif dof == 'y':
                        wrench_msg.wrench.force.y = control_output
                    elif dof == 'z':
                        wrench_msg.wrench.force.z = control_output

                # print(wrench_msg)  # Uncomment to visualize the wrench message

                # Publish the computed wrench to the thruster command topic
                self.wrench_pub.publish(wrench_msg)
            rate.sleep()  # Sleep to maintain loop rate


def get_has_booted():
    """
    Checks if the system has booted by retrieving the '/module_info/hasBooted' parameter.

    Returns:
        bool: True if the system has booted; False otherwise.
    """
    return rospy.get_param('/module_info/hasBooted')


if __name__ == '__main__':
    """
    Entry point for the ROS PID Setpoint Control script.

    This section initializes the ROS node, waits for the system to boot, creates an instance of the
    StabilizationController, and starts the control loop.
    """
    rospy.init_node('stabilization_controller')  # Initialize the ROS node with the name 'stabilization_controller'
    
    # Continuously check if the system has booted before starting the controller
    while not get_has_booted():
        time.sleep(5)  # Wait for 5 seconds before rechecking
    
    time.sleep(3)  # Additional wait to ensure all systems are ready
    
    controller = StabilizationController()  # Create an instance of the controller
    controller.control_loop()  # Start the main control loop
