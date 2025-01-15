#!/usr/bin/env python
"""
blended_manual_and_pid_setpoint_controller.py

Author: Scott Mayberry
Date: 2025-01-10

Description:
    This script implements a blended control strategy for the Miniature Underwater Robot (MUR),
    combining both manual keyboard inputs and PID-based automated control for pose setpoints.
    It allows users to manually adjust the robot's position and orientation while simultaneously
    utilizing PID controllers to stabilize or guide the robot towards desired setpoints. The script
    publishes wrench commands to thrusters based on the blended inputs, enabling precise and responsive
    maneuvering of the robot in underwater environments.

    Key Functionalities:
        1. Initializes PID controllers for each degree of freedom (DOF) based on configuration parameters.
        2. Listens to pose setpoint updates and updates internal setpoints accordingly.
        3. Retrieves the robot's current pose using TF2 transformations.
        4. Captures keyboard inputs to allow manual adjustments to the pose setpoints.
        5. Blends manual inputs with PID-controlled efforts to compute final wrench commands.
        6. Publishes wrench commands to the appropriate ROS topic for thruster actuation.
        7. Handles emergency stop and reset functionalities via specific keyboard inputs.

Dependencies:
    - rospy: Python client library for ROS.
    - pygame: Library for creating multimedia applications (used here for capturing keyboard inputs).
    - geometry_msgs.msg: ROS message types for geometric data.
    - tf2_ros: TF2 library for ROS transformations.
    - tf.transformations: Utility functions for transforming between different representations (e.g., Euler angles to quaternions).
    - simple_pid.PID: PID controller class.
    - math: Standard Python library for mathematical operations.
    - time: Time-related functions.

License:
    MIT License
"""

import rospy  # ROS Python client library
import pygame  # Library for handling multimedia (keyboard inputs)
from geometry_msgs.msg import WrenchStamped, PoseStamped  # ROS message types for wrench and poses
from std_msgs.msg import Bool  # Import the Bool message type for publishing boolean flags
import tf2_ros  # TF2 library for handling transformations
from tf.transformations import euler_from_quaternion  # Utility function to convert quaternions to Euler angles
from simple_pid import PID  # PID controller class from the simple_pid package
import math  # Standard Python math library
import time  # Time-related functions
import tf.transformations  # Import the tf.transformations module for additional transformation utilities

# Initialize Pygame to capture keyboard events
pygame.init()
screen = pygame.display.set_mode((200, 200))  # Create a small Pygame window (required for event handling)

class BlendedController:
    def __init__(self):
        """
        Initializes the BlendedController by setting up PID controllers, TF2 listener,
        subscribers, and publishers based on configuration parameters.
        """
        # Load control_info parameters from ROS parameter server
        self.control_info = rospy.get_param('/control_info')  # Retrieve all control_info parameters
        self.rate = self.control_info['stabilization']['rate']  # Control loop rate in Hz
        self.gains_info = self.control_info['stabilization']['gains']  # PID gains for each DOF
        self.control_modes = self.control_info['stabilization']['control_mode']  # Control modes for each DOF

        # Initialize setpoints for each Degree of Freedom (DOF)
        self.current_setpoints = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Initialize PID controllers for each DOF
        self.pids = {}
        for dof, params in self.gains_info.items():
            # Create a PID instance for each DOF with corresponding gains and initial setpoint
            self.pids[dof] = PID(params['kp'], params['ki'], params['kd'], setpoint=self.current_setpoints[dof])

        # Initialize TF2 listener to retrieve robot's current pose
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers
        self.wrench_pub = rospy.Publisher('/model/requested_thruster_wrench', WrenchStamped, queue_size=10)  # Publisher for wrench commands
        self.reset_pub = rospy.Publisher('/model_info/reset_zero_z_depth_pressure', Bool, queue_size=10)  # Publisher for reset functionality
        self.pose_pub = rospy.Publisher('/control/pose_setpoint', PoseStamped, queue_size=10)  # Publisher for pose setpoints

        self.emergency_stop = False  # Initialize emergency stop state

    def get_keyboard_command(self):
        """
        Captures the current state of the keyboard and maps specific keys to pose adjustments.

        Returns:
            dict: A dictionary containing incremental adjustments for each DOF.
        """
        global current_x, current_y, current_z, current_yaw  # Access and modify global setpoint variables

        # Initialize command increments for position and orientation
        x_inc, y_inc, z_inc, roll_inc, pitch_inc, yaw_inc = 0, 0, 0, 0, 0, 0
        scale = 11.0  # Scale factor for direct manual control
        setpoint_scale = 0.05  # Scale factor for setpoint adjustments in PID mode
        yaw_scale = 0.05  # Separate scale factor for yaw adjustments

        # Get the current state of all keyboard keys
        keys = pygame.key.get_pressed()

        # Emergency stop control
        if keys[pygame.K_PERIOD]:  # '.' key for emergency stop
            self.emergency_stop = True
            print("Emergency stop activated. Sending zero wrench.")
        elif keys[pygame.K_COMMA]:  # ',' key to turn the system back on
            self.emergency_stop = False
            print("System reactivated. Resuming control.")

        # If emergency stop is activated, all commands are set to zero
        if self.emergency_stop:
            print("EMERGENCY STOP ACTIVE")
            return {'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

        # Adjust setpoints if in PID mode, otherwise direct control
        if self.control_modes['x'] == 'pid':
            if keys[pygame.K_w]: x_inc = setpoint_scale  # Increment x setpoint in PID mode
            if keys[pygame.K_s]: x_inc = -setpoint_scale  # Decrement x setpoint in PID mode
        else:
            if keys[pygame.K_w]: x_inc = 1  # Direct control: Increase x
            if keys[pygame.K_s]: x_inc = -1  # Direct control: Decrease x

        if self.control_modes['y'] == 'pid':
            if keys[pygame.K_a]: y_inc = -setpoint_scale  # Increment y setpoint in PID mode
            if keys[pygame.K_d]: y_inc = setpoint_scale  # Decrement y setpoint in PID mode
        else:
            if keys[pygame.K_a]: y_inc = -1  # Direct control: Decrease y
            if keys[pygame.K_d]: y_inc = 1  # Direct control: Increase y

        if self.control_modes['z'] == 'pid':
            if keys[pygame.K_q]: z_inc = setpoint_scale  # Increment z setpoint in PID mode
            if keys[pygame.K_e]: z_inc = -setpoint_scale  # Decrement z setpoint in PID mode
        else:
            if keys[pygame.K_q]: z_inc = 1  # Direct control: Increase z
            if keys[pygame.K_e]: z_inc = -1  # Direct control: Decrease z

        if self.control_modes['roll'] == 'pid':
            if keys[pygame.K_i]: roll_inc = setpoint_scale  # Increment roll setpoint in PID mode
            if keys[pygame.K_k]: roll_inc = -setpoint_scale  # Decrement roll setpoint in PID mode
        else:
            if keys[pygame.K_i]: roll_inc = 1  # Direct control: Increase roll
            if keys[pygame.K_k]: roll_inc = -1  # Direct control: Decrease roll

        if self.control_modes['yaw'] == 'pid':
            if keys[pygame.K_j]: yaw_inc = yaw_scale  # Increment yaw setpoint in PID mode
            if keys[pygame.K_l]: yaw_inc = -yaw_scale  # Decrement yaw setpoint in PID mode
        else:
            if keys[pygame.K_j]: yaw_inc = yaw_scale  # Direct control: Increment yaw
            if keys[pygame.K_l]: yaw_inc = -yaw_scale  # Direct control: Decrement yaw

        # Apply direct manual control scaling
        if self.control_modes['x'] != 'pid': x_inc *= scale  # Scale x increment if not in PID mode
        if self.control_modes['y'] != 'pid': y_inc *= scale  # Scale y increment if not in PID mode
        if self.control_modes['z'] != 'pid': z_inc *= scale  # Scale z increment if not in PID mode
        if self.control_modes['roll'] != 'pid': roll_inc /= 8  # Reduce roll increment if not in PID mode
        if self.control_modes['yaw'] != 'pid': yaw_inc /= 8  # Reduce yaw increment if not in PID mode

        # Update setpoints if in PID mode
        if self.control_modes['x'] == 'pid': self.current_setpoints['x'] += x_inc  # Adjust x setpoint
        if self.control_modes['y'] == 'pid': self.current_setpoints['y'] += y_inc  # Adjust y setpoint
        if self.control_modes['z'] == 'pid': self.current_setpoints['z'] = max(-4.0, min(0.0, z_inc+self.current_setpoints['z']))  # Adjust z setpoint with limits
        if self.control_modes['roll'] == 'pid': self.current_setpoints['roll'] = (self.current_setpoints['roll'] + roll_inc) % (2 * math.pi)  # Adjust roll setpoint with wrap-around
        if self.control_modes['yaw'] == 'pid': self.current_setpoints['yaw'] = (self.current_setpoints['yaw'] + yaw_inc) % (2 * math.pi)  # Adjust yaw setpoint with wrap-around

        # Check for reset key ('/') to reset zero depth pressure
        if keys[pygame.K_SLASH]:
            self.reset_pub.publish(True)  # Publish True to reset topic
            print("Published True to /model_info/reset_zero_z_depth_pressure")

        # Return the manual command values for non-PID controlled DOFs
        return {'x': x_inc, 'y': y_inc, 'z': z_inc, 'roll': roll_inc, 'pitch': pitch_inc, 'yaw': yaw_inc}

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

    def shortest_angular_difference(self, current, target):
        """
        Computes the shortest angular difference between two angles.

        Args:
            current (float): Current angle in radians.
            target (float): Target angle in radians.

        Returns:
            float: Shortest difference in radians between current and target angles.
        """
        difference = target - current
        while difference > math.pi:
            difference -= 2 * math.pi
        while difference < -math.pi:
            difference += 2 * math.pi
        return difference

    def control_loop(self):
        """
        Main control loop that runs at the specified rate, computes control efforts using PID controllers,
        and publishes wrench commands to actuate the thrusters.
        """
        rate = rospy.Rate(self.rate)  # Set the loop rate based on configuration
        while not rospy.is_shutdown():  # Continue looping until ROS is shut down
            # Check for keyboard input and get manual input commands
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    rospy.signal_shutdown('Quit event received')  # Gracefully shut down the ROS node

            manual_input = self.get_keyboard_command()  # Generate manual commands based on keyboard inputs

            # Get current pose from the robot
            orientation_and_position = self.get_current_orientation_and_position()

            if orientation_and_position:
                wrench_msg = WrenchStamped()  # Initialize a WrenchStamped message to hold control efforts
                wrench_msg.header.stamp = rospy.Time.now()  # Current timestamp
                wrench_msg.header.frame_id = "fossen_com"  # Reference frame for the wrench

                # Create and publish PoseStamped message for PID-controlled setpoints
                pose_msg = PoseStamped()
                pose_msg.header.stamp = rospy.Time.now()  # Current timestamp
                pose_msg.header.frame_id = "fossen_world"  # Reference frame for the pose

                # Set the desired position in the pose message
                pose_msg.pose.position.x = self.current_setpoints['x']
                pose_msg.pose.position.y = self.current_setpoints['y']
                pose_msg.pose.position.z = self.current_setpoints['z']

                # Convert Euler angles (roll, pitch, yaw) to a quaternion for orientation
                quat = tf.transformations.quaternion_from_euler(
                    self.current_setpoints['roll'],
                    self.current_setpoints['pitch'],
                    self.current_setpoints['yaw']
                )
                pose_msg.pose.orientation.x = quat[0]
                pose_msg.pose.orientation.y = quat[1]
                pose_msg.pose.orientation.z = quat[2]
                pose_msg.pose.orientation.w = quat[3]

                self.pose_pub.publish(pose_msg)  # Publish the PoseStamped message

                # Iterate over each Degree of Freedom (DOF) to compute control outputs
                for dof in ['x', 'y', 'z', 'roll', 'pitch', 'yaw']:
                    if self.control_modes[dof] == 'pid':
                        error = orientation_and_position[dof] - self.current_setpoints[dof] if dof not in ['roll', 'pitch', 'yaw'] else \
                                BlendedController.shortest_angular_difference(orientation_and_position[dof], self.current_setpoints[dof])
                        control_output = self.pids[dof](error)
                        
                        # Print the target vs actual position
                        print(f"DOF: {dof.upper():<5} | Target: {self.current_setpoints[dof]:8.4f} | Current: {orientation_and_position[dof]:8.4f} | Error: {error:8.4f}")

                    else:
                        # Use manual input for non-PID controlled DOFs
                        control_output = manual_input[dof]

                    # Assign the control output to the appropriate component of the wrench message
                    if dof in ['x', 'y', 'z']:
                        setattr(wrench_msg.wrench.force, dof, control_output)  # Assign to force components
                    else:
                        # Map 'roll', 'pitch', 'yaw' to torque components
                        setattr(wrench_msg.wrench.torque, {'roll': 'x', 'pitch': 'y', 'yaw': 'z'}[dof], control_output)

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
    Entry point for the Blended Manual and PID Setpoint Controller script.

    This section initializes the ROS node, waits for the system to boot, creates an instance of the
    BlendedController, and starts the control loop.
    """
    rospy.init_node('blended_controller')  # Initialize the ROS node with the name 'blended_controller'
    while not get_has_booted():  # Continuously check if the system has booted
        time.sleep(5)  # Wait for 5 seconds before rechecking
    time.sleep(3)  # Additional wait to ensure all systems are ready
    controller = BlendedController()  # Create an instance of the controller
    controller.control_loop()  # Start the main control loop
    pygame.quit()  # Ensure that Pygame quits properly upon exiting the script
