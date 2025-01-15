#!/usr/bin/env python
"""
ros_thruster_command_conversion.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script handles the conversion of desired wrench commands into thruster-specific control signals
    for the Miniature Underwater Robot (MUR). It subscribes to a `WrenchStamped` topic containing desired
    force and torque values, computes the necessary thruster forces to achieve the desired wrench, and
    publishes the resulting thruster commands in both unit and microsecond (PWM) formats.

    The script performs the following steps:
        1. Receives desired wrench commands (force and torque) via a ROS topic.
        2. Computes the corresponding thruster forces using a pseudo-inverse approach.
        3. Converts the thruster forces into normalized unit commands.
        4. Translates unit commands into PWM signals based on ESC (Electronic Speed Controller) formulas.
        5. Publishes the thruster commands for execution.

    This process ensures accurate and efficient control of the robot's thrusters, enabling precise maneuvering
    and stabilization based on the desired motion commands.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model ros_thruster_command_conversion.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - numpy
    - std_msgs.msg (Float64MultiArray, Int32MultiArray)
    - geometry_msgs.msg (WrenchStamped, Vector3Stamped)
    - tf2_ros
    - tf2_geometry_msgs
    - sympy

License:
    MIT License
"""

import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Int32MultiArray
from geometry_msgs.msg import WrenchStamped, Vector3Stamped
import tf2_ros
import tf2_geometry_msgs
from sympy import symbols, lambdify
import sympy as sp
import queue

def compute_force_thruster_commands(desired_wrench, thrusters):
    """
    Computes the thruster commands required to achieve the desired wrench.

    This function constructs a wrench matrix based on the thrusters' positions and orientations,
    then uses the pseudo-inverse of this matrix to determine the necessary thruster forces.

    Args:
        desired_wrench (WrenchStamped): The desired force and torque to be achieved.
        thrusters (list): A list of thruster configurations, each containing position, orientation,
                          and identification information.

    Returns:
        np.ndarray: An array of thruster force commands corresponding to each thruster.
    """
    num_thrusters = len(thrusters)
    # Initialize a wrench matrix with zeros. Changed from 5 to 6 for full wrench (force + torque)
    T = np.zeros((6, num_thrusters))

    # Define the force direction in the thruster's frame (aligned with x-axis)
    force_in_thruster_frame = Vector3Stamped()
    force_in_thruster_frame.vector.x = 1
    force_in_thruster_frame.vector.y = 0
    force_in_thruster_frame.vector.z = 0
    force_in_thruster_frame.header.stamp = rospy.Time(0)

    for thruster in thrusters:
        thruster_id = thruster['board_plug_id']  # Unique identifier for the thruster
        thruster_frame = f"thruster_{thruster_id}"  # Frame name for the thruster
        force_in_thruster_frame.header.frame_id = thruster_frame  # Set the frame ID for the transform

        # Transform the force direction from the thruster's frame to the desired wrench's frame using tf2
        transformed_force = tf_buffer.transform(
            force_in_thruster_frame,
            target_frame=desired_wrench.header.frame_id,
            timeout=rospy.Duration(1)
        )
        # Extract the force direction as a numpy array
        force_direction = np.array([
            transformed_force.vector.x,
            transformed_force.vector.y,
            transformed_force.vector.z
        ])

        # Retrieve the transform between the thruster frame and the desired wrench frame
        transform = tf_buffer.lookup_transform(
            desired_wrench.header.frame_id,
            thruster_frame,
            rospy.Time(0)
        )
        # Extract the position of the thruster relative to the desired wrench frame
        pos = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ])
        F = force_direction  # Force vector
        tau = np.cross(pos, force_direction)  # Torque vector resulting from the thruster's force
        # Populate the wrench matrix with force and torque contributions from the thruster
        T[:, thruster_id] = np.concatenate([F, tau])

    # Use preloaded rows_to_keep to select relevant degrees of freedom
    T = T[rows_to_keep, :]

    # Construct the desired wrench vector based on selected degrees of freedom
    desired_vector = np.array([
        desired_wrench.wrench.force.x,
        desired_wrench.wrench.force.y,
        desired_wrench.wrench.force.z,
        desired_wrench.wrench.torque.x,
        desired_wrench.wrench.torque.y,
        desired_wrench.wrench.torque.z
    ])[rows_to_keep]
    # Compute thruster commands using the pseudo-inverse of the wrench matrix
    thruster_commands = np.linalg.pinv(T).dot(desired_vector)
    # Optionally clip the thruster commands to a specific range
    # thruster_commands = np.clip(thruster_commands, -1, 1)

    return thruster_commands

def convert_force_to_unit_thruster_commands(force_thruster_commands, model_info):
    """
    Converts thruster force commands into normalized unit commands.

    This function maps the computed thruster forces to a normalized range suitable for further
    processing or direct control inputs.

    Args:
        force_thruster_commands (np.ndarray): Array of thruster force commands.
        model_info (dict): Configuration information for the model, including thruster and ESC details.

    Returns:
        list: A list of normalized unit thruster commands.
    """
    # Initialize a list to store unit thruster commands with zero values
    unit_thruster_commands = [0] * len(force_thruster_commands)
    for idx, thruster in enumerate(model_info['thrusters']):
        min_force, max_force = model_info['escs'][thruster['esc']]['thrust_force']
        thruster_id = thruster['board_plug_id']
        # Normalize the thruster force command to a range of [-1, 1]
        unit_value = (force_thruster_commands[thruster_id] - min_force) / (max_force - min_force) * 2 - 1
        unit_thruster_commands[thruster_id] = unit_value
    return unit_thruster_commands

def convert_unit_to_microseconds_thruster_commands(unit_thruster_commands, model_info):
    """
    Converts normalized unit thruster commands into microsecond (PWM) signals.

    This function applies ESC-specific formulas to translate normalized commands into PWM
    signals that control the thrusters' speeds.

    Args:
        unit_thruster_commands (list): List of normalized unit thruster commands.
        model_info (dict): Configuration information for the model, including thruster and ESC details.

    Returns:
        list: A list of thruster commands in microseconds (PWM signals).
    """
    # Initialize a list to store PWM thruster commands with zero values
    pwm_thruster_commands = [0] * len(unit_thruster_commands)
    for thruster in model_info['thrusters']:
        thruster_id = thruster['board_plug_id']
        esc_name = thruster['esc']
        zero_threshold = model_info['escs'][thruster['esc']]['zero_threshold']
        # Check if the unit command is within the zero threshold to set PWM to zero value
        if np.abs(unit_thruster_commands[thruster_id]) < zero_threshold:
            pwm_value = model_info['escs'][thruster['esc']]['zero_value']
        else:
            # Apply the ESC-specific formula to convert unit command to PWM value
            pwm_value = esc_formulas[esc_name](unit_thruster_commands[thruster_id])
        pwm_thruster_commands[thruster_id] = int(pwm_value)
    return pwm_thruster_commands

def wrench_callback(msg):
    """
    Callback function for processing incoming desired wrench commands.

    This function is triggered upon receiving a `WrenchStamped` message. It computes the necessary
    thruster commands to achieve the desired wrench, converts these commands into normalized units
    and PWM signals, and publishes the resulting commands to the appropriate ROS topics.

    Args:
        msg (WrenchStamped): The desired wrench command message containing force and torque.
    """
    # Compute thruster force commands based on the desired wrench
    force_thruster_commands = compute_force_thruster_commands(msg, model_info['thrusters'])
    
    # Convert thruster force commands into normalized unit commands
    unit_thruster_commands = convert_force_to_unit_thruster_commands(force_thruster_commands, model_info)

    # Convert unit thruster commands into PWM signals for actual thruster control
    us_thruster_commands = convert_unit_to_microseconds_thruster_commands(unit_thruster_commands, model_info)

    # Publish PWM thruster commands as Int32MultiArray messages
    # print("force_thruster_commands", force_thruster_commands)
    # print("unit_thruster_commands", unit_thruster_commands)
    # print("us_thruster_commands", us_thruster_commands)

    # Publish all thruster data to their respective topics
    us_thruster_command_msg = Int32MultiArray(data=us_thruster_commands)
    us_thruster_pub.publish(us_thruster_command_msg)

    unit_thruster_command_msg = Float64MultiArray(data=unit_thruster_commands)
    unit_thruster_pub.publish(unit_thruster_command_msg)

    force_thruster_commands_msg = Float64MultiArray(data=force_thruster_commands)
    force_thruster_pub.publish(force_thruster_commands_msg)

    # wrench_queue.put(msg)  # Placeholder for future queue implementation

# wrench_queue = queue.Queue()

if __name__ == '__main__':
    """
    Entry point for the Thruster Command Conversion script.

    Initializes the ROS node, retrieves model information from ROS parameters, sets up the TF2 buffer
    and listener for coordinate transformations, preloads ESC-specific formulas, subscribes to the
    desired wrench command topic, and initializes publishers for thruster commands.

    The node continuously listens for wrench commands, processes them, and publishes the corresponding
    thruster control signals to enable accurate and responsive maneuvering of the MUR.
    """
    # Initialize the ROS node named 'thruster_command_conversion_node'
    rospy.init_node('thruster_command_conversion_node')
    
    # Retrieve model configuration parameters from the ROS parameter server
    model_info = rospy.get_param('/model_info')

    # Preload rows_to_keep based on the desired control inputs
    control_inputs = model_info.get('control_inputs', ['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
    dof_map = {'x': 0, 'y': 1, 'z': 2, 'roll': 3, 'pitch': 4, 'yaw': 5}
    rows_to_keep = [dof_map[dof] for dof in control_inputs]

    # Initialize TF2 buffer and listener for handling coordinate transformations
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2)  # Allow some time for the TF listener to populate the buffer

    # Preload the lambda functions for each ESC to convert unit commands to PWM signals
    esc_formulas = {}
    for esc_name, esc_data in model_info['escs'].items():
        x = symbols('x')  # Define a symbolic variable for the formula
        # Generate the lambda function for the ESC's formula using SymPy
        esc_formulas[esc_name] = lambdify(x, esc_data['formula'], modules=["numpy", "sympy"])

    # Subscribe to the 'requested_thruster_wrench' topic to receive desired wrench commands
    wrench_sub = rospy.Subscriber('requested_thruster_wrench', WrenchStamped, wrench_callback)
    
    # Initialize ROS publishers for thruster commands
    unit_thruster_pub = rospy.Publisher('unit_thruster_commands', Float64MultiArray, queue_size=2)
    us_thruster_pub = rospy.Publisher('us_thruster_commands', Int32MultiArray, queue_size=2)
    force_thruster_pub = rospy.Publisher('force_thruster_commands', Float64MultiArray, queue_size=2)
    
    # Keep the node running and processing callbacks until ROS is shut down
    rospy.spin()
