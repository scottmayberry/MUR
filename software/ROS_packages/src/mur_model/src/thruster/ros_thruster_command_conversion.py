#!/usr/bin/env python

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
    num_thrusters = len(thrusters)
    T = np.zeros((6, num_thrusters))  # Changed from 5 to 6 for full wrench

    # Define the force direction in the thruster's frame (aligned with x-axis)
    force_in_thruster_frame = Vector3Stamped()
    force_in_thruster_frame.vector.x = 1
    force_in_thruster_frame.vector.y = 0
    force_in_thruster_frame.vector.z = 0
    force_in_thruster_frame.header.stamp = rospy.Time(0)
    

    for thruster in thrusters:
        thruster_id = thruster['id']
        thruster_frame = f"thruster_{thruster_id}"
        force_in_thruster_frame.header.frame_id = thruster_frame

        # Transform the force direction from the thruster's frame to the COM frame using tf2
        transformed_force = tf_buffer.transform(force_in_thruster_frame, target_frame=desired_wrench.header.frame_id, timeout=rospy.Duration(1))
        force_direction = np.array([transformed_force.vector.x, transformed_force.vector.y, transformed_force.vector.z])

        # Extract the translation from the transform
        transform = tf_buffer.lookup_transform(desired_wrench.header.frame_id, thruster_frame, rospy.Time(0))
        pos = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        F = force_direction
        tau = np.cross(pos, force_direction)
        T[:, thruster_id] = np.concatenate([F, tau])

    # Use preloaded rows_to_keep (assuming you still want to do this)
    T = T[rows_to_keep, :]

    desired_vector = np.array([desired_wrench.wrench.force.x, desired_wrench.wrench.force.y, desired_wrench.wrench.force.z, desired_wrench.wrench.torque.x, desired_wrench.wrench.torque.y, desired_wrench.wrench.torque.z])[rows_to_keep]
    thruster_commands = np.linalg.pinv(T).dot(desired_vector)
    # thruster_commands = np.clip(thruster_commands, -1, 1)

    return thruster_commands



def convert_force_to_unit_thruster_commands(force_thruster_commands, model_info):
    # calculate unit thruster commands
    unit_thruster_commands = [0]*len(force_thruster_commands)
    for idx, thruster in enumerate(model_info['thrusters']):
        min_force, max_force = model_info['escs'][thruster['esc']]['thrust_force']
        thruster_id = thruster['id']
        unit_value = (force_thruster_commands[thruster_id] - min_force) / (max_force - min_force) * 2 - 1
        unit_thruster_commands[thruster_id] = unit_value
    return unit_thruster_commands

def convert_unit_to_microseconds_thruster_commands(unit_thruster_commands, model_info):
    # Convert to the PWM signal thruster command using the esc formula
    pwm_thruster_commands = [0]*len(unit_thruster_commands)
    for thruster in model_info['thrusters']:
        thruster_id = thruster['id']
        esc_name = thruster['esc']
        zero_threshold = model_info['escs'][thruster['esc']]['zero_threshold']
        if np.abs(unit_thruster_commands[thruster_id]) < zero_threshold:
            pwm_value = model_info['escs'][thruster['esc']]['zero_value']
        else:
            pwm_value = esc_formulas[esc_name](unit_thruster_commands[thruster_id])
        pwm_thruster_commands[thruster_id] = int(pwm_value)
    return pwm_thruster_commands


def wrench_callback(msg):
    # force thruster commands
    force_thruster_commands = compute_force_thruster_commands(msg, model_info['thrusters'])
    
    # calculate unit thruster commands
    unit_thruster_commands = convert_force_to_unit_thruster_commands(force_thruster_commands, model_info)

    # calculate pwm thruster commands
    us_thruster_commands = convert_unit_to_microseconds_thruster_commands(unit_thruster_commands, model_info)

    # Publish PWM thruster commands
    # print("force_thruster_commands", force_thruster_commands)
    # print("unit_thruster_commands", unit_thruster_commands)
    # print("us_thruster_commands", us_thruster_commands)

    # publish all thruster data
    us_thruster_command_msg = Int32MultiArray(data=us_thruster_commands)
    us_thruster_pub.publish(us_thruster_command_msg)

    unit_thruster_command_msg = Float64MultiArray(data=unit_thruster_commands)
    unit_thruster_pub.publish(unit_thruster_command_msg)

    force_thruster_commands_msg = Float64MultiArray(data=force_thruster_commands)
    force_thruster_pub.publish(force_thruster_commands_msg)
    

    
# wrench_queue = queue.Queue()

if __name__ == '__main__':
    rospy.init_node('thruster_command_conversion_node')
    
    model_info = rospy.get_param('/model_info')

    # Preload rows_to_keep
    control_inputs = model_info.get('control_inputs', ['x', 'y', 'z', 'roll', 'pitch', 'yaw'])
    dof_map = {'x': 0, 'y': 1, 'z': 2, 'roll': 3, 'pitch': 4, 'yaw': 5}
    rows_to_keep = [dof_map[dof] for dof in control_inputs]

    # Initialize tf2 buffer and listener
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2)

    # Preload the lambda functions for every ESC
    esc_formulas = {}
    for esc_name, esc_data in model_info['escs'].items():
        x = symbols('x')
        # Generate the lambda function for the formula
        esc_formulas[esc_name] = lambdify(x, esc_data['formula'], modules=["numpy", "sympy"])
    
    wrench_sub = rospy.Subscriber('requested_thruster_wrench', WrenchStamped, wrench_callback)
    unit_thruster_pub = rospy.Publisher('unit_thruster_commands', Float64MultiArray, queue_size=2)
    us_thruster_pub = rospy.Publisher('us_thruster_commands', Int32MultiArray, queue_size=2)
    force_thruster_pub = rospy.Publisher('force_thruster_commands', Float64MultiArray, queue_size=2)
    rospy.spin()
