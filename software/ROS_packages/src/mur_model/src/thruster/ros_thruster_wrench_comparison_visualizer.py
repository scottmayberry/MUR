#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped, TransformStamped
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import threading
import queue
import tf2_ros
import time
from tf.transformations import quaternion_matrix

# Create three subplots: one for thrusters, one for torques, and one for linear forces
fig = plt.figure(figsize=(15, 5))
ax_thruster = fig.add_subplot(131, projection='3d', title='Thrusters')
ax_linear = fig.add_subplot(132, projection='3d', title='Linear Forces')
ax_torque = fig.add_subplot(133, projection='3d', title='Torques')

force_thruster_commands = None
update_queue = queue.Queue()
wrench_queue = queue.Queue()
lengthOfQuiver = 0.01
torque_queue = queue.Queue()


def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..'''
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def update_thruster_plot(thruster_data, com_pos, wrench_pos, wrench_force):
    global lengthOfQuiver
    ax = ax_thruster  # Use the linear forces subplot
    # Initialize total force vector
    total_force = np.array([0.0, 0.0, 0.0])

    # Draw the first thruster force with a label (if needed)
    pos, force_vector = thruster_data[0]
    if ax.get_legend() is None or 'Thruster Forces' not in [l.get_label() for l in ax.get_legend().get_lines()]:
        ax.quiver(pos[0], pos[1], pos[2], force_vector[0], force_vector[1], force_vector[2], color='b', length=lengthOfQuiver, label='Thruster Forces')
    else:
        ax.quiver(pos[0], pos[1], pos[2], force_vector[0], force_vector[1], force_vector[2], color='b', length=lengthOfQuiver)
    total_force += force_vector

    # Draw the remaining thruster forces without a label
    for pos, force_vector in thruster_data[1:]:
        ax.quiver(pos[0], pos[1], pos[2], force_vector[0], force_vector[1], force_vector[2], length=lengthOfQuiver, color='b')
        total_force += force_vector

    # # Draw the wrench force
    # if ax.get_legend() is None or 'Wrench Force' not in [l.get_label() for l in ax.get_legend().get_lines()]:
    #     ax.quiver(wrench_pos[0], wrench_pos[1], wrench_pos[2], wrench_force[0], wrench_force[1], wrench_force[2], color='g', length=lengthOfQuiver, label='Wrench Force')
    # else:
    #     ax.quiver(wrench_pos[0], wrench_pos[1], wrench_pos[2], wrench_force[0], wrench_force[1], wrench_force[2], color='g', length=lengthOfQuiver)

    # # If com_pos is not None, draw the total force vector at the com position
    # if com_pos is not None:
    #     if ax.get_legend() is None or 'Total Force' not in [l.get_label() for l in ax.get_legend().get_lines()]:
    #         ax.quiver(com_pos[0], com_pos[1], com_pos[2], total_force[0], total_force[1], total_force[2], color='r', alpha=0.6, length=lengthOfQuiver, label='Total Force')
    #     else:
    #         ax.quiver(com_pos[0], com_pos[1], com_pos[2], total_force[0], total_force[1], total_force[2], color='r', alpha=0.6, length=lengthOfQuiver)

    set_axes_equal(ax)  # Ensure axes are equal after each update

    # Set the axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # Add legend
    ax.legend()
        

def compute_torque(pos, force_vector, com_pos):
    """Compute the torque given the position and force vector of a thruster."""
    r = pos - com_pos  # Position vector from center of mass to thruster
    return np.cross(r, force_vector)


def update_torque_plot(thruster_data, com_pos, wrench_pos, wrench_torque):
    global lengthOfQuiver
    ax = ax_torque  # Use the torques subplot

    # Compute the total torque from all thrusters
    total_torque = np.array([0.0, 0.0, 0.0])
    for pos, force_vector in thruster_data:
        total_torque += compute_torque(pos, force_vector, com_pos)

    # Draw the total torque from thrusters
    ax.quiver(0, 0, 0, total_torque[0], total_torque[1], total_torque[2], color='r', alpha=0.5, label='Thruster Torque', length=lengthOfQuiver)

    # Draw the wrench torque
    ax.quiver(0, 0, 0, wrench_torque[0], wrench_torque[1], wrench_torque[2], color='g', alpha=0.5, label='Wrench Torque', length=lengthOfQuiver)

    # Set the axes labels
    ax.set_xlabel('Roll')
    ax.set_ylabel('Pitch')
    ax.set_zlabel('Yaw')

    set_axes_equal(ax)  # Ensure axes are equal after each update

    # Add legend
    ax.legend()

def update_linear_plot(thruster_data, com_pos, wrench_pos, wrench_force):
    global lengthOfQuiver
    ax = ax_linear  # Use the linear forces subplot

    # Compute the total force from all thrusters
    total_force = np.array([0.0, 0.0, 0.0])
    for pos, force_vector in thruster_data:
        total_force += force_vector

    # Draw the total force from thrusters
    ax.quiver(0, 0, 0, total_force[0], total_force[1], total_force[2], color='r', alpha=0.5, label='Thruster Force', length=lengthOfQuiver)

    # Draw the wrench force
    ax.quiver(0, 0, 0, wrench_force[0], wrench_force[1], wrench_force[2], color='g', alpha=0.5, label='Wrench Force', length=lengthOfQuiver)

    # Set the axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    set_axes_equal(ax)  # Ensure axes are equal after each update

    # Add legend
    ax.legend()


def update_plots():
    plt.tight_layout()
    while not rospy.is_shutdown():
        time.sleep(0.02)
        if wrench_queue.empty() or update_queue.empty():
            continue
        thruster_data, com_pos = update_queue.get()
        wrench_pos, wrench_force, wrench_torque = wrench_queue.get()
        ax_thruster.clear()  # Clear the thrusters subplot
        ax_linear.clear()    # Clear the linear forces subplot
        ax_torque.clear()    # Clear the torques subplot
        update_thruster_plot(thruster_data, com_pos, wrench_pos, wrench_force)
        update_linear_plot(thruster_data, com_pos, wrench_pos, wrench_force)
        update_torque_plot(thruster_data, com_pos, wrench_pos, wrench_torque)
        plt.draw()
        plt.pause(0.01)

def force_thruster_commands_callback(msg):
    if not update_queue.empty():
        return
    force_thruster_commands = np.array(msg.data)
    thruster_frames = [f'thruster_{i}' for i in range(len(force_thruster_commands))]
    
    thruster_data = []
    for idx, thruster_frame in enumerate(thruster_frames):
        try:
            transform = tf_buffer.lookup_transform('base_link', thruster_frame, rospy.Time(0))
            pos = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
            
            # Get the rotation matrix from the quaternion
            quat = transform.transform.rotation
            rotation_matrix = quaternion_matrix([quat.x, quat.y, quat.z, quat.w])
            
            # Rotate the force vector to align with the thruster's x-axis
            force_magnitude = force_thruster_commands[idx]
            force_vector = np.dot(rotation_matrix, np.array([force_magnitude, 0, 0, 1]))[:3]
            
            thruster_data.append((pos, force_vector))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Error getting transform for {thruster_frame}: {e}")

    # Fetch the com position
    try:
        transform_com = tf_buffer.lookup_transform('base_link', 'com', rospy.Time(0))
        com_pos = np.array([transform_com.transform.translation.x, transform_com.transform.translation.y, transform_com.transform.translation.z])
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Error getting transform for com: {e}")
        com_pos = None

    update_queue.put((thruster_data, com_pos))

def body_frame_wrench_callback(msg):
    if not wrench_queue.empty():
        return
    try:
        # Get the position of the frame_id with respect to base_link
        transform = tf_buffer.lookup_transform('base_link', msg.header.frame_id, rospy.Time(0))
        pos = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
        
        # Extract the linear force components
        force = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        torque = np.array([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        
        # Add the data to the wrench queue
        wrench_queue.put((pos, force, torque))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        rospy.logwarn(f"Error getting transform for {msg.header.frame_id}: {e}")


if __name__ == '__main__':
    rospy.init_node('thruster_command_visualizer_node')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(2)

    # Subscribers
    rospy.Subscriber('force_thruster_commands', Float64MultiArray, force_thruster_commands_callback)
    rospy.Subscriber('requested_thruster_wrench', WrenchStamped, body_frame_wrench_callback)

    # Start the plotting in the main thread
    update_plots()