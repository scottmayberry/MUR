#!/usr/bin/env python
"""
static_tf2_broadcaster.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script broadcasts both static and dynamic transforms for the Miniature Underwater Robot (MUR)
    using ROS's TF2 library. It reads configuration parameters from the ROS parameter server,
    constructs TransformStamped messages for various components (such as thrusters, cameras, and sensors),
    and broadcasts these transforms to establish the spatial relationships between different frames
    of reference within the robot's model.

    The script ensures that all fixed components have their transforms broadcasted once at startup,
    while dynamic transforms can be handled separately if needed. This setup is essential for accurate
    localization, navigation, and sensor data interpretation within the MUR system.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    You can launch the script directly or use the provided ROS launch file:

    Direct Execution:
        ```bash
        rosrun mur_model static_tf2_broadcaster.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - tf2_ros
    - tf.transformations
    - geometry_msgs.msg (TransformStamped)

License:
    MIT License
"""

import rospy
import tf2_ros
import tf.transformations as tf_transformations
from geometry_msgs.msg import TransformStamped

def create_transform_msg(parent_frame, child_frame, pos, orientation):
    """
    Creates a TransformStamped message with the given parameters.

    Args:
        parent_frame (str): The name of the parent frame.
        child_frame (str): The name of the child frame.
        pos (list or tuple): The [x, y, z] translation coordinates.
        orientation (list or tuple): The [roll, pitch, yaw] Euler angles in radians.

    Returns:
        TransformStamped: The constructed TransformStamped message.
    """
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()  # Timestamp of the transform
    t.header.frame_id = parent_frame   # Parent frame reference
    t.child_frame_id = child_frame     # Child frame reference
    t.transform.translation.x = pos[0] # X-coordinate translation
    t.transform.translation.y = pos[1] # Y-coordinate translation
    t.transform.translation.z = pos[2] # Z-coordinate translation
    quat = tf_transformations.quaternion_from_euler(*orientation)  # Convert Euler angles to quaternion
    t.transform.rotation.x = quat[0] # Quaternion X component
    t.transform.rotation.y = quat[1] # Quaternion Y component
    t.transform.rotation.z = quat[2] # Quaternion Z component
    t.transform.rotation.w = quat[3] # Quaternion W component
    return t

def create_static_transform_msgs(device_list, base_string=None):
    """
    Creates a list of TransformStamped messages for a given list of devices.

    Args:
        device_list (list): A list of dictionaries, each containing device information such as position,
                            orientation, ID, and the frame it is relative to.
        base_string (str, optional): A base string to prefix the device name. Defaults to None.

    Raises:
        ValueError: If neither base_string nor device name is provided for a device.

    Returns:
        list: A list of TransformStamped messages.
    """
    # Initialize an empty list to store TransformStamped messages
    static_transforms = []
    
    # Iterate through each device in the provided device list
    for device in device_list:
        pos = device['pos']                  # Position of the device relative to the parent frame
        orientation = device['orientation']  # Orientation of the device in Euler angles
        # Determine the device ID, prefer 'id' over 'board_plug_id' if available
        id_of_device = device['id'] if 'id' in device.keys() else device['board_plug_id']
        tf2_relative_to = device['relative_to']  # Parent frame reference
        
        # Ensure that either a base_string or device name is provided
        if base_string is None and 'name' not in device.keys():
            raise ValueError("Please use base_string or name in static tf2 broadcaster.")
        
        # Determine the device name either from the 'name' key or by using the base_string and ID
        name = device['name'] if 'name' in device.keys() else f'{base_string}_{id_of_device}'

        # Log the creation of the static transform for traceability
        rospy.loginfo(f"Static transform: {tf2_relative_to} -> {name}")
        # Create the TransformStamped message using the provided parameters
        t_msg = create_transform_msg(tf2_relative_to, f'{name}', pos, orientation)

        # Append the created transform message to the list
        static_transforms.append(t_msg)
    
    return static_transforms

def broadcast_static_transforms():
    """
    Broadcasts all static transforms defined in the model and sensor configurations.

    This function performs the following steps:
        1. Initializes a StaticTransformBroadcaster.
        2. Retrieves model and sensor information from ROS parameters.
        3. Creates TransformStamped messages for static links defined in the model.
        4. Adds thrusters, cameras, and sensors to the static transform broadcaster.
        5. Sends all static transforms once to establish frame relationships.
    """
    # Initialize a StaticTransformBroadcaster to handle static transforms
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    # Initialize an empty list to store all TransformStamped messages
    static_transforms = []

    # Retrieve model and sensor information from ROS parameters
    model_info = rospy.get_param('/model_info')
    sensor_info = rospy.get_param('/sensor_info/sensors')

    rospy.loginfo("Broadcasting static transforms")

    # Add general static TF2 links defined in the model configuration
    for tf2_link in model_info['static_tf2_links']:
        tf2_name = tf2_link['name']                  # Name of the child frame
        tf2_pos = tf2_link['pos']                    # Position [x, y, z] of the child frame
        tf2_orientation = tf2_link['orientation']    # Orientation [roll, pitch, yaw] in radians
        tf2_relative_to = tf2_link['relative_to']    # Parent frame reference
        rospy.loginfo(f"Static transform: {tf2_relative_to} -> {tf2_name}")
        # Create the TransformStamped message for the static link
        tf2_msg = create_transform_msg(tf2_relative_to, tf2_name, tf2_pos, tf2_orientation)
        # Append the transform message to the list
        static_transforms.append(tf2_msg)

    # The following code is temporarily commented out and can be used for alternative static transform addition
    # # add general static tf2 links
    # device_list = model_info['static_tf2_links']
    # new_static_transforms = create_static_transform_msgs(device_list)
    # static_transforms.extend(new_static_transforms)

    # Add thrusters to the static transform broadcaster with a base string 'thruster'
    base_string = 'thruster'
    device_list = model_info['thrusters']
    # Create TransformStamped messages for each thruster and add to the list
    new_static_transforms = create_static_transform_msgs(device_list, base_string)
    static_transforms.extend(new_static_transforms)

    # Add cameras to the static transform broadcaster with a base string 'camera'
    base_string = 'camera'
    device_list = model_info['camera_data']['cameras']
    # Create TransformStamped messages for each camera and add to the list
    new_static_transforms = create_static_transform_msgs(device_list, base_string)
    static_transforms.extend(new_static_transforms)

    # Add sensors to the static transform broadcaster, relative to 'fossen_base_link'
    for sensor_name, sensor_data in sensor_info.items():
        for instance in sensor_data['instances']:
            module = instance['module']              # Module name associated with the sensor
            pos = instance['pos']                    # Position [x, y, z] of the sensor relative to base_link
            orientation = instance['orientation']    # Orientation [roll, pitch, yaw] in radians
            frame_id = f"{sensor_name}_{module}"     # Frame ID for the sensor
            rospy.loginfo(f"Static transform: fossen_base_link -> {frame_id}")
            # Create the TransformStamped message for the sensor
            t_msg = create_transform_msg('fossen_base_link', frame_id, pos, orientation)
            # Append the transform message to the list
            static_transforms.append(t_msg)

    # The following code is temporarily commented out and can be used for adding dynamic links
    # # add dynamic links to static broadcast
    # device_list = model_info['dynamic_tf2_links']
    # new_static_transforms = create_static_transform_msgs(device_list)
    # static_transforms.extend(new_static_transforms)

    # Add dynamic static TF2 links defined in the model configuration
    for tf2_link in model_info['dynamic_tf2_links']:
        tf2_name = tf2_link['name']                  # Name of the child frame
        tf2_pos = tf2_link['pos']                    # Position [x, y, z] of the child frame
        tf2_orientation = tf2_link['orientation']    # Orientation [roll, pitch, yaw] in radians
        tf2_relative_to = tf2_link['relative_to']    # Parent frame reference
        rospy.loginfo(f"Static transform: {tf2_relative_to} -> {tf2_name}")
        # Create the TransformStamped message for the dynamic static link
        tf2_msg = create_transform_msg(tf2_relative_to, tf2_name, tf2_pos, tf2_orientation)
        # Append the transform message to the list
        static_transforms.append(tf2_msg)

    # Broadcast all static transforms once to establish frame relationships
    static_broadcaster.sendTransform(static_transforms)
    rospy.loginfo("Static transforms broadcasted successfully")

def broadcast_initial_transform(br):
    """
    Broadcasts initial dynamic transforms. (Currently Placeholder)

    Args:
        br (tf2_ros.TransformBroadcaster): The broadcaster used to send dynamic transforms.
    """
    model_info = rospy.get_param('/model_info')

    rospy.loginfo("Broadcasting initial dynamic transforms")

    # The following code is temporarily commented out and serves as a placeholder for future dynamic transform broadcasting
    # for tf2_link in model_info['dynamic_tf2_links']:
    #     tf2_name = tf2_link['name']
    #     tf2_pos = tf2_link['pos']
    #     tf2_orientation = tf2_link['orientation']
    #     tf2_relative_to = tf2_link['relative_to']
    #     rospy.loginfo(f"Dynamic transform: {tf2_relative_to} -> {tf2_name}")
    #     t_base_msg = create_transform_msg(tf2_relative_to, tf2_name, tf2_pos, tf2_orientation)
    #     br.sendTransform(t_base_msg)

if __name__ == '__main__':
    """
    Entry point for the Static TF2 Broadcaster script.

    Initializes the ROS node, broadcasts all static transforms, and sets up dynamic transform broadcasting
    (currently a placeholder). The node runs indefinitely until ROS is shut down.
    """
    # Initialize the ROS node named 'tf_broadcaster'
    rospy.init_node('tf_broadcaster')

    # Broadcast all static transforms once to establish fixed frame relationships
    broadcast_static_transforms()

    # Initialize a TransformBroadcaster for dynamic transforms (currently not used)
    dynamic_broadcaster = tf2_ros.TransformBroadcaster()

    # Broadcast the initial transform from world to base_link (currently a placeholder)
    broadcast_initial_transform(dynamic_broadcaster)

    # Keep the node running until ROS is shut down
    rospy.spin()
