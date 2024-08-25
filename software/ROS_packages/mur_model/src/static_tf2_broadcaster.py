#!/usr/bin/env python

import rospy
import tf2_ros
import tf.transformations as tf_transformations
from geometry_msgs.msg import TransformStamped

def create_transform_msg(parent_frame, child_frame, pos, orientation):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = parent_frame
    t.child_frame_id = child_frame
    t.transform.translation.x = pos[0]
    t.transform.translation.y = pos[1]
    t.transform.translation.z = pos[2]
    quat = tf_transformations.quaternion_from_euler(*orientation)
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t


def create_static_transform_msgs(device_list, base_string=None):
    # add thrusters to static broadcaster
    static_transforms = []
    for device in device_list:
        pos = device['pos']
        orientation = device['orientation']
        id_of_device = device['id']
        tf2_relative_to = device['relative_to']
        if base_string is None and 'name' not in device.keys():
            raise ValueError("Please use base_string or name in static tf2 broadcaster.")
        
        name = device['name'] if 'name' in device.keys() else f'{base_string}_{id_of_device}'

        rospy.loginfo(f"Static transform: {tf2_relative_to} -> {name}")
        t_msg = create_transform_msg(tf2_relative_to, f'{name}', pos, orientation)

        static_transforms.append(t_msg)
    return static_transforms

def broadcast_static_transforms():
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transforms = []

    model_info = rospy.get_param('/model_info')
    sensor_info = rospy.get_param('/sensor_info/sensors')

    rospy.loginfo("Broadcasting static transforms")

    # # add general static tf2 links
    # for tf2_link in model_info['static_tf2_links']:
    #     tf2_name = tf2_link['name']
    #     tf2_pos = tf2_link['pos']
    #     tf2_orientation = tf2_link['orientation']
    #     tf2_relative_to = tf2_link['relative_to']
    #     rospy.loginfo(f"Static transform: {tf2_relative_to} -> {tf2_name}")
    #     tf2_msg = create_transform_msg(tf2_relative_to, tf2_name, tf2_pos, tf2_orientation)
    #     static_transforms.append(tf2_msg)

    # add general static tf2 links
    device_list = model_info['static_tf2_links']
    new_static_transforms = create_static_transform_msgs(device_list)
    static_transforms.extend(new_static_transforms)

    # add thrusters to static broadcast
    base_string = 'thruster'
    device_list = model_info['thrusters']
    new_static_transforms = create_static_transform_msgs(device_list, base_string)
    static_transforms.extend(new_static_transforms)

    # add cameras to static broadcast
    base_string = 'camera'
    device_list = model_info['camera_data']['cameras']
    new_static_transforms = create_static_transform_msgs(device_list, base_string)
    static_transforms.extend(new_static_transforms)

    # add sensors to static broadcaster
    for sensor_name, sensor_data in sensor_info.items():
        for instance in sensor_data['instances']:
            module = instance['module']
            pos = instance['pos']
            orientation = instance['orientation']
            frame_id = f"{sensor_name}_{module}"
            rospy.loginfo(f"Static transform: fossen_base_link -> {frame_id}")
            t_msg = create_transform_msg('fossen_base_link', frame_id, pos, orientation)
            static_transforms.append(t_msg)

    # add dynamic links to static broadcast
    device_list = model_info['dynamic_tf2_links']
    new_static_transforms = create_static_transform_msgs(device_list)
    static_transforms.extend(new_static_transforms)

    # Broadcast all static transforms once
    static_broadcaster.sendTransform(static_transforms)

def broadcast_initial_transform(br):
    model_info = rospy.get_param('/model_info')

    rospy.loginfo("Broadcasting initial dynamic transforms")

    # for tf2_link in model_info['dynamic_tf2_links']:
    #     tf2_name = tf2_link['name']
    #     tf2_pos = tf2_link['pos']
    #     tf2_orientation = tf2_link['orientation']
    #     tf2_relative_to = tf2_link['relative_to']
    #     rospy.loginfo(f"Dynamic transform: {tf2_relative_to} -> {tf2_name}")
    #     t_base_msg = create_transform_msg(tf2_relative_to, tf2_name, tf2_pos, tf2_orientation)
    #     br.sendTransform(t_base_msg)

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    # Broadcast static transforms once
    broadcast_static_transforms()

    # Dynamic transform broadcaster
    dynamic_broadcaster = tf2_ros.TransformBroadcaster()

    # Broadcast the initial transform from world to base_link
    broadcast_initial_transform(dynamic_broadcaster)

    rospy.spin()
