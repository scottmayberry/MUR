#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import *

def initialize_sensor_publishers():
    sensors_config = rospy.get_param('/sensor_info/sensors')

    # Initialize publishers
    pubs = {}
    for sensor_name, sensor_data in sensors_config.items():
        topic_base = sensor_data['topic']
        msg_type = sensor_data['type']
        for instance in sensor_data['instances']:
            module = instance['module']
            topic = f"/sensors/{module}/{topic_base}"
            if msg_type == "Twist":
                pubs[topic] = rospy.Publisher(topic, Twist, queue_size=instance['queue'])
            elif msg_type == "Imu":
                pubs[topic] = rospy.Publisher(topic, Imu, queue_size=instance['queue'])
            elif msg_type == "Vector3":
                pubs[topic] = rospy.Publisher(topic, Vector3, queue_size=instance['queue'])
            elif msg_type == "Float32":
                pubs[topic] = rospy.Publisher(topic, Float32, queue_size=instance['queue'])
            elif msg_type == "String":
                pubs[topic] = rospy.Publisher(topic, String, queue_size=instance['queue'])
            elif msg_type == "FluidPressure":
                pubs[topic] = rospy.Publisher(topic, FluidPressure, queue_size=instance['queue'])
    
    return pubs, sensors_config
