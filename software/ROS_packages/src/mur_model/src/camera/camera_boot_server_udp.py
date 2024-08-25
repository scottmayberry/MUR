#!/usr/bin/env python
import socket
import time
import rospy
import json
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

def construct_camera_udp_json(camera):
    camera_json = json.dumps(camera)
    encoded_camera_json = camera_json.encode('utf-8')
    return encoded_camera_json

def udp_camera_boot_server(compute_module, transmit_port, cameras, secondsBetweenPings):
    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rospy.init_node('camera_boot_server', anonymous=True)
    rate = rospy.Rate(1/secondsBetweenPings)
    while not rospy.is_shutdown():
        for camera in cameras:
            udp_camera_encoded_json = construct_camera_udp_json(camera)
            sock.sendto(udp_camera_encoded_json, (compute_module['ipaddress'], transmit_port))
            time.sleep(0.1)
        rate.sleep()


# _, UDP_IP = get_Host_name_IP()
if __name__ == "__main__":
    camera_data = rospy.get_param("/model_info/camera_data")
    try:
        secondsBetweenPings = camera_data["seconds_between_camera_boot_requests"]
    except:
        secondsBetweenPings = 15
    modules = compute_module = None
    while True:
        modules = rospy.get_param("/module_info/modules")
        compute_module = [module for module in modules if module["name"] == "mur_compute_module"][0]
        if 'ipaddress' in compute_module.keys():
            break
        time.sleep(5)
    print("Starting Camera Service")
    transmit_port = compute_module['ports']['camera']
    cameras = camera_data['cameras']
    try:
        udp_camera_boot_server(compute_module, transmit_port, cameras, secondsBetweenPings)
    except rospy.ROSInterruptException:
        pass
