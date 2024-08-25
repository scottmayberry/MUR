#!/usr/bin/env python
import socket
import sys
import time
import yaml
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import json
import numpy
import imufusion


# UDP_IP = ['100.70.12.132','100.70.22.102']
UDP_IP = ['100.70.12.132']
# UDP_IP = ['100.70.22.102']
UDP_PORT_IN = 8888
sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind(('', UDP_PORT_IN))
sock.settimeout(0.005)

server_id_message = "hello\n\n"
server_id_message_to_send = server_id_message.encode()
print(server_id_message_to_send)

sock = socket.socket(socket.AF_INET, # Internet
                    socket.SOCK_DGRAM) # UDP
# server_id_message_to_send = server_id_message + "," + str(module["receivePort"])
for ip_add in UDP_IP:
    print("UDP target IP: %s" % ip_add)
    sock.sendto(server_id_message_to_send, (ip_add, UDP_PORT_IN))