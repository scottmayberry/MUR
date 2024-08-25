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

def convert_string_to_byteMultiArray(message):
    mal = MultiArrayLayout()
    mal.dim = [MultiArrayDimension()]
    byte_array = ByteMultiArray()
    byte_array.layout = mal
    if type(message) is str:
        message = message.encode()
    byte_array.data = message
    return byte_array


def message_to_transmit_to_bluebuzz_bytes(data):
    try:
        message_to_transmit.append(bytes(data.data))
    except:
        print("bytes message error")

def message_to_transmit_to_bluebuzz_string(data):
    try:
        message_to_transmit.append((data.data).encode())
    except:
        print("string message error")

def is_config_message_valid(message):
    messKeys = message.keys()
    if "volume" in messKeys or "baud" in messKeys or "rscodec" in messKeys or "messageBreakLength" in messKeys or "response" in messKeys:
        return True
    return False

def message_to_config_bluebuzz(data):
    global volume, baud, rscodec, messageBreakLength
    try:
        config_message = data.data
        if not is_config_message_valid(config_message):
            return
        config_message["bluebuzz"] = True

        # update stored values
        if "volume" in config_message.keys():
            volume = config_message["volume"]
        if "baud" in config_message.keys():
            baud = config_message["baud"]
        if "rscodec" in config_message["rscodec"]:
            rscodec = config_message["rscodec"]
        if "messageBreakLength" in config_message["messageBreakLength"]:
            messageBreakLength = config_message["messageBreakLength"]

        config_json = json.dumps(config_message).encode()
        message_to_config.append(config_json)
    except:
        print("config message error")


def bluebuzz_loop(bluebuzz_ip_address, bluebuzz_receive_port, global_udp_transmit_port=8888, sample_rate = 20):
    global message_to_transmit, message_to_config

    # connect to socket
    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', bluebuzz_receive_port))
    sock.settimeout(0.005)

    rospy.init_node('bluebuzz', anonymous=True)
    pub = rospy.Publisher('received', ByteMultiArray, queue_size=2)
    rospy.Subscriber('transmit/bytes', ByteMultiArray, message_to_transmit_to_bluebuzz_bytes, queue_size=2)
    rospy.Subscriber('transmit/string', String, message_to_transmit_to_bluebuzz_string, queue_size=2)
    rospy.Subscriber('config', String, message_to_config_bluebuzz, queue_size=2)
    rate = rospy.Rate(sample_rate) # 10hz
    
    while not rospy.is_shutdown():
        # if there is an availble message, transmit it to the modem
        if len(message_to_transmit) > 0:
            bluebuzz_transmit_message = message_to_transmit.pop()
            sock.sendto(bluebuzz_transmit_message, (bluebuzz_ip_address, global_udp_transmit_port))
        # if there is an available message to config, transmit to modem
        if len(message_to_config) > 0:
            config_message = message_to_config.pop()
            sock.sendto(config_message, (bluebuzz_ip_address, global_udp_transmit_port))
        # cut excess messages, save the most recent messages
        if len(message_to_transmit) > 2:
            message_to_transmit = message_to_transmit[:2]
        if len(message_to_config) > 2:
            message_to_config = message_to_config[:2]

        try:
            data, addr = sock.recvfrom(2048)
            data_message_as_bytemultiarray = convert_string_to_byteMultiArray(data)
            pub.publish(data_message_as_bytemultiarray)
        except:
            pass
        rate.sleep()

# socket.write()
    
message_to_transmit = []
message_to_config = []
volume = 0.1
baud = 200
rscodec = 10
messageBreakLength = 32

# _, UDP_IP = get_Host_name_IP()
if __name__ == "__main__":
    try:
        module_info = rospy.get_param("/module_info")
        bluebuzz_module = [module for module in module_info if module["name"] == "bluebuzz"][0]
        bluebuzz_module_ip = bluebuzz_module['ipaddress']

        bluebuzz_info = rospy.get_param("/bluebuzz_info")
        bluebuzz_receive_port = bluebuzz_info['receive_port']
        transmit_port = bluebuzz_info['transmit_port']
        sample_rate = bluebuzz_info["sample_rate"]

        # set bluebuzz variables: volume, baud, rscodec, messageBreakLength
        volume = bluebuzz_info["volume"]
        baud = bluebuzz_info["baud"]
        rscodec = bluebuzz_info["rscodec"]
        messageBreakLength = bluebuzz_info["messageBreakLength"]
        bluebuzz_loop(bluebuzz_module_ip, bluebuzz_receive_port, transmit_port, sample_rate)
    except rospy.ROSInterruptException:
        pass
