#!/usr/bin/env python
import socket
import sys
import time
import yaml
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *


def convert_string_to_byteMultiArray(message):
    dim = MultiArrayDimension()
    mal = MultiArrayLayout()
    mal.dim = [MultiArrayDimension()]
    byte_array = ByteMultiArray()
    byte_array.layout = mal
    if type(message) is str:
        message = message.encode()
    byte_array.data = message
    return byte_array


def loop_test():
    rospy.init_node('tester', anonymous=True)
    pub = rospy.Publisher('transmit/bytes', ByteMultiArray, queue_size=2)
    pub2 = rospy.Publisher('transmit/string', String, queue_size=2)
    rate = rospy.Rate(0.05) # 10hz
    
    while not rospy.is_shutdown():
        appendTime = int(time.time())%1000
        # pub.publish(b"hello")
        # dim = MultiArrayDimension()
        # dim.label = "width"
        # dim.size = 100
        # dim.stride = 0
        # mal = MultiArrayLayout()
        # mal.dim = [dim]
        # mess = ByteMultiArray()
        # mess.layout = mal
        # mess.data = ("hello" + str(appendTime)).encode()
        mess = convert_string_to_byteMultiArray("hello" + str(appendTime))
        # pub.publish(mess)
        # pub2.publish("hello world")
        # print("published")
        rate.sleep()


# _, UDP_IP = get_Host_name_IP()
if __name__ == "__main__":
    try:
        loop_test()
    except rospy.ROSInterruptException:
        pass
