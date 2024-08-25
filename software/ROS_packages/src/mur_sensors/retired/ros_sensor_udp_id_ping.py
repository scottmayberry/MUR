#!/usr/bin/env python
import socket
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

def udp_server_ping(module_data_with_ip_and_port, global_udp_transmit_port=8888, server_id_message="server", secondsBetweenPings=15):

    # if type(server_id_message) is str:
    #     server_id_message = server_id_message.encode()
    server_id_message_to_send = server_id_message.encode()
    print(server_id_message_to_send)

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rospy.init_node('server_ping', anonymous=True)
    rate = rospy.Rate(1/secondsBetweenPings)
    while not rospy.is_shutdown():
        print("Time:",time.time())
        for module in module_data_with_ip_and_port:
            print("UDP target IP: %s" % module['ipaddress'])
            # server_id_message_to_send = server_id_message + "," + str(module["receivePort"])
            sock.sendto(server_id_message_to_send, (module['ipaddress'], global_udp_transmit_port))
        rate.sleep()


# _, UDP_IP = get_Host_name_IP()
if __name__ == "__main__":
    try:
        secondsBetweenPings = int(rospy.get_param("/sensor_info/server_ping_info/seconds_between_pings"))
    except:
        secondsBetweenPings = 15
    module_info = rospy.get_param("/module_info")
    module_data_with_ip_and_port = [module for module in module_info if "ipaddress" in module.keys() and "receivePort" in module.keys()]
    transmit_port = rospy.get_param("/sensor_info/server_ping_info/transmit_port")
    server_id_message = rospy.get_param("/sensor_info/server_ping_info/server_id_message")
    try:
        udp_server_ping(module_data_with_ip_and_port, transmit_port, server_id_message, secondsBetweenPings)
    except rospy.ROSInterruptException:
        pass
