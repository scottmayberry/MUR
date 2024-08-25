#!/usr/bin/env python
import socket
import rospy
import json
from std_msgs.msg import Float64MultiArray, Int32MultiArray
import time

class ThrusterUDPPublisher:
    def __init__(self, model_info, udp_ip, udp_port):
        self.model_info = model_info
        self.num_thrusters = len(self.model_info['thrusters'])
        self.active_thruster_idx = 0
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sub = rospy.Subscriber('us_thruster_commands', Int32MultiArray, self.callback)
        self.thruster_start_time = time.time()
        self.thruster_increment_time = 5
        self.thruster_active_value = 1550


    def callback(self, msg):
        # Convert the thruster commands to a string format for UDP transmission

        # Get the thruster commands from the message
        thruster_commands = msg.data
        
        # Reorder the thruster commands based on the id indexes
        ordered_thruster_commands = [0] * len(thruster_commands)
        for idx, thruster in enumerate(self.model_info['thrusters']):
            thruster_id = int(thruster['id'])
            print(thruster_id)
            thruster_type = thruster['esc']
            thruster_comm_zero_value = round(self.model_info['escs'][thruster_type]['zero_value'], 3)
            thruster_comm = thruster_comm_zero_value
            if idx == self.active_thruster_idx:
                thruster_comm = round(self.thruster_active_value, 3)
                if thruster['flip_esc_command']:
                    thruster_comm = 2*thruster_comm_zero_value - thruster_comm
            ordered_thruster_commands[thruster_id] = thruster_comm

        if time.time() > self.thruster_start_time + self.thruster_increment_time:
            self.thruster_start_time = time.time()
            self.active_thruster_idx += 1
            self.active_thruster_idx = self.active_thruster_idx%self.num_thrusters
        
        
        # Convert the dictionary to a JSON format with a 'thruster' header
        data_json = json.dumps({'thruster_us': ordered_thruster_commands}, separators=(',', ':'))
        print('Active Thruster', self.active_thruster_idx, 'Thruster US Commands', data_json)
        # print(data_json)
        self.sock.sendto(data_json.encode(), (self.udp_ip, self.udp_port))

if __name__ == "__main__":
    rospy.init_node('thruster_udp_publisher', anonymous=True)
    
    module_info = rospy.get_param("/module_info/modules")
    battery_fuselage_module_info = [module for module in module_info if module["name"] == "mur_battery_fuselage"][0]
    
    udp_ip = battery_fuselage_module_info["ipaddress"]
    udp_port = battery_fuselage_module_info["receivePort"]

    model_info = rospy.get_param("/model_info")
    
    publisher = ThrusterUDPPublisher(model_info, udp_ip, udp_port)
    
    rospy.spin()