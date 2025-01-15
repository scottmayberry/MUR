#!/usr/bin/env python
"""
ros_thruster_to_id_microsecond_udp.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script manages the conversion and transmission of thruster commands for the Miniature Underwater Robot (MUR).
    It subscribes to a ROS topic containing thruster commands in microseconds (PWM signals), processes these commands
    based on the robot's model configuration, and sends the processed commands via UDP to the specified thruster
    controllers. The script ensures that thruster commands are correctly ordered, normalized, and formatted
    before transmission, enabling precise control over the robot's movement and stability.

    The script performs the following key functions:
        1. Subscribes to the 'us_thruster_commands' ROS topic to receive thruster PWM commands.
        2. Orders the thruster commands based on thruster IDs and applies necessary transformations.
        3. Sends the ordered thruster commands via UDP to the designated thruster controllers.
        4. Handles thruster identification by cycling through active thrusters at specified intervals.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model ros_thruster_to_id_microsecond_udp.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - socket
    - json
    - time
    - std_msgs.msg (Float64MultiArray, Int32MultiArray)

License:
    MIT License
"""

import socket
import rospy
import json
import time
from std_msgs.msg import Float64MultiArray, Int32MultiArray

class ThrusterUDPPublisher:
    """
    ThrusterUDPPublisher handles the subscription to thruster command topics and
    the transmission of processed thruster commands via UDP.

    Attributes:
        model_info (dict): Configuration information for the robot's model, including thruster details.
        udp_ip (str): The IP address of the thruster controller to send UDP messages to.
        udp_port (int): The UDP port number of the thruster controller.
        sock (socket.socket): The UDP socket used for sending thruster commands.
        num_thrusters (int): The total number of thrusters configured.
        active_thruster_idx (int): The index of the currently active thruster for identification.
        thruster_start_time (float): Timestamp marking the start time of the current thruster identification cycle.
        thruster_increment_time (int): Time interval (in seconds) after which the active thruster index increments.
        thruster_active_value (int): The PWM value assigned to the active thruster during identification.
    """
    def __init__(self, model_info, udp_ip, udp_port):
        """
        Initializes the ThrusterUDPPublisher with the given model information and UDP settings.

        Args:
            model_info (dict): Configuration information for the robot's model, including thruster details.
            udp_ip (str): The IP address of the thruster controller to send UDP messages to.
            udp_port (int): The UDP port number of the thruster controller.
        """
        self.model_info = model_info
        self.num_thrusters = len(self.model_info['thrusters'])  # Total number of thrusters
        self.active_thruster_idx = 0  # Index of the currently active thruster
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        # Initialize a UDP socket for communication
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # self.sub = rospy.Subscriber('us_thruster_commands', Int32MultiArray, self.callback)  # Temporarily commented out subscriber
        self.thruster_start_time = time.time()  # Record the start time for thruster identification
        self.thruster_increment_time = 7  # Time interval (in seconds) to switch active thruster
        self.thruster_active_value = 1550  # PWM value to activate the thruster

        # Start the thruster identification process
        self.run_thruster_identification()

    def run_thruster_identification(self):
        """
        Continuously identifies and activates thrusters in a cyclic manner for testing or calibration.

        This function runs in a loop, periodically activating each thruster by sending a specific PWM value
        while ensuring that only one thruster is active at a time. It constructs the thruster command message
        in JSON format and sends it via UDP to the specified thruster controller.
        """
        while not rospy.is_shutdown():
            # Initialize a list to hold ordered thruster commands, defaulting to zero
            ordered_thruster_commands = [0] * self.num_thrusters

            # Iterate through each thruster to assign PWM values
            for idx, thruster in enumerate(self.model_info['thrusters']):
                thruster_type = thruster['esc']  # ESC type associated with the thruster
                thruster_comm_zero_value = round(self.model_info['escs'][thruster_type]['zero_value'], 3)  # Zero PWM value
                ordered_thruster_commands[idx] = round(self.thruster_active_value, 3)  # Assign active PWM value

            ordered_thruster_commands[self.active_thruster_idx] = round(self.thruster_active_value, 3)
            
            # Temporarily commented-out code for writing zeros to all thrusters
            # # write zeros to all thrusters
            # for idx, thruster in enumerate(self.model_info['thrusters']):
            #     thruster_id = int(thruster['board_plug_id'])
            #     print(thruster_id)
            #     thruster_type = thruster['esc']
            #     thruster_comm_zero_value = round(self.model_info['escs'][thruster_type]['zero_value'], 3)
            #     thruster_comm = thruster_comm_zero_value
            #     if idx == self.active_thruster_idx:
            #         thruster_comm = round(self.thruster_active_value, 3)
            #         if thruster['flip_esc_command']:
            #             thruster_comm = 2*thruster_comm_zero_value - thruster_comm
            #     ordered_thruster_commands[thruster_id] = thruster_comm

            # Check if it's time to increment the active thruster index
            if time.time() > self.thruster_start_time + self.thruster_increment_time:
                self.thruster_start_time = time.time()  # Reset the start time
                self.active_thruster_idx += 1  # Move to the next thruster
                self.active_thruster_idx = self.active_thruster_idx % self.num_thrusters  # Cycle back if necessary

            # Convert the thruster commands into a JSON-formatted string with a 'thruster_us' header
            data_json = json.dumps({'thruster_us': ordered_thruster_commands}, separators=(',', ':'))
            print('Active Thruster', self.active_thruster_idx, 'Thruster US Commands', data_json)  # Debugging output
            # print(data_json)  # Temporarily commented out print statement for raw JSON

            # Send the JSON-formatted thruster commands via UDP to the specified IP and port
            self.sock.sendto(data_json.encode(), (self.udp_ip, self.udp_port))
            time.sleep(0.05)  # Short sleep to prevent overwhelming the network

def reload_module_info():
    """
    Reloads the module information from the ROS parameter server.

    Returns:
        list: A list of module dictionaries containing information about each module.
    """
    return rospy.get_param('/module_info/modules')

if __name__ == "__main__":
    """
    Entry point for the Thruster to ID Microsecond UDP Publisher script.

    This section initializes the ROS node, retrieves necessary configuration parameters,
    waits until the relevant module information is available, and then instantiates
    the ThrusterUDPPublisher to begin processing and transmitting thruster commands.
    """
    # Initialize the ROS node named 'thruster_udp_publisher'
    rospy.init_node('thruster_udp_publisher', anonymous=True)
    
    module_info = rospy.get_param("/module_info/modules")
    battery_fuselage_module_info = [module for module in module_info if module["name"] == "mur_battery_fuselage"][0]

    while "ipaddress" not in battery_fuselage_module_info.keys():
        time.sleep(5)
        modules = reload_module_info()
        battery_fuselage_module_info = [module for module in modules if module["name"] == "mur_battery_fuselage"][0]
        if not isinstance(battery_fuselage_module_info, dict):
            battery_fuselage_module_info = {}
    
    udp_ip = battery_fuselage_module_info["ipaddress"]
    udp_port = battery_fuselage_module_info["ports"]["comm"]

    model_info = rospy.get_param("/model_info")
    
    publisher = ThrusterUDPPublisher(model_info, udp_ip, udp_port)
    
    rospy.spin()
