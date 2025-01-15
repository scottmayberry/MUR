#!/usr/bin/env python
"""
ros_thruster_microsecond_udp.py

Author: Scott Mayberry
Date: 2025-01-10
Description:
    This script handles the conversion and transmission of thruster commands for the Miniature Underwater Robot (MUR).
    It subscribes to a ROS topic containing thruster commands in microseconds (PWM signals), processes these commands
    based on the robot's model configuration, and sends the processed commands via UDP to the specified thruster
    controllers. The script ensures that thruster commands are correctly ordered, normalized, and formatted
    before transmission, enabling precise control over the robot's movement and stability.

Usage:
    Ensure that the ROS master is running and the necessary parameters are set before executing this script.
    Launch the script using ROS launch or run it directly:

    Direct Execution:
        ```bash
        rosrun mur_model ros_thruster_microsecond_udp.py
        ```

    Using Launch File:
        ```bash
        roslaunch mur_model mur_model.launch
        ```

Dependencies:
    - rospy
    - numpy
    - std_msgs.msg (Float64MultiArray, Int32MultiArray)
    - geometry_msgs.msg (WrenchStamped, Vector3Stamped)
    - tf2_ros
    - tf2_geometry_msgs
    - sympy
    - socket
    - json
    - time

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
        sub (rospy.Subscriber): The ROS subscriber for receiving thruster command messages.
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
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        # Create a UDP socket for communication
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Allow the socket to reuse addresses
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # Subscribe to the 'us_thruster_commands' topic to receive thruster PWM commands
        self.sub = rospy.Subscriber('us_thruster_commands', Int32MultiArray, self.callback)

    def callback(self, msg):
        """
        Callback function triggered upon receiving thruster command messages.

        This function processes incoming thruster commands by ordering them based on thruster IDs,
        applying any necessary transformations (such as flipping commands), and sending the processed
        commands via UDP to the thruster controllers.

        Args:
            msg (Int32MultiArray): The incoming thruster commands in microseconds (PWM signals).
        """
        # Extract thruster commands from the message
        thruster_commands = msg.data
        
        # Initialize a list to hold ordered thruster commands, defaulting to zero
        ordered_thruster_commands = [0] * len(thruster_commands)
        
        # Iterate through each thruster in the model configuration to order commands
        for idx, thruster in enumerate(self.model_info['thrusters']):
            thruster_id = thruster['board_plug_id']  # Unique identifier for the thruster
            thruster_type = thruster['esc']          # ESC type associated with the thruster
            thruster_comm_zero_value = round(self.model_info['escs'][thruster_type]['zero_value'], 3)  # Zero threshold for PWM
            
            # Assign the PWM command to the correct thruster ID
            ordered_thruster_commands[thruster_id] = round(thruster_commands[thruster_id], 3)
            
            # If the thruster command needs to be flipped, adjust the PWM value accordingly
            if thruster['flip_esc_command']:
                ordered_thruster_commands[thruster_id] = 2 * thruster_comm_zero_value - ordered_thruster_commands[thruster_id]
        
        # Convert the ordered thruster commands into a JSON-formatted string with a 'thruster_us' header
        data_json = json.dumps({'thruster_us': ordered_thruster_commands}, separators=(',', ':'))
        # print(data_json)  # Debugging: Print the JSON string (temporarily commented out)
        
        # Send the JSON-formatted thruster commands via UDP to the specified IP and port
        self.sock.sendto(data_json.encode(), (self.udp_ip, self.udp_port))

def reload_module_info():
    """
    Reloads the module information from the ROS parameter server.

    Returns:
        list: A list of module dictionaries containing information about each module.
    """
    return rospy.get_param('/module_info/modules')

if __name__ == "__main__":
    """
    Entry point for the Thruster Microsecond UDP Publisher script.

    This section initializes the ROS node, retrieves necessary configuration parameters,
    waits until the relevant module information is available, and then instantiates
    the ThrusterUDPPublisher to begin processing and transmitting thruster commands.
    """
    # Initialize the ROS node named 'thruster_udp_publisher'
    rospy.init_node('thruster_udp_publisher', anonymous=True)

    modules = None
    battery_fuselage_module_info = {}
    
    # Continuously attempt to retrieve the 'mur_battery_fuselage' module information
    while "ipaddress" not in battery_fuselage_module_info.keys():
        time.sleep(5)  # Wait for 5 seconds before retrying
        modules = reload_module_info()  # Reload module information from ROS parameters
        # Attempt to find the module with the name 'mur_battery_fuselage'
        battery_fuselage_module_info = [module for module in modules if module["name"] == "mur_battery_fuselage"][0]
        # Ensure that the retrieved module information is a valid dictionary
        if not isinstance(battery_fuselage_module_info, dict):
            battery_fuselage_module_info = {}
        
    # Extract the UDP IP address and port from the module information
    udp_ip = battery_fuselage_module_info["ipaddress"]
    udp_port = battery_fuselage_module_info["ports"]["comm"]

    # Retrieve the robot's model information from ROS parameters
    model_info = rospy.get_param("/model_info")
    
    # Instantiate the ThrusterUDPPublisher with the retrieved configuration and UDP settings
    publisher = ThrusterUDPPublisher(model_info, udp_ip, udp_port)
    
    # Keep the node running and processing callbacks until ROS is shut down
    rospy.spin()
