#!/usr/bin/env python3
"""
broadcast_ip_data.py

Author: Scott Mayberry
Date: 2025-03-02

Description:
    This script broadcasts network configuration data over UDP. It retrieves the MAC address, IP address,
    and subnet mask for a specified network interface, calculates the corresponding broadcast address, and
    then continuously sends a compressed JSON message containing these details along with designated ports.
    This is useful for network discovery or configuration setups within a local network.

    Key Functionalities:
        1. Retrieves the MAC address from the system for the specified network interface.
        2. Obtains the IP address and subnet mask using the netifaces library.
        3. Calculates the broadcast address by applying bitwise operations on the IP address and netmask.
        4. Constructs a JSON message with network details including IP, MAC, broadcast and communication ports.
        5. Broadcasts the JSON message periodically over UDP to the computed broadcast address.
        6. Logs successful transmissions and errors for troubleshooting.

Dependencies:
    - json: To encode the broadcast data into JSON format.
    - socket: For creating UDP sockets and handling network communication.
    - time: For introducing delays between broadcasts.
    - netifaces: To obtain network interface details such as IP address and netmask.
    - struct: For converting IP addresses between binary and string representations.
    - logging: For logging informational and error messages.

License:
    MIT License
"""

import json
import socket
import time
import netifaces
import struct
import logging

class BroadcastIPData:
    def __init__(self, network_interface='eth0', broadcast_port=51584, comm_port=51585):
        # Initialize network parameters.
        self.network_interface = network_interface
        self.broadcast_port = broadcast_port
        self.comm_port = comm_port
        
        # Retrieve the MAC address from the system file for the specified network interface.
        self.mac_address = self.get_mac_address()
        # Retrieve the IP address and netmask using the netifaces library.
        self.ip_address, self.netmask = self.get_ip_and_subnet()

        # If IP address or netmask retrieval fails, log an error and exit.
        if self.ip_address is None or self.netmask is None:
            logging.error("Failed to retrieve IP address or netmask. Exiting.")
            exit(1)

        # Calculate the broadcast address based on the IP address and netmask.
        self.broadcast_address = self.calculate_broadcast_address()
        # Construct the data dictionary to be broadcasted.
        self.data = {
            "id": 0,
            "serverRequestForIP": self.ip_address,
            "mac": self.mac_address,
            "BROADCAST_PORT": self.broadcast_port,
            "COMM_PORT": self.comm_port
        }

    def get_mac_address(self):
        """
        Reads and returns the MAC address of the network interface from the system file.
        """
        try:
            with open(f'/sys/class/net/{self.network_interface}/address') as f:
                mac = f.read().strip()
            return mac
        except FileNotFoundError:
            logging.error(f"Could not find MAC address for interface {self.network_interface}")
            return None

    def get_ip_and_subnet(self):
        """
        Retrieves the IP address and subnet mask for the network interface using netifaces.
        """
        try:
            iface_data = netifaces.ifaddresses(self.network_interface)[netifaces.AF_INET][0]
            ip_address = iface_data['addr']
            netmask = iface_data['netmask']
            return ip_address, netmask
        except (KeyError, IndexError):
            logging.error(f"Could not find IP address or netmask for interface {self.network_interface}")
            return None, None

    def calculate_broadcast_address(self):
        """
        Calculates the broadcast address using bitwise operations on the IP address and subnet mask.
        """
        # Convert the IP address and netmask to their 32-bit binary representations.
        ip_bin = struct.unpack('!I', socket.inet_aton(self.ip_address))[0]
        netmask_bin = struct.unpack('!I', socket.inet_aton(self.netmask))[0]
        # Compute the broadcast address in binary form.
        broadcast_bin = ip_bin | ~netmask_bin
        # Convert the binary broadcast address back to dotted-quad string format,
        # ensuring the result is treated as a 32-bit unsigned integer.
        broadcast_address = socket.inet_ntoa(struct.pack('!I', broadcast_bin & 0xFFFFFFFF))
        return broadcast_address

    def send_data(self):
        """
        Continuously sends the JSON encoded data over UDP to the broadcast address.
        """
        # Create a UDP socket and enable the broadcast option.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            # Infinite loop to broadcast data periodically.
            while True:
                try:
                    # Encode the data dictionary into a compact JSON string and then to bytes.
                    compressed_data = json.dumps(self.data, separators=(',', ':')).encode('utf-8')
                    # Send the JSON data to the calculated broadcast address and port.
                    s.sendto(compressed_data, (self.broadcast_address, self.broadcast_port))
                    logging.info(f"Sent data: {compressed_data}")
                    # Wait for 30 seconds before sending the next broadcast.
                    time.sleep(30)
                except Exception as e:
                    # Log any exceptions and wait 5 seconds before retrying.
                    logging.error(f"Error: {e}")
                    time.sleep(5)

if __name__ == "__main__":
    # Delay start to ensure network interfaces are fully initialized.
    time.sleep(30)
    # Configure logging to include timestamps, log levels, and messages.
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    # Initialize the BroadcastIPData class with default parameters.
    broadcaster = BroadcastIPData(network_interface='eth0', broadcast_port=51584, comm_port=51585)
    # Start broadcasting the network data continuously.
    broadcaster.send_data()
