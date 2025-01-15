#!/usr/bin/env python3
import json
import socket
import time
import netifaces
import struct
import logging

class BroadcastIPData:
    def __init__(self, network_interface='eth0', broadcast_port=51584, comm_port=51585):
        self.network_interface = network_interface
        self.broadcast_port = broadcast_port
        self.comm_port = comm_port
        self.mac_address = self.get_mac_address()
        self.ip_address, self.netmask = self.get_ip_and_subnet()

        if self.ip_address is None or self.netmask is None:
            logging.error("Failed to retrieve IP address or netmask. Exiting.")
            exit(1)

        self.broadcast_address = self.calculate_broadcast_address()
        self.data = {
            "id": 0,
            "serverRequestForIP": self.ip_address,
            "mac": self.mac_address,
            "BROADCAST_PORT": self.broadcast_port,
            "COMM_PORT": self.comm_port
        }

    def get_mac_address(self):
        try:
            with open(f'/sys/class/net/{self.network_interface}/address') as f:
                mac = f.read().strip()
            return mac
        except FileNotFoundError:
            logging.error(f"Could not find MAC address for interface {self.network_interface}")
            return None

    def get_ip_and_subnet(self):
        try:
            iface_data = netifaces.ifaddresses(self.network_interface)[netifaces.AF_INET][0]
            ip_address = iface_data['addr']
            netmask = iface_data['netmask']
            return ip_address, netmask
        except (KeyError, IndexError):
            logging.error(f"Could not find IP address or netmask for interface {self.network_interface}")
            return None, None

    def calculate_broadcast_address(self):
        ip_bin = struct.unpack('!I', socket.inet_aton(self.ip_address))[0]
        netmask_bin = struct.unpack('!I', socket.inet_aton(self.netmask))[0]
        broadcast_bin = ip_bin | ~netmask_bin
        broadcast_address = socket.inet_ntoa(struct.pack('!I', broadcast_bin & 0xFFFFFFFF))
        return broadcast_address

    def send_data(self):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            while True:
                try:
                    compressed_data = json.dumps(self.data, separators=(',', ':')).encode('utf-8')
                    s.sendto(compressed_data, (self.broadcast_address, self.broadcast_port))
                    logging.info(f"Sent data: {compressed_data}")
                    time.sleep(30)
                except Exception as e:
                    logging.error(f"Error: {e}")
                    time.sleep(5)

if __name__ == "__main__":
    time.sleep(30)
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    broadcaster = BroadcastIPData(network_interface='eth0', broadcast_port=51584, comm_port=51585)
    broadcaster.send_data()
