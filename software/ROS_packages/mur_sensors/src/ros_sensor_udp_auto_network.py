#!/usr/bin/env python
import socket
import time
import rospy
import json
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *

def get_ip_address():
    try:
        # Get the hostname of the machine
        hostname = socket.gethostname()
        # Get the IP address associated with the hostname
        ip_address = socket.gethostbyname(hostname)
    except Exception as e:
        # Print error and set IP address to an error message if there's an exception
        print(f"Error obtaining IP address: {e}")
        ip_address = "Unable to determine IP"
    
    return ip_address

def normalize_mac(mac):
    # Remove any delimiters (":" or "-") and convert to lowercase
    return mac.replace(":", "").replace("-", "").lower()

def update_ip_address(modules, mac_address, new_ip):
    # Normalize the MAC address for comparison
    normalized_mac = normalize_mac(mac_address)
    for module in modules:
        if 'mac' in module and normalize_mac(module['mac']) == normalized_mac:
            # Check if the current IP address matches the new IP address
            current_ip = module.get('ipaddress', None)
            if current_ip == new_ip:
                # Return the module if the IP address is already correct
                # print(f"IP address for module {module['name']} is already {new_ip}")
                return module
            else:
                # Update the IP address and set the parameter
                module['ipaddress'] = new_ip
                rospy.set_param('/module_info/modules', modules)
                return module
    return None

def update_comm_port(modules, mac_address, new_comm_port):
    # Normalize the MAC address for comparison
    normalized_mac = normalize_mac(mac_address)
    for module in modules:
        if 'mac' in module and normalize_mac(module['mac']) == normalized_mac:
            # Check if the current comm port matches the new comm port
            current_comm_port = module.get('ports', {}).get('comm', None)
            if current_comm_port == new_comm_port:
                # Return the module if the comm port is already correct
                # print(f"Comm port for module {module['name']} is already {new_comm_port}")
                return module
            else:
                # Update the comm port and set the parameter
                module['ports']['comm'] = new_comm_port
                rospy.set_param('/module_info/modules', modules)
                return module
    return None

def send_response(ip_address, comm_port, server_id_message):
    # Encode the server ID message and send it to the specified IP address and comm port
    message = server_id_message.encode('utf-8')
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.sendto(message, (ip_address, comm_port))
    # print(f"Sent response to {ip_address}:{comm_port}")

def udp_server_receive_broadcast(modules, broadcast_port, server_id_message, my_ip_address):
    # Initialize the ROS node
    rospy.init_node('udp_auto_network', anonymous=True)

    UDP_IP = ''
    UDP_PORT_IN = broadcast_port
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT_IN))
    sock.settimeout(0.005)

    while not rospy.is_shutdown():
        # Sleep briefly to prevent busy waiting
        time.sleep(0.2)
        try:
            # Receive data from the socket
            data, addr = sock.recvfrom(2048) # buffer size is 2048 bytes
        except:
            continue

        try:
            # Decode and load the JSON data
            datastring = data.decode('utf8')
            dataJson = json.loads(datastring)
            if "serverRequestForIP" not in dataJson:
                continue

            # print(dataJson)
            # Extract MAC address, new IP, and new comm port from the received data
            mac_address = dataJson['mac']
            new_ip = dataJson['serverRequestForIP']
            new_comm_port = dataJson.get('COMM_PORT', None)
            
            # Update the IP address in the module info
            updated_module = update_ip_address(modules, mac_address, new_ip)
            # if updated_module:
            #     print(f"Updated IP address for module {updated_module['name']} to {new_ip}")

            # Update the comm port in the module info if provided
            if new_comm_port is not None:
                updated_module = update_comm_port(modules, mac_address, new_comm_port)
                # if updated_module:
                #     print(f"Updated comm port for module {updated_module['name']} to {new_comm_port}")
            
            # Send response back to the module if serverIP does not match my_ip_address
            if updated_module and 'ipaddress' in updated_module and 'ports' in updated_module:
                server_ip = dataJson.get('serverIP', None)
                # print(server_ip, my_ip_address)
                if server_ip != my_ip_address:
                    ip_address = updated_module['ipaddress']
                    comm_port = updated_module['ports']['comm']
                    send_response(ip_address, comm_port, server_id_message)
                # else:
                #     print(f"Server IP {server_ip} matches my IP {my_ip_address}, not sending response.")

        except Exception as e:
            # Print any errors that occur during processing
            print(f"Error processing received data: {e}")
            continue

if __name__ == "__main__":
    # Get the parameters from the ROS parameter server
    broadcast_port = int(rospy.get_param("/sensor_info/broadcast_port"))
    my_ip_address = get_ip_address()
    server_id_message = rospy.get_param("/sensor_info/server_id_message")
    modules = rospy.get_param("/module_info/modules")
    # Filter modules that have a MAC address key
    modules = [module for module in modules if 'mac' in module.keys()]
    try:
        # Start the UDP server to receive broadcasts
        udp_server_receive_broadcast(modules, broadcast_port, server_id_message, my_ip_address)
    except rospy.ROSInterruptException:
        pass
