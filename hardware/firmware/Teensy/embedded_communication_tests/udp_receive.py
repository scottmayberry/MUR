import socket
import sys
import time


import socket

def get_Host_name_IP():
    try:
        host_name = socket.gethostname()
        host_ip = socket.gethostbyname(host_name)
        print("Hostname :  ", host_name)
        print("IP : ", host_ip)
        return host_name, host_ip
    except:
        print("Unable to get Hostname and IP")

_, UDP_IP = get_Host_name_IP()
UDP_IP = ''
UDP_PORT = 51584

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((UDP_IP, UDP_PORT))
sock.settimeout(0.005)
previousTime = time.time()-10
print("########## STARTING ###########")
while True:
    try:
        data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
        # print("received message: %s" % data)
        print(data, addr)
    except:
        pass
    # if time.time() > previousTime+10:
    #     MESSAGE = b"hello world2 howdy yes"
    #     sock.sendto(MESSAGE, ("192.168.1.199", UDP_PORT))
    #     previousTime = time.time() 