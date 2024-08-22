import socket
import time

# "192.168.1.10": sensor board
# "192.168.1.222": interface board
UDP_IPs = ["192.168.1.90" ]#, "192.168.1.160"]
UDP_PORT = 51585
MESSAGE = b"{error_attealsdflajsdfklajsdlfkjaskldfjaklsjdflkajsdklfjaklsdjfaklsdfjkalsdjfklasjdflkjsdlkmpt\":[1400,1400]}"*10
# MESSAGE = b"{\"thruster_us\":[1500,1500,1500,1500,1501,1500]}\n"
# MESSAGE = b"how is this going"
for UDP_IP in UDP_IPs:
    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_PORT)
    print("message: %s" % MESSAGE)

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    timeOut = time.time()
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
    print(timeOut)
    sock.close()