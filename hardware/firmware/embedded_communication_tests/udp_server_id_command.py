import socket
import yaml
import time

# Send server_id_message to all listed IP_ADDRESSES on the MUR.
# By dynamically broadcasting this computers ipaddress to the MUR,
# the ip address does not need to be hardcoded in any MUR device

# load ip table listed in 'filename'
def loadIpTable(filename):
    with open(filename, "r") as stream:
        try:
            return yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            return None

if __name__ == "__main__":
    # load ip_addresses listed in the file
    file = "iptable.yaml"
    IP_TABLE_DICT = loadIpTable(file)
    UDP_IPs = [IP_TABLE_DICT[x]['ipaddress'] for x in IP_TABLE_DICT.keys()]

    # how long to wait before repinging all listed ip addresses
    timePerPingSeconds = 15

    # port for UDP
    UDP_PORT = 8888

    # server id string
    SERVER_ID_MESSAGE = b"server"

    print("UDP target port: %s" % UDP_PORT)
    print("message to send: %s" % SERVER_ID_MESSAGE)

    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP

    # loop every 'timePerPingSeconds', and send the SERVER_ID_MESSAGE
    while True:
        print("Time:",time.time())
        for ip_address in UDP_IPs:
            print("UDP target IP: %s" % ip_address)
            sock.sendto(SERVER_ID_MESSAGE, (ip_address, UDP_PORT))
        time.sleep(timePerPingSeconds)