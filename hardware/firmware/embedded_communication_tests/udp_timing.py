import socket
import time
import json

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
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

print("########## STARTING ###########")
counter = 0
previousTime = 0
# idToLookFor = "LSM6DS3TR"
# idToLookFor = "MPU6500"
# idToLookFor = "KXTJ3_1057"
# idToLookFor = "LIS2MDL"
# idToLookFor = "DPS310"
idToLookFor = "BNO055"
# idToLookFor = "MS5837"
# idToLookFor = "HSCDTD"
# idToLookFor = "LIS2MDL"
numberToCount = 500
previousTime = 0
counter = numberToCount-2
while True:
    try:
        data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
        if addr[0] == '192.168.1.11':
            continue
        datastring = data.decode('utf8')
        dataJson = json.loads(datastring)
        if idToLookFor in dataJson.keys():
            counter = counter + 1
            # print(dataJson[idToLookFor])
            if counter >= numberToCount:
                print("Hz: ", counter/(time.time()-previousTime))
                counter = 0
                previousTime = time.time()
    except:
        print("reset", data)
        counter = 0
        previousTime = time.time()
    