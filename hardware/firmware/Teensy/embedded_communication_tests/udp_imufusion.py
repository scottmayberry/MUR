import socket
import sys
import json
import imufusion
import numpy
import time

# def get_Host_name_IP():
#     try:
#         host_name = socket.gethostname()
#         host_ip = socket.gethostbyname(host_name)
#         print("Hostname :  ", host_name)
#         print("IP : ", host_ip)
#         return host_name, host_ip
#     except:
#         print("Unable to get Hostname and IP")

UDP_IP = ''
UDP_PORT = 8888

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

idInData = "LSM6DS3TR"
#  "LIS2MDL"

sample_rate = 50
# Instantiate algorithms
offset = imufusion.Offset(sample_rate)
ahrs = imufusion.Ahrs()

ahrs.settings = imufusion.Settings(0.5,  # gain
                                   10,  # acceleration rejection
                                   20,  # magnetic rejection
                                   5 * sample_rate)  # rejection timeout = 5 seconds
gyroscope = numpy.empty((3))
accelerometer = numpy.empty((3))
magnetometer = numpy.empty((3))
deltaTime = numpy.float64(1/sample_rate)
euler = numpy.empty((3))
print("########## STARTING ###########")
while True:
    data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
    # print("received message: %s" % data)
    try:
        datastring = data.decode('utf8')
        dataJson = json.loads(datastring)
    except:
        continue
    
    if idInData in dataJson.keys():
        gyroscope[0] = dataJson["MPU6500"]["gyr_x"]
        gyroscope[1] = dataJson["MPU6500"]["gyr_y"]
        gyroscope[2] = dataJson["MPU6500"]["gyr_z"]

        accelerometer[0] = dataJson["MPU6500"]["acc_x"]
        accelerometer[1] = dataJson["MPU6500"]["acc_y"]
        accelerometer[2] = dataJson["MPU6500"]["acc_z"]

        magnetometer[0] = dataJson["LIS2MDL"]["x"]
        magnetometer[1] = dataJson["LIS2MDL"]["y"]
        magnetometer[2] = dataJson["LIS2MDL"]["z"]
        gyroscope = offset.update(gyroscope)

        ahrs.update(gyroscope, accelerometer, magnetometer, deltaTime)
        euler = ahrs.quaternion.to_euler()
        print(euler)