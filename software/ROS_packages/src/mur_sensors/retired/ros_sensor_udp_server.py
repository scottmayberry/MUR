#!/usr/bin/env python
import socket
import sys
import time
import yaml
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import json

def udp_server_listener_and_distributor(UDP_PORT_OUT = 8888):

    UDP_IP = ''
    UDP_PORT_IN = rospy.get_param('/sensor_info/sensor_udp_receive_info/receive_port')
    print("UDP PORT IN", UDP_PORT_IN)
    sock = socket.socket(socket.AF_INET, # Internet
                        socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT_IN))
    sock.settimeout(0.005)
    
    topicHeader = ""
    pubs = {'MPU6500':{"topic":'MPU6500', 'type': Twist, 'queue': 2},
            'GPS':{"topic":'GPS', 'type': String, 'queue': 2},
            'LSM6DS3TR':{"topic":'LSM6DS3TR', 'type': Twist, 'queue': 2},
            'KXTJ3_1057':{"topic":'KXTJ3_1057', 'type': Vector3, 'queue': 2},
            'BNO055':{"topic":'BNO055', 'type': Imu, 'queue': 2},
            'LIS2MDL':{"topic":'LIS2MDL', 'type': Vector3, 'queue': 2},
            'AHT20':{"topic":'AHT20', 'type': Float32, 'queue': 2},
            'DPS310':{"topic": 'DPS310', 'type': Float32, 'queue': 2},
            'temp':{"topic": 'temp', 'type': Float32, 'queue': 2},
            'MS5837_02BA':{"topic": 'water_pressure', 'type': Float32, 'queue': 2}}

    imu_covariance = [0]*9
        
    for key in pubs.keys():
        pubs[key]['topic'] = topicHeader + pubs[key]['topic']
        pubs[key]['pub'] = rospy.Publisher(pubs[key]['topic'], pubs[key]['type'], queue_size=pubs[key]['queue'])
    rospy.init_node('udp_server', anonymous=True)
    # rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(2048) # buffer size is 1024 bytestry:
        except:
            continue
        try:
            datastring = data.decode('utf8')
            dataJson = json.loads(datastring)
        except:
            continue
        try:
            if 'BNO055' in dataJson.keys():
                info = dataJson['BNO055']
                orientation = Quaternion(info['q']['x'], info['q']['y'], info['q']['z'], info['q']['w'])
                ang_vel = Vector3(info['av']['x'], info['av']['y'], info['av']['z'])
                lin_acc = Vector3(info['la']['x'], info['la']['y'], info['la']['z'])
                header = Header()
                header.stamp = rospy.Time.now()
                dataToSend = Imu(header, orientation, imu_covariance, ang_vel, imu_covariance, lin_acc, imu_covariance)
                pubs['BNO055']['pub'].publish(dataToSend)
            if 'MPU6500' in dataJson.keys():
                info = dataJson['MPU6500']
                accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                gyro = Vector3(info['gyr_x'], info['gyr_y'], info['gyr_z'])
                dataToSend = Twist(accel, gyro)
                # print(dataToSend)
                pubs['MPU6500']['pub'].publish(dataToSend)
            if 'LSM6DS3TR' in dataJson.keys():
                info = dataJson['LSM6DS3TR']
                accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                gyro = Vector3(info['gyr_x'], info['gyr_y'], info['gyr_z'])
                dataToSend = Twist(accel, gyro)
                pubs['LSM6DS3TR']['pub'].publish(dataToSend)
            if 'KXTJ3_1057' in dataJson.keys():
                info = dataJson['KXTJ3_1057']
                accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                pubs['KXTJ3_1057']['pub'].publish(accel)
            if 'LIS2MDL' in dataJson.keys():
                info = dataJson['LIS2MDL']
                # negative z to align the axis with right-hand rule. This is considered
                # in the model position and orientation as well.
                mag = Vector3(info['x'], info['y'], -info['z'])
                pubs['LIS2MDL']['pub'].publish(mag)
            if 'AHT20' in dataJson.keys():
                info = dataJson['AHT20']
                humid = Float32(info['humidity'])
                temp = Float32(info['temp'])
                pubs['AHT20']['pub'].publish(humid)
                pubs['temp']['pub'].publish(temp)
            if 'DPS310' in dataJson.keys():
                info = dataJson['DPS310']
                pressure = Float32(info['pressure'])
                temp = Float32(info['temp'])
                pubs['DPS310']['pub'].publish(pressure)
                pubs['temp']['pub'].publish(temp)
            if 'MS5837_02BA' in dataJson.keys():
                info = dataJson['MS5837_02BA']
                pressure = Float32(info['pressure'])
                pubs['MS5837_02BA']['pub'].publish(pressure)
        except:
            print("[ERROR]: packet failed")
            continue
    


# _, UDP_IP = get_Host_name_IP()
if __name__ == "__main__":
    try:
        module_info = rospy.get_param("/module_info")
        battery_fuselage_module_info = [module for module in module_info if module["name"] == "mur-battery-fuselage"][0]
        udp_server_listener_and_distributor(battery_fuselage_module_info["receivePort"])
    except rospy.ROSInterruptException:
        pass
    # load ip_addresses listed in the file
    # file = "iptable.yaml"
    # ip_addresses = getIpAddresses(file)
    # print(ip_addresses)

    # UDP_IP = ''
    # UDP_PORT = 8888

    # sock = socket.socket(socket.AF_INET, # Internet
    #                     socket.SOCK_DGRAM) # UDP
    # sock.bind((UDP_IP, UDP_PORT))

    # print("########## STARTING ###########")
    # lastMessageTime = time.time()
    # timeBetweenPings = 1
    # while True:
    #     data, addr = sock.recvfrom(2048) # buffer size is 1024 bytes
    #     # print("received message: %s" % data)
    #     print(data)
    #     if(time.time() > lastMessageTime+timeBetweenPings):
    #         lastMessageTime = time.time()
    #         sendServerIdMessage(sock, ip_addresses)
