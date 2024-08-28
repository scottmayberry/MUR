#!/usr/bin/env python
import socket
import sys
import time
import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import json
import sensor_publisher_constructor

def reload_module_info():
    return rospy.get_param('/module_info/modules')

def get_has_booted():
    return rospy.get_param('/module_info/hasBooted')

def set_has_booted(hasBooted:bool):
    rospy.set_param('/module_info/hasBooted', hasBooted)


def udp_server_listener_and_distributor():
    rospy.init_node('udp_server', anonymous=True)

    hasBooted = get_has_booted()

    UDP_IP = ''
    UDP_PORT_IN = int(rospy.get_param('/sensor_info/comm_port'))
    sock = socket.socket(socket.AF_INET, # Internet
                         socket.SOCK_DGRAM) # UDP
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((UDP_IP, UDP_PORT_IN))
    sock.settimeout(0.005)

    imu_covariance = [0]*9

    # Fetch parameters from the sensors namespace
    module_info = reload_module_info()

    # Initialize publishers
    pubs, sensors_config = sensor_publisher_constructor.initialize_sensor_publishers()

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(2048) # buffer size is 2048 bytes
        except:
            continue

        try:
            datastring = data.decode('utf8')
            dataJson = json.loads(datastring)
            if "serverRequestForIP" in dataJson:
                continue
        except:
            continue
        
        try:
            sender_ip = addr[0]
            module_name = next((m['name'] for m in module_info if 'ipaddress' in m.keys() and m['ipaddress'] == sender_ip), None)
            if not module_name:
                module_info = reload_module_info()
                continue

            # remove module_id
            module_id = dataJson.pop('id', 255)
            are_thrusters_working = dataJson.pop('isThrust', True)

            if not are_thrusters_working:
                print("THRUSTERS NOT CONNECTED...")
                raise ValueError("THRUSTERS NOT CONNECTED...")

            if not hasBooted:
                hasBooted = True
                set_has_booted(hasBooted)
        except:
            continue

        try:
            
            for sensor_name, info in dataJson.items():
                topic_base = sensors_config[sensor_name]['topic']
                topic = f"/sensors/{module_name}/{topic_base}"
                frame_id = f"{sensor_name}_{module_name}"
                # print(topic, topic_base)
                if topic in pubs:
                    if sensor_name == "BNO055":
                        orientation = Quaternion(info['q']['x'], info['q']['y'], info['q']['z'], info['q']['w'])
                        ang_vel = Vector3(info['av']['x'], info['av']['y'], info['av']['z'])
                        lin_acc = Vector3(info['la']['x'], info['la']['y'], info['la']['z'])
                        header = Header()
                        header.frame_id = frame_id
                        header.stamp = rospy.Time.now()
                        dataToSend = Imu(header, orientation, imu_covariance, ang_vel, imu_covariance, lin_acc, imu_covariance)
                        pubs[topic].publish(dataToSend)
                    elif sensor_name == "MPU6500":
                        accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                        gyro = Vector3(info['gyr_x'], info['gyr_y'], info['gyr_z'])
                        dataToSend = Twist(accel, gyro)
                        pubs[topic].publish(dataToSend)
                    elif sensor_name == "LSM6DS3":
                        accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                        gyro = Vector3(info['gyr_x'], info['gyr_y'], info['gyr_z'])
                        dataToSend = Twist(accel, gyro)
                        pubs[topic].publish(dataToSend)
                    elif sensor_name == "KXTJ3":
                        accel = Vector3(info['acc_x'], info['acc_y'], info['acc_z'])
                        dataToSend = accel
                        pubs[topic].publish(dataToSend)
                    elif sensor_name == "LIS2MDL":
                        mag = Vector3(info['x'], info['y'], -info['z'])
                        dataToSend = mag
                        pubs[topic].publish(dataToSend)
                    elif sensor_name == "HSCDTD":
                        mag = Vector3(info['x'], info['y'], -info['z'])
                        dataToSend = mag
                        pubs[topic].publish(dataToSend)
                    elif sensor_name == "AHT20":
                        humid = Float32(info['humidity'])
                        temp = Float32(info['temp'])
                        pubs[topic].publish(humid)
                        # pubs[f"{topic}_temp"].publish(temp)
                    elif sensor_name == "DPS310":
                        pressure = Float32(info['pressure'])
                        temp = Float32(info['temp'])
                        pubs[topic].publish(pressure)
                    elif sensor_name == "MS5837": #MS5837 fluid pressure
                        header = Header()
                        header.frame_id = frame_id
                        header.stamp = rospy.Time.now()
                        pressure = FluidPressure(header, info['pressure'], 0)
                        pubs[topic].publish(pressure)

        except Exception as e:
            print(f"[ERROR]: packet processing failed due to {e}")
            print(f"[ERROR]: topic with error {topic}")
            continue

if __name__ == "__main__":
    try:
        udp_server_listener_and_distributor()
    except rospy.ROSInterruptException:
        pass
