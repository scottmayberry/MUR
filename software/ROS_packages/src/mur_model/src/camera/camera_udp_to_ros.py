#!/usr/bin/env python
import time
import rospy
import cv2
import threading
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError

def open_stream(camera, fps, index):
    stream_url = camera['stream_url']
    fps = camera['fps']
    cam_id = camera['cam_id']
    cam_frame_tf2 = f'camera_{cam_id}' if 'name' not in camera.keys() else camera['name']

    while not rospy.is_shutdown():
        cap = cv2.VideoCapture(stream_url)
        if cap.isOpened():
            rospy.loginfo(f"Successfully opened stream for camera_{cam_id} at {fps} FPS")
            # Launch a camera node with the appropriate index
            launch_camera_node(cam_id, cap, fps, cam_frame_tf2)
            break
        else:
            rospy.logwarn(f"Failed to open stream for camera_{index}. Retrying in 10 seconds...")
            time.sleep(10)
        cap.release()

def launch_camera_node(cam_id, cap, target_fps, cam_frame_tf2):
    pub = rospy.Publisher(f'/camera_{cam_id}/image_raw', Image, queue_size=10)
    rate = rospy.Rate(target_fps)  # Target FPS for publishing frames
    bridge = CvBridge()

    header = Header()
    header.frame_id = cam_frame_tf2

    while not rospy.is_shutdown() and cap.isOpened():
        ret, frame = cap.read()
        if ret:
            try:
                # Convert frame to a ROS Image message and publish it
                image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                header.stamp = rospy.Time.now()
                image_msg.header = header
                pub.publish(image_msg)
            except CvBridgeError as e:
                rospy.logerr(f"Failed to convert frame for camera_{cam_id}: {e}")
        else:
            rospy.logwarn(f"Failed to read frame from camera_{cam_id}")
        
        rate.sleep()

def load_camera_streams(compute_module_ip, cameras):

    # construct stream_url
    for camera in cameras:
        stream_port = camera['stream_port']
        camera['stream_url'] = f"http://{compute_module_ip}:{stream_port}/?action=stream"

    # start camera threads
    threads = []
    for index, camera in enumerate(cameras):
        t = threading.Thread(target=open_stream, args=(camera, index))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

if __name__ == "__main__":
    rospy.init_node('cameras', anonymous=True)
    camera_data = rospy.get_param("/model_info/camera_data")
    modules = compute_module = None
    while True:
        modules = rospy.get_param("/module_info/modules")
        compute_module = [module for module in modules if module["name"] == "mur_compute_module"][0]
        if 'ipaddress' in compute_module.keys():
            break
        time.sleep(5)
    compute_module_ip = compute_module['ipaddress']
    cameras = camera_data['cameras']
    try:
        load_camera_streams(compute_module_ip, cameras)
    except rospy.ROSInterruptException:
        pass
