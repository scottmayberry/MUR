import os
import rosbag
import cv2
import numpy as np
import tf2_ros
import tf2_py as tf2
import pickle
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge
from tqdm import tqdm

def convert_image_to_numpy(msg, bridge):
    return bridge.imgmsg_to_cv2(msg, "bgr8")

def get_fossen_com_pose(transform):
    translation = transform.transform.translation
    rotation = transform.transform.rotation
    quaternion = {
        'x': rotation.x,
        'y': rotation.y,
        'z': rotation.z,
        'w': rotation.w
    }
    roll, pitch, yaw = euler_from_quaternion([rotation.x, rotation.y, rotation.z, rotation.w])
    euler_angles = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
    return {
        'translation': {'x': translation.x, 'y': translation.y, 'z': translation.z},
        'quaternion': quaternion,
        'euler_angles': euler_angles
    }

def save_partial_data(output_file_path, data):
    with open(output_file_path, 'ab') as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)

def main(bag_path='your_rosbag.bag'):
    bag = rosbag.Bag(bag_path)
    bridge = CvBridge()
    
    # Create the output file path with the same name as the bag file, but with a .pkl extension, in a different directory
    bag_name = os.path.basename(bag_path)  # Get the base name of the bag file
    output_file_path = os.path.splitext(bag_name)[0] + '.pkl'  # Save in a different directory


    # Calculate the total number of messages
    total_pose_messages = bag.get_message_count(['/tf'])
    total_camera_messages = bag.get_message_count(['/camera_forward/image_raw', '/camera_radial_right/image_raw', '/camera_radial_left/image_raw'])
    total_messages = total_pose_messages + total_camera_messages

    data = {
        "images": {'front': [], 'right': [], 'left': []},
        "pose": []
    }

    pose_batch_size = 1000  # Increased batch size for poses
    image_batch_size = 200  # Increased batch size for images

    with tqdm(total=total_messages, desc="Processing all data") as pbar:
        # Process camera data
        for topic, msg, t in bag.read_messages(topics=['/camera_forward/image_raw', '/camera_radial_right/image_raw', '/camera_radial_left/image_raw']):
            img_data = convert_image_to_numpy(msg, bridge)
            if topic == '/camera_forward/image_raw':
                data['images']['front'].append({'t': t.to_sec(), 'image': img_data})
            elif topic == '/camera_radial_right/image_raw':
                data['images']['right'].append({'t': t.to_sec(), 'image': img_data})
            elif topic == '/camera_radial_left/image_raw':
                data['images']['left'].append({'t': t.to_sec(), 'image': img_data})

            if len(data['images']['front']) >= image_batch_size or len(data['images']['right']) >= image_batch_size or len(data['images']['left']) >= image_batch_size:
                save_partial_data(output_file_path, {'images': data['images']})
                data['images'] = {'front': [], 'right': [], 'left': []}  # Reuse the dictionary to reduce overhead

            pbar.update(1)
    
        if data['images']['front'] or data['images']['right'] or data['images']['left']:
            save_partial_data(output_file_path, {'images': data['images']})
            data['images'] = {'front': [], 'right': [], 'left': []}  # Reuse the dictionary to reduce overhead
        
        # Process pose data
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                if transform.header.frame_id == 'world' and transform.child_frame_id == 'fossen_base_link':
                    data['pose'].append({
                        't': t.to_sec(),
                        'posedata': get_fossen_com_pose(transform)
                    })

            if len(data['pose']) >= pose_batch_size:
                save_partial_data(output_file_path, {'pose': data['pose']})
                data['pose'] = []  # Reuse the list to reduce overhead

            pbar.update(1)

        if data['pose']:
            save_partial_data(output_file_path, {'pose': data['pose']})
            data['pose'] = []  # Reuse the list to reduce overhead

if __name__ == "__main__":
    bag_path = '/media/scott/Samsung USB/camera_tf_thruster_data_20240829_165029.bag'
    main(bag_path)
