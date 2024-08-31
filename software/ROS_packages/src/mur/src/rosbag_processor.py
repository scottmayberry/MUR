import os
import rosbag
import cv2
import numpy as np
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

def save_partial_data(file_path, data):
    with open(file_path, 'ab') as f:
        pickle.dump(data, f, protocol=pickle.HIGHEST_PROTOCOL)

def main(bag_path, output_dir_base):
    bag = rosbag.Bag(bag_path)
    bridge = CvBridge()
    
    # Create a folder with the bag_name in the output directory
    bag_name = os.path.splitext(os.path.basename(bag_path))[0]
    output_dir = os.path.join(output_dir_base, bag_name)
    os.makedirs(output_dir, exist_ok=True)

    # Define the paths for the different .pkl files
    forward_camera_path = os.path.join(output_dir, 'forward_camera.pkl')
    right_camera_path = os.path.join(output_dir, 'right_camera.pkl')
    left_camera_path = os.path.join(output_dir, 'left_camera.pkl')
    pose_path = os.path.join(output_dir, 'pose.pkl')

    # Calculate the total number of messages
    total_pose_messages = bag.get_message_count(['/tf'])
    total_camera_messages = bag.get_message_count(['/camera_forward/image_raw', '/camera_radial_right/image_raw', '/camera_radial_left/image_raw'])
    total_messages = total_pose_messages + total_camera_messages

    # Define batch sizes
    pose_batch_size = 1000
    image_batch_size = 200

    # Initialize batches
    forward_camera_batch = []
    right_camera_batch = []
    left_camera_batch = []
    pose_batch = []

    with tqdm(total=total_messages, desc="Processing all data") as pbar:
        # Process camera data
        for topic, msg, t in bag.read_messages(topics=['/camera_forward/image_raw', '/camera_radial_right/image_raw', '/camera_radial_left/image_raw']):
            img_data = convert_image_to_numpy(msg, bridge)
            data = {'t': t.to_sec(), 'data': img_data}

            if topic == '/camera_forward/image_raw':
                forward_camera_batch.append(data)
                if len(forward_camera_batch) >= image_batch_size:
                    save_partial_data(forward_camera_path, forward_camera_batch)
                    forward_camera_batch = []  # Clear the batch
            elif topic == '/camera_radial_right/image_raw':
                right_camera_batch.append(data)
                if len(right_camera_batch) >= image_batch_size:
                    save_partial_data(right_camera_path, right_camera_batch)
                    right_camera_batch = []  # Clear the batch
            elif topic == '/camera_radial_left/image_raw':
                left_camera_batch.append(data)
                if len(left_camera_batch) >= image_batch_size:
                    save_partial_data(left_camera_path, left_camera_batch)
                    left_camera_batch = []  # Clear the batch

            pbar.update(1)
    
        # Save any remaining camera data
        if forward_camera_batch:
            save_partial_data(forward_camera_path, forward_camera_batch)
        if right_camera_batch:
            save_partial_data(right_camera_path, right_camera_batch)
        if left_camera_batch:
            save_partial_data(left_camera_path, left_camera_batch)
        
        # Process pose data
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for transform in msg.transforms:
                if transform.header.frame_id == 'world' and transform.child_frame_id == 'fossen_base_link':
                    pose_data = {
                        't': t.to_sec(),
                        'data': get_fossen_com_pose(transform)
                    }
                    pose_batch.append(pose_data)
                    if len(pose_batch) >= pose_batch_size:
                        save_partial_data(pose_path, pose_batch)
                        pose_batch = []  # Clear the batch

            pbar.update(1)

        # Save any remaining pose data
        if pose_batch:
            save_partial_data(pose_path, pose_batch)

def find_bag_files(directory):
    bag_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".bag.active"):
                bag_files.append(os.path.join(root, file))
    return bag_files


if __name__ == "__main__":
    
    directory = '/media/scott/Samsung USB1'  # Replace with the directory containing your .bag files
    output_dir_base = '/run/user/1000/gvfs/smb-share:server=tower.local,share=misc/pickle_files'  # Replace with your desired output directory
    
    bag_files = find_bag_files(directory)
    
    # Print out all the .bag file names
    print("Found the following .bag files:")
    for bag_file in bag_files:
        print(bag_file)
    
    # Run the main function for each .bag file
    for i, bag_file in enumerate(bag_files):
        print(f"Processing {bag_file}...{i+1}/{len(bag_files)}")
        main(bag_file, output_dir_base)

    # # single file example
    # main('/media/scott/Samsung USB1/camera_tf_thruster_data_20240829_165353.bag', output_dir_base)
