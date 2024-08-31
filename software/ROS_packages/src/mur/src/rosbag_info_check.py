import rosbag
from tqdm import tqdm

def check_tf_data_in_bag(bag_path):
    bag = rosbag.Bag(bag_path)
    
    tf_topic = '/tf'
    unique_frames = set()
    
    # Get the total number of messages for progress tracking
    total_messages = bag.get_message_count([tf_topic])

    # Iterate through the messages with a progress bar
    for topic, msg, t in tqdm(bag.read_messages(topics=[tf_topic]), total=total_messages, desc="Processing TF messages"):
        for transform in msg.transforms:
            parent_frame = transform.header.frame_id
            child_frame = transform.child_frame_id
            unique_frames.add(str(parent_frame) + '_' + str(child_frame))
    
    bag.close()

    print("\nUnique Frame Names (Axes):")
    for frame in sorted(unique_frames):
        print(frame)

if __name__ == "__main__":
    bag_path = '/media/scott/Samsung USB/camera_tf_thruster_data_20240829_165029.bag'  # Replace with your actual bag file path
    check_tf_data_in_bag(bag_path)
