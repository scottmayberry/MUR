import rosbag

def check_ros_time_in_bag(bag_path):
    bag = rosbag.Bag(bag_path)
    
    for topic, msg, t in bag.read_messages(topics=['/tf']):
        print(f"Message timestamp: {t.to_sec()}")
        break  # Just check the first message for demonstration purposes
    
    bag.close()

if __name__ == "__main__":
    bag_path = '/media/scott/Samsung USB/camera_tf_thruster_data_20240829_165029.bag'  # Replace with your actual bag file path
    check_ros_time_in_bag(bag_path)
