import os
import pickle
import cv2
import numpy as np
from tqdm import tqdm

def get_frame_rate(timestamps):
    """Calculate the frame rate based on timestamps."""
    if len(timestamps) < 2:
        return 30  # Default frame rate if not enough data
    time_diffs = np.diff(timestamps)
    avg_time_diff = np.mean(time_diffs)
    frame_rate = 1.0 / avg_time_diff
    return frame_rate

def create_video_from_pkl(pkl_path):
    # Load the data from the pkl file
    with open(pkl_path, 'rb') as f:
        frames = []
        timestamps = []
        while True:
            try:
                data_batch = pickle.load(f)
                for frame_data in data_batch:
                    frames.append(frame_data['data'])
                    timestamps.append(frame_data['t'])
            except EOFError:
                break
    
    # Calculate the frame rate
    frame_rate = get_frame_rate(timestamps)
    
    # Get the output file name
    parent_dir = os.path.basename(os.path.dirname(pkl_path))
    pkl_name = os.path.splitext(os.path.basename(pkl_path))[0]
    output_file = f"{parent_dir}_{pkl_name}.mp4"
    output_path = os.path.join(os.path.dirname(pkl_path), output_file)
    
    # Get frame size
    frame_height, frame_width, _ = frames[0].shape
    
    # Create the video writer
    video_writer = cv2.VideoWriter(output_path, cv2.VideoWriter_fourcc(*'mp4v'), frame_rate, (frame_width, frame_height))
    
    # Write frames to the video file
    for frame in tqdm(frames, desc=f"Creating video from {pkl_name}"):
        video_writer.write(frame)
    
    video_writer.release()
    print(f"Video saved to {output_path}")

def find_and_process_pkl_files(directory):
    pkl_files = []

    # Traverse the directory and find all relevant .pkl files
    for root, dirs, files in os.walk(directory):
        for file in files:
            if 'camera' in file.lower() and file.endswith(".pkl"):
                pkl_files.append(os.path.join(root, file))
    
    # Use tqdm to show progress while processing each .pkl file
    for pkl_path in tqdm(pkl_files, desc="Processing .pkl files"):
        create_video_from_pkl(pkl_path)

if __name__ == "__main__":
    input_directory = '/run/user/1000/gvfs/smb-share:server=tower.local,share=misc/pickle_files'  # Replace with the path to the directory containing your .pkl files
    
    # Process all .pkl files in the directory
    find_and_process_pkl_files(input_directory)
