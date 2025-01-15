import socket
import json
import subprocess
import time
import os

# Function to get the directory where the script is located
def get_script_dir():
    return os.path.dirname(os.path.realpath(__file__))

# Function to get a list of video devices that match the given ID_PATH_TAG
def get_video_device_by_id_path_tag(id_path_tag):
    devices = []
    for i in range(10):  # Assuming a maximum of 10 video devices
        device = f"/dev/video{i}"
        try:
            # Run udevadm command to get device info
            result = subprocess.run(
                ["udevadm", "info", "--query=all", "--name=" + device],
                capture_output=True, text=True, check=True
            )
            # Check if the device has the specified ID_PATH_TAG
            if f"ID_PATH_TAG={id_path_tag}" in result.stdout:
                devices.append(device)
        except subprocess.CalledProcessError:
            continue
    return devices

# Function to check if the camera control exists and get its current value
def get_camera_control(device, control_name):
    try:
        result = subprocess.run(
            ["v4l2-ctl", "-d", device, "--get-ctrl", control_name],
            capture_output=True, text=True, check=True
        )
        return result.stdout.strip()
    except subprocess.CalledProcessError:
        return None

def disable_autofocus(device):
    autofocus_control = "focus_automatic_continuous"
    
    try:
        # Attempt to disable autofocus
        subprocess.run(
            ["v4l2-ctl", "-d", device, "-c", f"{autofocus_control}=0"],
            check=True
        )
        print(f"Autofocus disabled on {device}")
    except subprocess.CalledProcessError:
        # If the command fails, just print a message and continue
        print(f"Failed to disable autofocus on {device}. Continuing without changes.")


# Function to launch the mjpg_streamer process for the specified device
def launch_mjpg_streamer(device, width, height, fps, stream_port, autofocus=True):
    # Get the directory of the script
    script_dir = get_script_dir()
    # Define the plugin directory based on the script's location
    plugin_dir = os.path.join(script_dir, "mjpg-streamer/mjpg-streamer-experimental")
    input_plugin = os.path.join(plugin_dir, "input_uvc.so")
    output_plugin = os.path.join(plugin_dir, "output_http.so")
    www_folder = os.path.join(plugin_dir, "www")
    
    # Check if the input plugin exists
    if not os.path.exists(input_plugin):
        print(f"ERROR: {input_plugin} does not exist")
        return False
    # Check if the output plugin exists
    if not os.path.exists(output_plugin):
        print(f"ERROR: {output_plugin} does not exist")
        return False

    # Disable autofocus if requested
    if not autofocus:
        disable_autofocus(device)

    # Define the command to launch mjpg_streamer
    command = [
        "/usr/local/bin/mjpg_streamer",
        "-i", f"{input_plugin} -d {device} -r {width}x{height} -f {fps}",
        "-o", f"{output_plugin} -w {www_folder} -p {stream_port}"
    ]
    # Set the environment variable for the plugin directory
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = plugin_dir
    try:
        # Launch the mjpg_streamer process
        subprocess.Popen(command, env=env)
        print(f"Successfully launched mjpg_streamer on {device}")
        return True
    except Exception as e:
        print(f"Failed to launch mjpg_streamer on {device}: {e}")
        return False

def main():
    UDP_IP = "0.0.0.0"  # Listen on all available network interfaces
    UDP_PORT = 5005  # Change this to your desired port

    launched_tags = set()  # Set to keep track of launched ID_PATH_TAGs

    # Create and bind the UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP messages on {UDP_IP}:{UDP_PORT}")

    while True:
        # Receive data from the UDP socket
        data, addr = sock.recvfrom(1024)
        try:
            # Decode and parse the JSON message
            message = json.loads(data.decode('utf-8'))
            id_path_tag = message["id_path_tag"]
            width = message["width"]
            height = message["height"]
            fps = message["fps"]
            stream_port = message["stream_port"]
            autofocus = message.get("autofocus", False)  # Default to turning off autofocus

            # Ignore messages for already launched ID_PATH_TAGs
            if id_path_tag in launched_tags:
                print(f"Ignoring message for already launched ID_PATH_TAG {id_path_tag}")
                continue

            # Get the list of video devices matching the ID_PATH_TAG
            devices = get_video_device_by_id_path_tag(id_path_tag)
            if not devices:
                print(f"No devices found for ID_PATH_TAG {id_path_tag}")
                continue

            # Try to launch mjpg_streamer for each matching device
            for device in devices:
                if launch_mjpg_streamer(device, width, height, fps, stream_port, autofocus):
                    launched_tags.add(id_path_tag)  # Add to launched tags if successful
                    break

        except json.JSONDecodeError:
            print("Received invalid JSON message")

        time.sleep(0.05)  # Reduce CPU usage

if __name__ == "__main__":
    main()
