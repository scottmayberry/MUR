# Camera Stream Setup Guide

This guide will help you set up cameras on MUR (Miniature Underwater Robot) so that a simple JSON command can auto-launch them with the desired resolution and frame rate, as well as be streamed over ethernet.

## Goal

The goal is to configure your cameras so that they can be started automatically via a JSON command, setting the correct resolution and frames per second (FPS), and allowing the camera feed to be streamed over the network.

## Step 1: Update and Install Dependencies

Begin by updating your system and installing the necessary dependencies to compile and run the camera streaming software.

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential libjpeg62-turbo-dev libv4l-dev cmake git netcat-openbsd
```

## Step 2: Download and Compile `mjpg-streamer`

Next, download and compile `mjpg-streamer`, the software that will handle the camera streaming.

```bash
cd /opt
sudo mkdir -p udp_camera_streamer
cd /opt/udp_camera_streamer
sudo git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
sudo make
sudo make install
```

## Step 3: Confirm `mjpg-streamer` Installation

To confirm that `mjpg-streamer` is installed correctly, plug in a USB camera and use the appropriate `/dev/video*` port to test.

```bash
./mjpg_streamer -i "./input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "./output_http.so -w ./www -p 8080"
```

Check the camera stream in your web browser by navigating to `http://<Raspberry_Pi_IP>:8080`. You should see the video feed, and you can modify camera parameters using the controls on the HTML page. Press `Ctrl+C` to stop the stream once confirmed.

## Step 4: Install ID_PATH_TAG Identifier

This step involves setting up a script that identifies USB devices connected as `/dev/video*` devices and retrieves their `ID_PATH_TAG`. This unique tag ensures that each camera is consistently identified, even if the `/dev/video*` link changes after a reboot.

1. Create a script to list unique USB `ID_PATH_TAG` identifiers:

```bash
cd /opt/udp_camera_streamer
sudo nano list_unique_usb_id_path_tags.sh
```

2. Copy the following code into `list_unique_usb_id_path_tags.sh`:

```bash
#!/bin/bash

# Array to store unique ID_PATH_TAGs
declare -A id_path_tags

# Loop through all /dev/video* devices
for device in /dev/video*; do
    if [[ -e "$device" ]]; then
        id_bus=$(udevadm info --query=all --name="$device" | grep 'ID_BUS' | cut -d '=' -f2)
        if [[ "$id_bus" == "usb" ]]; then
            id_path_tag=$(udevadm info --query=all --name="$device" | grep 'ID_PATH_TAG' | cut -d '=' -f2)
            if [[ -n "$id_path_tag" ]]; then
                id_path_tags["$id_path_tag"]=1
            fi
        fi
    fi
done

# Print all unique ID_PATH_TAGs
for tag in "${!id_path_tags[@]}"; do
    echo "$tag"
done
```

3. Make the script executable:

```bash
sudo chmod +x list_unique_usb_id_path_tags.sh 
```

4. To test the script, plug in your USB cameras and run the script:

```bash
./list_unique_usb_id_path_tags.sh
```

Example output:

```bash
platform-fe9c0000_xhci-usb-0_1_4_1_0
platform-fe9c0000_xhci-usb-0_1_3_1_0
platform-fe9c0000_xhci-usb-0_1_2_2_1_0
```

These `ID_PATH_TAG`s will be used in the next steps. Make sure to save them.

## Step 5: Set Up `udp_camera_streamer` Python Script

The `udp_camera_streamer` service listens for JSON commands over UDP on port 5005 (configurable) to launch the camera with the corresponding `ID_PATH_TAG`. The camera stream will be available at `http://<DEVICE_IP_ADDRESS>:<STREAM_PORT>`.

1. Create the `udp_camera_streamer.py` file:

```bash
cd /opt/udp_camera_streamer
sudo nano udp_camera_streamer.py
```

2. Copy the following Python code into `udp_camera_streamer.py`:

```python
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
```

3. Make the script executable:

```bash
sudo chmod +x udp_camera_streamer.py
```

4. Run the script to test it:

```bash
python ./udp_camera_streamer.py
```

At this point, the script will be running and waiting to receive JSON commands over UDP.

5. Send a test JSON command

In a separate terminal, send a test JSON command to start streaming from a specific USB camera device. If sending from the same device, use `127.0.0.1`, or replace it with `<DEVICE_IP_ADDRESS>`:

```bash
# Replace id_path_tag with a valid one from the previous steps
echo '{"id_path_tag": "platform-fe9c0000_xhci-usb-0_1_4_1_0", "width": 640, "height": 480, "fps": 10, "stream_port": 8080}' | nc -u -w1 127.0.0.1 5005
```

You should see an output in the terminal where the `udp_camera_streamer.py` script is running. If you navigate to `http://<DEVICE_IP_ADDRESS>:<stream_port>`, you should see the camera stream and be able to adjust the camera parameters. Press `Ctrl+C` when done.

## Step 6: Set Up `udp_camera_streamer` Service

Now that the `udp_camera_streamer.py` script is working, we'll set it up as a service so that it starts automatically on boot.

1. Create the service file:

```bash
sudo nano /etc/systemd/system/udp_camera_streamer.service
```

2. Add the following configuration:

```ini
[Unit]
Description=UDP Camera Streamer Service
After=network.target

[Service]
# User=your_username
WorkingDirectory=/opt/udp_camera_streamer
ExecStart=/usr/bin/python3 /opt/udp_camera_streamer/udp_camera_streamer.py
Restart=always

[Install]
WantedBy=multi-user.target
```

If you want the service to run as a specific user, uncomment the `User` line and replace `your_username` with your actual username.

3. Reload the systemd daemon, enable the service to start on boot, and start the service:

```bash
sudo systemctl daemon-reload
sudo systemctl enable udp_camera_streamer.service
sudo systemctl start udp_camera_streamer.service
```

4. Check the status of the service:

```bash
sudo systemctl status udp_camera_streamer.service
```

You can now send the `echo` command from the previous step to relaunch the camera stream. To view the output of the script, recheck the service status.

## Conclusion

You should now have a camera service set up on your MUR that allows any connected camera to start streaming with a simple JSON command sent over UDP.