# Goal
Setup cameras on MUR so that a simple JSON command will autolaunch them with the proper resolution and fps.

# Step 1: Update and install dependencies
```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get install build-essential libjpeg62-turbo-dev libv4l-dev cmake git netcat-openbsd
```

# Step 2: Download and Compile mjpg-streamer
```bash
cd /opt
sudo mkdir -p udp_camera_streamer
cd /opt/udp_camera_streamer
sudo git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
sudo make
sudo make install
```

# Step 3: Confirm mjpg-streamer install
Plug in a usb camera and use the proper /dev/video* port to test.
```bash
./mjpg_streamer -i "./input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "./output_http.so -w ./www -p 8080"
```
Check in browser by going to  http://<Raspberry_Pi_IP>:8080. Camera controls on html page allow for changing camera parameters.

Ctrl+C once confirmed.

# Step 4: Install ID_PATH_TAG Identifier
This code identified USB devices plugged in the are /dev/video* devices. It then gets their ID_PATH_TAG, a unique tag related to the specific hardware bus the USB device is plugged into. This ID_PATH_TAG will point to the same camera on reboot, even if the /dev/video* link changes (ie from /dev/video0 to /dev/video2 and etc.)

```bash
cd /opt/udp_camera_streamer
sudo nano list_unique_usb_id_path_tags.sh
```

Copy the below code into 'list_unique_usb_id_path_tags.sh'.

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

Run chmod to make the .sh file executable.
```bash
sudo chmod +x list_unique_usb_id_path_tags.sh 
```
To test with USB cameras plugged in, run
```bash
./list_unique_usb_id_path_tags.sh
```
Example output:
```bash
platform-fe9c0000_xhci-usb-0_1_4_1_0
platform-fe9c0000_xhci-usb-0_1_3_1_0
platform-fe9c0000_xhci-usb-0_1_2_2_1_0
```

These ID_PATH_TAGs will be used in the next steps. Save.

# Step 4: udp_camera_streamer python
The udp_camera_streamer service will receive a JSON over UDP on port 5005 (changeable in below code). This will launch the camera with corresponding ID_PATH_TAG, and will stream it on 

http://<DEVICE_IP_ADDRESS>:<STREAM_PORT>. 

Example, for device 192.168.8.232, the below json will launch the camera stream on http://192.168.8.232:8080.

```json
# Example JSON of message received by udp_camera_streamer.py
{
    "id_path_tag": "platform-fe9c0000_xhci-usb-0_1_4_1_0",
    "width": 640,
    "height": 480,
    "fps": 10,
    "stream_port": 8080
}
```
This format allows for configuring exact resolutions, fps, and exact camera for streaming.

To start setup, 
### 1. create/open the udp_camera_streamer.py file
```bash
cd /opt/udp_camera_streamer
sudo nano udp_camera_streamer.py
```
### 2. Copy the below into udp_camera_streamer.py.
```python
# udp_camera_streamer.py
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

# Function to launch the mjpg_streamer process for the specified device
def launch_mjpg_streamer(device, width, height, fps, stream_port):
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
                if launch_mjpg_streamer(device, width, height, fps, stream_port):
                    launched_tags.add(id_path_tag)  # Add to launched tags if successful
                    break

        except json.JSONDecodeError:
            print("Received invalid JSON message")

        time.sleep(0.05)  # Reduce CPU usage

if __name__ == "__main__":
    main()

```
and save the file. 

### 3. Make the file executable

```bash
sudo chmod +x udp_camera_streamer.py
```

### 4. Run the file to test. Nothing should really happen yet.
```bash
python ./udp_camera_streamer.py
```

### 5. Send test echo JSON
In a seperate terminal window, send a test echo command matching an IP_PATH_TAG of an existing USB camera device. If sending from the same device, use 127.0.0.1, or replace with <DEVICE_IP_ADDRESS>
```bash
# swap the id_path_tag for a valid one
echo '{"id_path_tag": "platform-fe9c0000_xhci-usb-0_1_4_1_0", "width": 640, "height": 480, "fps": 10, "stream_port": 8080}' | nc -u -w1 127.0.0.1 5005
```
You should see an output in the other port. If you go to http://<DEVICE_IP_ADDRESS>:<stream_port>, you should be able to see the stream, modify camera parameters, ect. Ctrl+C when done.

# Step 5: udp_camera_streamer Service
Now that the udp_camera_streamer.py is functioning, we want to set up a service to autolaunch on boot.

### 1. Create the service file
```bash
sudo nano /etc/systemd/system/udp_camera_streamer.service
```

### 2. Add the Following Configuration:
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
This will run as root. If you want to run as user, uncomment the User line and replace `your_username` with your actual username.

### 3. Reload Systemd Daemon, enable and start service
```bash
sudo systemctl daemon-reload
sudo systemctl enable udp_camera_streamer.service
sudo systemctl start udp_camera_streamer.service
```
Check the service status with
```bash
sudo systemctl status udp_camera_streamer.service
```
Run the previous `echo` command on the other terminal to relaunch the camera stream. If you want to see the output of the file, rerun the service status check.

# Conclusion
You now should have a camera service that with start any camera on the device with a simple JSON command sent over UDP.

# END