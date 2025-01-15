```bash
nano udp_camera_streamer.py
```
and copy in the below code

```python
import socket
import json
import subprocess
import time
import os

def get_script_dir():
    return os.path.dirname(os.path.realpath(__file__))

def get_video_device_by_id_path_tag(id_path_tag):
    devices = []
    for i in range(10):  # Assuming a maximum of 10 video devices
        device = f"/dev/video{i}"
        try:
            result = subprocess.run(
                ["udevadm", "info", "--query=all", "--name=" + device],
                capture_output=True, text=True, check=True
            )
            if f"ID_PATH_TAG={id_path_tag}" in result.stdout:
                devices.append(device)
        except subprocess.CalledProcessError:
            continue
    return devices

def launch_mjpg_streamer(device, width, height, fps, stream_port):
    script_dir = get_script_dir()
    plugin_dir = os.path.join(script_dir, "mjpg-streamer/mjpg-streamer-experimental")
    input_plugin = os.path.join(plugin_dir, "input_uvc.so")
    output_plugin = os.path.join(plugin_dir, "output_http.so")
    www_folder = os.path.join(plugin_dir, "www")
    
    # Check if the plugins exist
    if not os.path.exists(input_plugin):
        print(f"ERROR: {input_plugin} does not exist")
        return False
    if not os.path.exists(output_plugin):
        print(f"ERROR: {output_plugin} does not exist")
        return False

    command = [
        "/usr/local/bin/mjpg_streamer",
        "-i", f"{input_plugin} -d {device} -r {width}x{height} -f {fps}",
        "-o", f"{output_plugin} -w {www_folder} -p {stream_port}"
    ]
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = plugin_dir
    try:
        subprocess.Popen(command, env=env)
        print(f"Successfully launched mjpg_streamer on {device}")
        return True
    except Exception as e:
        print(f"Failed to launch mjpg_streamer on {device}: {e}")
        return False

def main():
    UDP_IP = ""
    UDP_PORT = 5005  # Change this to your desired port

    launched_tags = set()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    print(f"Listening for UDP messages on {UDP_IP}:{UDP_PORT}")

    while True:
        data, addr = sock.recvfrom(1024)
        try:
            message = json.loads(data.decode('utf-8'))
            id_path_tag = message["id_path_tag"]
            width = message["width"]
            height = message["height"]
            fps = message["fps"]
            stream_port = message["stream_port"]

            if id_path_tag in launched_tags:
                print(f"Ignoring message for already launched ID_PATH_TAG {id_path_tag}")
                continue

            devices = get_video_device_by_id_path_tag(id_path_tag)
            if not devices:
                print(f"No devices found for ID_PATH_TAG {id_path_tag}")
                continue

            for device in devices:
                if launch_mjpg_streamer(device, width, height, fps, stream_port):
                    launched_tags.add(id_path_tag)
                    break

        except json.JSONDecodeError:
            print("Received invalid JSON message")

        time.sleep(0.05)  # Reduce CPU usage

if __name__ == "__main__":
    main()
```

Make it executable.
```bash
chmod +x udp_camera_streamer.py
```

Run
```bash
python ./udp_camera_streamer.py
```

Example Message sent from the running device
```bash
echo '{"id_path_tag": "platform-fd500000_pcie-pci-0000_01_00_0-usb-0_1_3_1_0", "width": 640, "height": 480, "fps": 30, "stream_port": 8080}' | nc -u -w1 127.0.0.1 5005
```

### IMPORTANT: Not all requested camera settings are valid.
Not all fps and resolutions are valid. Please run
```bash
v4l2-ctl --device=/dev/video0 --list-formats-ext
```
to get the available formats you should pass as the json.