
# Step 1: Update the System
```bash
sudo apt-get update
sudo apt-get upgrade
```

# Step 2: Install Dependencies
```bash
sudo apt-get install build-essential libjpeg62-turbo-dev libv4l-dev cmake git netcat-openbsd
```
# Step 3: Download and Compile mjpg-streamer
```bash
cd /opt/udp_camera_streamer
sudo git clone https://github.com/jacksonliam/mjpg-streamer.git
cd mjpg-streamer/mjpg-streamer-experimental
sudo make
sudo make install
```

# Step 4: Run mjpg-streamer
```bash
./mjpg_streamer -i "./input_uvc.so -d /dev/video0 -r 640x480 -f 30" -o "./output_http.so -w ./www -p 8080"
```

# Step 5: Test

Check in browser by going to  http://<Raspberry_Pi_IP>:8080. Camera controls on html page allow for changing camera parameters.

or

Read stream using opencv

```python
import cv2

# Replace with your Raspberry Pi's IP address and the port used by mjpg-streamer

stream_url = "http://<Raspberry_Pi_IP>:8080/?action=stream"

# Open the video stream
cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Display the resulting frame
    cv2.imshow('Video Stream', frame)

    # Exit the stream window by pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release the capture when done
cap.release()
cv2.destroyAllWindows()
```

Replace <Raspberry_Pi_IP> with the IP address of your Raspberry Pi when accessing the stream in your browser.