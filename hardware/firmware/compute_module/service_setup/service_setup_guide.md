# Service Setup Guide

This guide provides an overview of the services required for IP broadcasting and camera streaming, which are essential for ROS auto-networking and enabling cameras to stream over ethernet. For detailed instructions on setup, navigate to the corresponding subfolder (`broadcast_ip` or `camera_streaming`) in their directories. The services include:

- **broadcast_ip.service**: This service broadcasts the device's IP address across the network, allowing for automatic discovery of ROS nodes.
- **camera_streaming.service**: This service streams video from connected USB cameras over ethernet, enabling access to the video feed by a tethered computer.

Refer to the specific subfolder for step-by-step setup instructions. Recommend to install all of them.
