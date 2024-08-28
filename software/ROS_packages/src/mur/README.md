# mur
rosbag record /camera_forward/image_raw /camera_radial_right/image_raw /camera_radial_left/image_raw /tf /model/requested_thruster_wrench -O ~/Desktop/MUR/videos/experiments/camera_tf_thruster_data_$(date +%Y%m%d_%H%M%S).bag

with compression:
rosbag record --lz4 /camera_forward/image_raw /camera_radial_right/image_raw /camera_radial_left/image_raw /tf /model/requested_thruster_wrench -O ~/Desktop/MUR/videos/experiments/camera_tf_thruster_data_$(date +%Y%m%d_%H%M%S).bag

with more compression (doesnt work):
rosbag record --bz2 /camera_forward/image_raw /camera_radial_right/image_raw /camera_radial_left/image_raw /tf /model/requested_thruster_wrench -O ~/Desktop/MUR/videos/experiments/camera_tf_thruster_data_$(date +%Y%m%d_%H%M%S).bag

run '''nload''' for network capacity