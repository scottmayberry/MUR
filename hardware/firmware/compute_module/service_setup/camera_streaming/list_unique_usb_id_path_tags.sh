#!/bin/bash
# list_unique_usb_id_path_tags.sh
#
# Author: Scott Mayberry
# Date: 2025-03-02
#
# Description:
#     This script lists all unique USB ID_PATH_TAGs from video devices found under /dev/video*.
#     It iterates through each video device, uses 'udevadm' to query device attributes, and 
#     checks if the device is connected via USB. For USB devices, it extracts the ID_PATH_TAG 
#     and stores each unique tag in an associative array. Finally, it prints all unique tags.
#
# Key Functionalities:
#     1. Iterates over all /dev/video* devices.
#     2. Uses 'udevadm' to retrieve device information.
#     3. Checks if the device's ID_BUS attribute equals "usb".
#     4. Extracts the ID_PATH_TAG attribute for USB devices.
#     5. Stores unique ID_PATH_TAGs in an associative array.
#     6. Outputs each unique tag.
#
# Dependencies:
#     - bash: The shell used to run the script.
#     - udevadm: Utility to query device information.
#
# License:
#     MIT License

# Declare an associative array to store unique ID_PATH_TAGs.
declare -A id_path_tags

# Loop through all video devices in /dev
for device in /dev/video*; do
    # Check if the device exists.
    if [[ -e "$device" ]]; then
        # Retrieve the ID_BUS attribute of the device using udevadm.
        id_bus=$(udevadm info --query=all --name="$device" | grep 'ID_BUS' | cut -d '=' -f2)
        # Check if the device is connected via USB.
        if [[ "$id_bus" == "usb" ]]; then
            # Retrieve the ID_PATH_TAG attribute of the device.
            id_path_tag=$(udevadm info --query=all --name="$device" | grep 'ID_PATH_TAG' | cut -d '=' -f2)
            # If an ID_PATH_TAG is found, store it in the associative array.
            if [[ -n "$id_path_tag" ]]; then
                id_path_tags["$id_path_tag"]=1
            fi
        fi
    fi
done

# Loop through the associative array and print each unique ID_PATH_TAG.
for tag in "${!id_path_tags[@]}"; do
    echo "$tag"
done
