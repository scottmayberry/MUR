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