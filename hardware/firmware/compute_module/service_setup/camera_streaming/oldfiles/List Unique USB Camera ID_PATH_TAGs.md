## List Unique USB Camera ID_PATH_TAGs

This script reads the `ID_PATH_TAG` from all `/dev/video*` devices that are USB cameras and prints all unique ones to the console. This enables dynamic camera launching by using the `ID_PATH_TAG` as a camera identifier.

### Bash Script

1. **Create the Script**:
   Save the following script to a file, for example, `list_unique_usb_id_path_tags.sh`.

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

2. **Make the Script Executable**:
    ```sh
    chmod +x list_unique_usb_id_path_tags.sh
    ```

3. **Run the Script**:
    ```sh
    ./list_unique_usb_id_path_tags.sh
    ```

### Explanation

- **Filter by `ID_BUS`**: The script checks if the `ID_BUS` attribute of each `/dev/video*` device is `usb`. Only USB devices are considered.
- **Store Unique `ID_PATH_TAG`**: The script extracts the `ID_PATH_TAG` for each USB device and stores it in an associative array to ensure uniqueness.
- **Print Unique Tags**: Finally, the script prints all unique `ID_PATH_TAG` values.

### Usage

1. Save the script to a file.
2. Make the script executable.
3. Run the script to print all unique `ID_PATH_TAG` values for USB cameras.
4. The printed `ID_PATH_TAG` can be copied into the ROS MUR model config file to identify each camera uniquely.