# aruco_tag_detector_in_image.py
# This script is used to verify the ArUco tag images generated for the MUR's banners.
# It detects ArUco tags within a given banner image, highlights them for visual confirmation,
# and saves the annotated image for review purposes.

import cv2  # OpenCV library for image processing tasks
import numpy as np  # NumPy library for numerical operations

# Function to detect ArUco tags in an image
def detect_aruco_tags(image_path, aruco_dict_name=cv2.aruco.DICT_4X4_50):
    """
    Detects ArUco tags in the specified banner image and highlights them.

    Parameters:
    - image_path (str): Path to the input banner image file.
    - aruco_dict_name (int): Predefined ArUco dictionary identifier from OpenCV (default: DICT_4X4_50).

    Returns:
    - None
    """
    # Load the specified ArUco dictionary and initialize detector parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)  # Select the ArUco dictionary for detection
    aruco_params = cv2.aruco.DetectorParameters()  # Initialize detector parameters with default settings

    # Load the banner image from the provided path
    image = cv2.imread(image_path)  # Read the image file
    if image is None:
        print("Could not load the image.")  # Error message if the image fails to load
        return  # Exit the function if image loading fails

    # Resize the image to fit within a 640x480 window while maintaining aspect ratio
    height, width = image.shape[:2]  # Get original image dimensions (height and width)
    new_width = 640  # Desired width in pixels for display purposes
    new_height = int((new_width / width) * height)  # Calculate new height to maintain aspect ratio
    image = cv2.resize(image, (new_width, new_height))  # Resize the image accordingly

    # Convert the resized image to grayscale as ArUco detection requires grayscale images
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  # Convert to grayscale

    # Detect ArUco markers in the grayscale banner image
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)
    # corners: Detected marker corner positions
    # ids: Identifiers of detected markers
    # rejected_img_points: Points that were detected but did not match any marker in the dictionary

    # Check if any ArUco markers were detected in the image
    if ids is not None:
        print(f"Detected {len(ids)} markers:")  # Print the number of detected markers
        for i, marker_id in enumerate(ids):
            print(f"ID: {marker_id[0]} at corners: {corners[i]}")  # Print each marker's ID and corner coordinates
            # Draw the detected markers on the banner image for visual verification
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
    else:
        print("No markers detected.")  # Message if no markers are found in the banner image

    # Display the banner image with detected markers highlighted
    cv2.imshow('Detected ArUco Markers', image)  # Create a window to show the image
    cv2.waitKey(0)  # Wait indefinitely until a key is pressed
    cv2.destroyAllWindows()  # Close the image display window

    # Save the annotated banner image with highlighted ArUco markers
    output_file = 'detected_aruco_tags.png'  # Define the output file name for the annotated image
    cv2.imwrite(output_file, image)  # Write the annotated image to disk
    print(f"Image with detected markers saved as {output_file}")  # Confirmation message

# Example usage of the detect_aruco_tags function
detect_aruco_tags(r'software\aruco_banner_tag_generator\banners\aruco_grid_0.png')  # Specify the path to the generated banner image
