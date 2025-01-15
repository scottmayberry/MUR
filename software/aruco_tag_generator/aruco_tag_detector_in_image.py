import cv2
import numpy as np

# Function to detect ArUco tags in an image
def detect_aruco_tags(image_path, aruco_dict_name=cv2.aruco.DICT_4X4_50):
    # Load the ArUco dictionary and parameters
    aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_name)
    aruco_params = cv2.aruco.DetectorParameters()

    # Load the image
    image = cv2.imread(image_path)
    # Resize the image to fit within a 640x480 window
    # Calculate the new height to maintain aspect ratio with a width of 640 pixels
    height, width = image.shape[:2]
    new_width = 640
    new_height = int((new_width / width) * height)
    image = cv2.resize(image, (new_width, new_height))
    if image is None:
        print("Could not load the image.")
        return

    # Convert the image to grayscale (required for ArUco detection)
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray_image, aruco_dict, parameters=aruco_params)

    # Check if any markers were found
    if ids is not None:
        print(f"Detected {len(ids)} markers:")
        for i, marker_id in enumerate(ids):
            print(f"ID: {marker_id[0]} at corners: {corners[i]}")
            # Draw the marker on the image
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
    else:
        print("No markers detected.")

    

    # Display the image with detected markers
    cv2.imshow('Detected ArUco Markers', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # Save the image with detected markers (optional)
    output_file = 'detected_aruco_tags.png'
    cv2.imwrite(output_file, image)
    print(f"Image with detected markers saved as {output_file}")

# Example usage
detect_aruco_tags(r'C:\Users\Scott\Downloads\aruco_grid_0_notransparent.jpg')
