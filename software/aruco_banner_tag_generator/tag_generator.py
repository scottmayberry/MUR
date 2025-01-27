# tag_generator.py
# This script generates images containing multiple ArUco tags arranged in a grid,
# along with a vertical descriptor text. The generated images are saved as PNG files
# for use in various applications such as robot localization and navigation.

import cv2  # OpenCV library for image processing
import numpy as np  # NumPy library for numerical operations
from tqdm import tqdm  # tqdm library for displaying progress bars

# Function to create an image with multiple ArUco tags and a vertical descriptor
def create_aruco_image(
    full_image_size_ft,  # Tuple representing the full image size in feet (width, height)
    aruco_tag_size_mm,  # Size of each ArUco tag in millimeters
    num_rows,  # Number of rows of ArUco tags
    num_columns,  # Number of columns of ArUco tags
    top_bottom_border_mm,  # Top and bottom border clearance in millimeters
    side_border_mm,  # Side border clearance in millimeters
    start_id,  # Starting ID for the first ArUco tag
    output_file,  # Path to save the generated image
    text_size_mm=10,  # Height of the descriptor text in millimeters (default: 10)
    text_thickness=10,  # Thickness of the descriptor text (default: 10)
    text_border_left_clearance_mm=10,  # Left clearance for the descriptor text in millimeters (default: 10)
    text_border_bottom_clearance_mm=10  # Bottom clearance for the descriptor text in millimeters (default: 10)
):
    # Convert sizes to pixels (assuming 600 DPI for high resolution)
    dpi = 600  # Dots per inch for high-resolution images
    mm_per_inch = 25.4  # Millimeters in one inch
    pixels_per_mm = dpi / mm_per_inch  # Conversion factor from millimeters to pixels

    # Convert full image size from feet to pixels
    full_image_width_pixels = int(full_image_size_ft[0] * 12 * dpi)  # Width in pixels
    full_image_height_pixels = int(full_image_size_ft[1] * 12 * dpi)  # Height in pixels

    # Convert ArUco tag size and border clearances from millimeters to pixels
    aruco_tag_size_pixels = int(aruco_tag_size_mm * pixels_per_mm)  # Size of ArUco tag in pixels
    top_bottom_border_pixels = int(top_bottom_border_mm * pixels_per_mm)  # Top and bottom borders in pixels
    side_border_pixels = int(side_border_mm * pixels_per_mm)  # Side borders in pixels
    text_size_pixels = int(text_size_mm * pixels_per_mm)  # Text size in pixels
    text_border_left_pixels = int(text_border_left_clearance_mm * pixels_per_mm)  # Left text clearance in pixels
    text_border_bottom_pixels = int(text_border_bottom_clearance_mm * pixels_per_mm)  # Bottom text clearance in pixels

    # Calculate the available width and height for placing ArUco tags within the image
    available_width = full_image_width_pixels - 2 * side_border_pixels  # Available width after side borders
    available_height = full_image_height_pixels - 2 * top_bottom_border_pixels  # Available height after top and bottom borders

    # Calculate spacing between the ArUco tags
    horizontal_spacing = (available_width - (num_columns * aruco_tag_size_pixels)) / (num_columns - 1) if num_columns > 1 else 0  # Horizontal spacing between tags
    vertical_spacing = (available_height - (num_rows * aruco_tag_size_pixels)) / (num_rows - 1) if num_rows > 1 else 0  # Vertical spacing between tags

    # Calculate the starting offset to center the grid of ArUco tags within the available area
    horizontal_offset = (available_width - ((num_columns - 1) * horizontal_spacing + num_columns * aruco_tag_size_pixels)) // 2  # Horizontal offset for centering
    vertical_offset = (available_height - ((num_rows - 1) * vertical_spacing + num_rows * aruco_tag_size_pixels)) // 2  # Vertical offset for centering

    # Create a blank transparent image (RGBA) with the specified full size
    image = np.zeros((full_image_height_pixels, full_image_width_pixels, 4), dtype=np.uint8)
    image[:, :, 3] = 0  # Set alpha channel to 0 (fully transparent)

    # Load the predefined ArUco dictionary (4x4 bits, 50 unique IDs)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Initialize the current ArUco tag ID
    current_id = start_id

    # Placeholder loop to increment the current_id (actual tag placement happens later)
    for row in range(num_rows):
        for col in range(num_columns):
            current_id += 1  # Increment ID for each tag

    # Determine the text descriptor based on the range of ArUco IDs generated
    end_id = current_id - 1  # Last ArUco tag ID used

    if start_id == end_id:
        descriptor = f'Dictionary: 4x4_50, ID: {start_id}'  # Single ID descriptor
    else:
        descriptor = f'Dictionary: 4x4_50, IDs: {start_id}-{end_id}'  # Range of IDs descriptor

    descriptor += f', Est tag dim: {aruco_tag_size_mm} mm'  # Append estimated tag dimension to descriptor

    # Create an image for the descriptor text (to rotate it vertically)
    text_font = cv2.FONT_HERSHEY_SIMPLEX  # Font type for the text
    (text_width, text_height), baseline = cv2.getTextSize(descriptor, text_font, 1, text_thickness)  # Get text size in pixels

    # Scale the text size to match the desired height in pixels
    scale = text_size_pixels / text_height  # Scaling factor based on desired text height

    # Recalculate the text size after scaling
    (text_width, text_height), baseline = cv2.getTextSize(descriptor, text_font, scale, text_thickness)  # Updated text size

    # Create a blank image for the text (to rotate it vertically)
    text_img_width = int(image.shape[0]*7/8)  # Width of the text image (scaled based on main image height)
    text_img_height = text_width + baseline  # Height of the text image after rotation
    text_img = np.zeros((text_img_height, text_img_width, 4), dtype=np.uint8)  # Initialize blank text image with transparency
    text_img[:, :, 3] = 0  # Set alpha channel to 0 (fully transparent)

    # Render the descriptor text onto the text image
    cv2.putText(text_img, descriptor, (0, text_width), text_font, scale, (0, 0, 0, 255), text_thickness, lineType=cv2.LINE_AA)  # Draw text in black color

    # Rotate the text image to make the text vertical
    text_img = cv2.rotate(text_img, cv2.ROTATE_90_COUNTERCLOCKWISE)  # Rotate the text image 90 degrees counter-clockwise

    # Determine the position to place the rotated text on the main image
    text_x = text_border_left_pixels  # X-coordinate based on left border clearance
    text_y = full_image_height_pixels - text_img.shape[0] - text_border_bottom_pixels  # Y-coordinate based on bottom border clearance

    # Overlay the rotated text image onto the main transparent image
    image[text_y:text_y + text_img.shape[0], image.shape[1]-text_img.shape[1]-text_border_left_pixels:image.shape[1]-text_border_left_pixels] = text_img  # Place text on the right side

    # Reset current_id to start placing ArUco tags from the initial ID
    current_id = start_id

    # Loop through each row and column to generate and place ArUco tags on the image
    for row in range(num_rows):
        for col in range(num_columns):
            # Generate an ArUco tag image with the current ID and specified size
            tag_image = cv2.aruco.generateImageMarker(aruco_dict, current_id, aruco_tag_size_pixels)
            
            # Calculate the top-left corner position for the current ArUco tag
            start_x = int(side_border_pixels + horizontal_offset + col * (aruco_tag_size_pixels + horizontal_spacing))  # X-coordinate for tag placement
            start_y = int(top_bottom_border_pixels + vertical_offset + row * (aruco_tag_size_pixels + vertical_spacing))  # Y-coordinate for tag placement

            # Create an RGBA version of the tag with a white background and black tag
            tag_rgba = np.zeros((aruco_tag_size_pixels, aruco_tag_size_pixels, 4), dtype=np.uint8)  # Initialize blank tag image with transparency
            tag_rgba[:, :, 0:3] = 255  # Set RGB channels to white
            tag_rgba[:, :, 3] = 255  # Set alpha channel to 255 (opaque)
            tag_rgba[tag_image == 0] = [0, 0, 0, 255]  # Set the tag's black areas based on the generated marker

            # Overlay the ArUco tag onto the main image at the calculated position
            image[start_y:start_y + aruco_tag_size_pixels, start_x:start_x + aruco_tag_size_pixels] = tag_rgba  # Place tag on the main image
            current_id += 1  # Increment ID for the next tag

    # Save the final image as a PNG file with transparency
    cv2.imwrite(output_file, image)  # Write the image to the specified output file

# Loop to create multiple ArUco images with a progress bar using tqdm
for i in tqdm(range(50), desc="Generating ArUco Images"):
    # Example usage of the create_aruco_image function
    create_aruco_image(
        full_image_size_ft=(3, 2),  # Full image size in feet (width, height)
        aruco_tag_size_mm=550,  # ArUco tag size in millimeters
        num_rows=1,  # Number of rows of ArUco tags
        num_columns=1,  # Number of columns of ArUco tags
        top_bottom_border_mm=25.4,  # Top and bottom border clearance in millimeters
        side_border_mm=100,  # Side border clearance in millimeters
        start_id=i,  # Starting ArUco tag ID for this image
        output_file=f'tag_generator/banners/aruco_grid_{i}.png',  # Output file path for the generated image
        text_size_mm=35,  # Height of the descriptor text in millimeters
        text_thickness=60,  # Thickness of the descriptor text
        text_border_left_clearance_mm=25.4,  # Left clearance for the descriptor text in millimeters
        text_border_bottom_clearance_mm=25.4  # Bottom clearance for the descriptor text in millimeters
    )
