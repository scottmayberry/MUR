import cv2
import numpy as np
from tqdm import tqdm

# Function to create an image with multiple ArUco tags and a vertical descriptor
def create_aruco_image(
    full_image_size_ft,
    aruco_tag_size_mm,
    num_rows,
    num_columns,
    top_bottom_border_mm,
    side_border_mm,
    start_id,
    output_file,
    text_size_mm=10,                   # Height of the text in millimeters
    text_thickness=10,
    text_border_left_clearance_mm=10,  # Clearance from the left edge in millimeters
    text_border_bottom_clearance_mm=10 # Clearance from the bottom edge in millimeters
):
    # Convert sizes to pixels (assuming 600 DPI for high resolution)
    dpi = 600
    mm_per_inch = 25.4
    pixels_per_mm = dpi / mm_per_inch

    # Convert full image size to pixels
    full_image_width_pixels = int(full_image_size_ft[0] * 12 * dpi)
    full_image_height_pixels = int(full_image_size_ft[1] * 12 * dpi)

    # Convert aruco tag size and borders to pixels
    aruco_tag_size_pixels = int(aruco_tag_size_mm * pixels_per_mm)
    top_bottom_border_pixels = int(top_bottom_border_mm * pixels_per_mm)
    side_border_pixels = int(side_border_mm * pixels_per_mm)
    text_size_pixels = int(text_size_mm * pixels_per_mm)
    text_border_left_pixels = int(text_border_left_clearance_mm * pixels_per_mm)
    text_border_bottom_pixels = int(text_border_bottom_clearance_mm * pixels_per_mm)

    # Calculate the available width and height for placing ArUco tags
    available_width = full_image_width_pixels - 2 * side_border_pixels
    available_height = full_image_height_pixels - 2 * top_bottom_border_pixels

    # Calculate spacing between the tags
    horizontal_spacing = (available_width - (num_columns * aruco_tag_size_pixels)) / (num_columns - 1) if num_columns > 1 else 0
    vertical_spacing = (available_height - (num_rows * aruco_tag_size_pixels)) / (num_rows - 1) if num_rows > 1 else 0

    # Calculate the starting offset to center the grid of tags
    horizontal_offset = (available_width - ((num_columns - 1) * horizontal_spacing + num_columns * aruco_tag_size_pixels)) // 2
    vertical_offset = (available_height - ((num_rows - 1) * vertical_spacing + num_rows * aruco_tag_size_pixels)) // 2

    # Create a blank transparent image (RGBA)
    image = np.zeros((full_image_height_pixels, full_image_width_pixels, 4), dtype=np.uint8)
    image[:, :, 3] = 0  # Set alpha channel to 0 (fully transparent)

    # Load the ArUco dictionary
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

    # Generate and place ArUco tags
    current_id = start_id
    for row in range(num_rows):
        for col in range(num_columns):
            current_id += 1

    # Determine the text descriptor
    end_id = current_id - 1

    if start_id == end_id:
        descriptor = f'Dictionary: 4x4_50, ID: {start_id}'
    else:
        descriptor = f'Dictionary: 4x4_50, IDs: {start_id}-{end_id}'

    descriptor += f', Est tag dim: {aruco_tag_size_mm} mm'

    # Create an image for the text (to rotate it)
    text_font = cv2.FONT_HERSHEY_SIMPLEX
    (text_width, text_height), baseline = cv2.getTextSize(descriptor, text_font, 1, text_thickness)

    # Scale the text size to the desired height in pixels
    scale = text_size_pixels / text_height

    # Recalculate the text size after scaling
    (text_width, text_height), baseline = cv2.getTextSize(descriptor, text_font, scale, text_thickness)

    # Create a blank image for the text (to rotate it)
    text_img_width = int(image.shape[0]*7/8) # Width becomes height after rotation
    text_img_height = text_width + baseline  # Height is width plus baseline after rotation
    text_img = np.zeros((text_img_height, text_img_width, 4), dtype=np.uint8)
    text_img[:, :, 3] = 0  # Set alpha channel to 0 (fully transparent)

    # Place the text onto the text image
    cv2.putText(text_img, descriptor, (0, text_width), text_font, scale, (0, 0, 0, 255), text_thickness, lineType=cv2.LINE_AA)

    # Rotate the text image to make it vertical
    text_img = cv2.rotate(text_img, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Determine the position to place the text on the main image
    text_x = text_border_left_pixels  # Position based on left border of the full image
    text_y = full_image_height_pixels - text_img.shape[0] - text_border_bottom_pixels  # Position based on bottom border

    # Place the rotated text image onto the main image
    image[text_y:text_y + text_img.shape[0], image.shape[1]-text_img.shape[1]-text_border_left_pixels:image.shape[1]-text_border_left_pixels] = text_img

    # Generate and place ArUco tags
    current_id = start_id
    for row in range(num_rows):
        for col in range(num_columns):
            tag_image = cv2.aruco.generateImageMarker(aruco_dict, current_id, aruco_tag_size_pixels)
            start_x = int(side_border_pixels + horizontal_offset + col * (aruco_tag_size_pixels + horizontal_spacing))
            start_y = int(top_bottom_border_pixels + vertical_offset + row * (aruco_tag_size_pixels + vertical_spacing))

            # Create an RGBA version of the tag (white background, black tag)
            tag_rgba = np.zeros((aruco_tag_size_pixels, aruco_tag_size_pixels, 4), dtype=np.uint8)
            tag_rgba[:, :, 0:3] = 255  # Set RGB to white
            tag_rgba[:, :, 3] = 255    # Set alpha to 255 (opaque)
            tag_rgba[tag_image == 0] = [0, 0, 0, 255]  # Set the tag's black areas

            # Place the tag image onto the transparent background
            image[start_y:start_y + aruco_tag_size_pixels, start_x:start_x + aruco_tag_size_pixels] = tag_rgba
            current_id += 1

    # Save the image as a PNG file
    cv2.imwrite(output_file, image)

# Loop to create multiple ArUco images with progress bar
for i in tqdm(range(50), desc="Generating ArUco Images"):
    # Example usage
    create_aruco_image(
        full_image_size_ft=(3, 2),  # Full image size in feet (width, height)
        aruco_tag_size_mm=550,       # ArUco tag size in millimeters
        num_rows=1,                 # Number of rows of ArUco tags
        num_columns=1,              # Number of columns of ArUco tags
        top_bottom_border_mm=25.4,   # Top and bottom border clearance in millimeters
        side_border_mm=100,         # Side border clearance in millimeters
        start_id=i,                 # Starting ArUco tag ID
        output_file=f'tag_generator/banners/aruco_grid_{i}.png',  # Output file name
        text_size_mm=35,            # Height of the text in millimeters
        text_thickness=60,
        text_border_left_clearance_mm=25.4,  # Clearance from the left edge in millimeters
        text_border_bottom_clearance_mm=25.4 # Clearance from the bottom edge in millimeters
    )
