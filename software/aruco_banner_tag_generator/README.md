# ArUco Banner Tag Generator

<p align="center">
  <img src="./images/example_banners/example_banner.png" alt="MUR Banner Render" width="50%"/>
</p>

The **ArUco Tag Generator** is a component of the **Miniature Underwater Robot (MUR)** project. This module is responsible for generating and verifying ArUco tag images used in the robot's banners. These tags are essential for accurate localization and navigation during underwater operations.

## Table of Contents

- [ArUco Banner Tag Generator](#aruco-banner-tag-generator)
  - [Table of Contents](#table-of-contents)
  - [Overview](#overview)
  - [Folder Structure](#folder-structure)
  - [Prerequisites](#prerequisites)
    - [Installation of Dependencies](#installation-of-dependencies)
  - [Usage](#usage)
    - [Generating ArUco Tags](#generating-aruco-tags)
      - [Steps to Generate Tags:](#steps-to-generate-tags)
      - [Customizing Tag Generation:](#customizing-tag-generation)
    - [Verifying ArUco Tags](#verifying-aruco-tags)
      - [Steps to Verify Tags:](#steps-to-verify-tags)
      - [Example Usage:](#example-usage)
  - [Output](#output)
  - [Notes](#notes)
  - [License](#license)

## Overview

The ArUco Tag Generator comprises two main scripts:

1. **`tag_generator.py`**: Generates images containing multiple ArUco tags arranged in a grid, accompanied by a vertical descriptor for identification and reference.
2. **`aruco_tag_detector_in_image.py`**: Verifies the generated ArUco tag images by detecting and highlighting the tags within them.

These tools ensure that the ArUco tags used in MUR's banners are correctly generated and functional, facilitating precise localization and navigation.

## Folder Structure

```
aruco_tag_generator/
├── banners/
│   └── [Generated ArUco Tag Images]
├── aruco_tag_detector_in_image.py
├── tag_generator.py
├── README.md
└── images/
    └── renders/
        └── render_27.png
```

- **banners/**: Directory where the generated ArUco tag images are saved.
- **aruco_tag_detector_in_image.py**: Script to verify and visualize detected ArUco tags in generated images.
- **tag_generator.py**: Script to create ArUco tag images with specified configurations.
- **images/renders/**: Contains render images used for documentation purposes.

## Prerequisites

Ensure that the following dependencies are installed before using the scripts:

- **Python 3.x**
- **OpenCV**: For image processing and ArUco tag detection.
- **NumPy**: For numerical operations.
- **tqdm**: For displaying progress bars during tag generation.

### Installation of Dependencies

You can install the required Python libraries using `pip`:

```bash
pip install opencv-python numpy tqdm
```

*Note: If you encounter issues with OpenCV installation, refer to the [OpenCV installation guide](https://docs.opencv.org/master/df/d65/tutorial_table_of_content_introduction.html).*

## Usage

### Generating ArUco Tags

The `tag_generator.py` script generates ArUco tag images based on specified parameters such as image size, tag size, grid layout, and descriptor text.

#### Steps to Generate Tags:

1. **Navigate to the Directory**:

   ```bash
   cd aruco_tag_generator/
   ```

2. **Run the Tag Generator Script**:

   ```bash
   python tag_generator.py
   ```

   This will generate a series of ArUco tag images saved in the `banners/` directory. The script uses a progress bar to indicate the generation status.

#### Customizing Tag Generation:

You can modify parameters such as image size, tag size, number of rows and columns, and descriptor text within the `tag_generator.py` script to suit your specific requirements.

### Verifying ArUco Tags

The `aruco_tag_detector_in_image.py` script verifies the generated ArUco tag images by detecting and highlighting the tags within them.

#### Steps to Verify Tags:

1. **Navigate to the Directory**:

   ```bash
   cd aruco_tag_generator/
   ```

2. **Run the Detector Script**:

   ```bash
   python aruco_tag_detector_in_image.py
   ```

   The script will process a specified image, display the detected ArUco tags, and save an annotated version of the image for review.

#### Example Usage:

By default, the script is set to verify the image located at:

```
C:\Users\Scott\Downloads\aruco_grid_0_notransparent.jpg
```

You can change the `image_path` variable in the script to point to a different image as needed.

## Output

- **Generated Images**: ArUco tag images are saved in the `banners/` directory with filenames like `aruco_grid_0.png`, `aruco_grid_1.png`, etc.
- **Annotated Images**: Verified images with detected ArUco tags highlighted are saved as `detected_aruco_tags_in_banner.png`.

## Notes

- **Resolution Settings**: The tag generator assumes a high-resolution setting (600 DPI) to ensure clear and precise ArUco tags suitable for underwater applications.
- **Descriptor Text**: The vertical descriptor provides information about the ArUco dictionary used, tag IDs, and tag dimensions, aiding in the identification and verification process.
- **Customization**: Feel free to adjust the script parameters to match the specific requirements of your application, such as changing the ArUco dictionary type or adjusting tag sizes.

## License

This project is licensed under the [MIT License](LICENSE.md). See the LICENSE file for more details.

---

*This README was last updated on 2025-01-10.*