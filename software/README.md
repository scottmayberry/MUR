### Introduction

When working with custom Python modules and ROS packages in a development environment, you often need to make these resources available globally so that they can be accessed from anywhere in your terminal. This is especially useful when your packages are not located in the default directories (e.g., Python packages not in `site-packages` or ROS packages outside the `catkin_ws/src` directory). The following steps will guide you on how to add your custom Python and ROS packages to the appropriate environment paths (`PYTHONPATH` and `ROS_PACKAGE_PATH`) to ensure they are correctly recognized and accessible.

### Steps to Add Python and ROS Packages to Paths

1. **Identify the Paths**:
   - **Python Packages Path**: `/path/to/your/python/packages`
   - **ROS Packages Path**: `/path/to/your/ros/packages`

2. **Edit `.bashrc`**:
   - Open `.bashrc` in a text editor:
     ```bash
     nano ~/.bashrc
     ```

3. **Add Paths**:
   - Append the following lines to the end of the `.bashrc` file:
     ```bash
     export PYTHONPATH=$PYTHONPATH:/path/to/your/python/packages
     export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/path/to/your/ros/packages
     ```

4. **Apply the Changes**:
   - Save the file and reload the `.bashrc`:
     ```bash
     source ~/.bashrc
     ```

5. **Rebuild and Source Workspace (For ROS)**:
   - If ROS packages were moved, rebuild your workspace:
     ```bash
     cd /path/to/your/catkin_ws
     catkin_make
     source devel/setup.bash
     ```

### Verification:

- **Check Python Path**:
  ```bash
  echo $PYTHONPATH
  ```

- **Check ROS Package Path**:
  ```bash
  echo $ROS_PACKAGE_PATH
  ```

Following these steps will ensure your custom Python and ROS packages are correctly added to the environment paths, making them accessible from any location in your terminal.