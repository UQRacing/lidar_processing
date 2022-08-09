# LiDAR Cone Detection
This is the new and improved LiDAR cone detection code, based on the work originally done by Riley
Bowyer and Caleb Aitken in 2020-2021.

Compared to the original, the following changes have been made:

- Does not implement the custom UQR point cloud data type, it uses ROS & Open3D types directly
- Only supports the LeiShen LiDAR used on the AV to simplify development
- Uses Open3D instead of PCL, for a better API and more performance

However, the general concept of ground plane segmentation to detect the cones is mostly left as-is.

**Original authors:**

- Riley Bowyer (riley.d.bowyer@gmail.com)
- Caleb Aitken

**Next version authors:**

- Matt Young (m.young2@uqconnect.edu.au)
- Fahed Alhanaee

The current maintainer of the repo is Matt Young.

## Building and running
Install dependencies:

- Install Eigen3: `sudo apt install libeigen3-dev`
- Install [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools): `sudo apt install ros-noetic-rviz-visual-tools`
- Install ddynamic_reconfigure: `sudo apt install ros-noetic-ddynamic-reconfigure`
- Install OpenCV. Either use the system package or [compile from source](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html). No
contrib module is required.
- Install Open3D. The recommended way is to
[click here](https://github.com/isl-org/Open3D/releases/download/v0.15.1/open3d-devel-linux-x86_64-cxx11-abi-0.15.1.tar.xz)
to download a compiled build (30MB LZMA TAR). Then:
    1. Extract the contained folder to a directory of your choosing, I use ~/build
    2. Copy open3d/lib/libOpen3D.so to /usr/local/lib (this and the below may require a root file window)
    3. Copy directory open3d/lib/cmake/Open3D/ to /usr/local/lib/cmake/
    4. Copy directory open3d/include/open3d/ to /usr/local/include/
    5. Run `sudo ldconfig`
