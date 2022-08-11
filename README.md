# LiDAR Processing
This is the new and improved LiDAR processing pipeline. It takes in a LiDAR point cloud, camera
parameters, and spits out a depth image that can be used in combination with a 2D cone detector
to estimate 3D cone positions. The code uses OpenCV

The original solution was to detect cones directly in the point cloud. This approach may still
be used if the 2D cone detector is too inaccurate or slow.

**Original authors (2020-2021):**

- Riley Bowyer (riley.d.bowyer@gmail.com)
- Caleb Aitken

**Next version authors (2022-):**

- Matt Young (m.young2@uqconnect.edu.au): depth map generation code and optimisations
- Tom Day: original Python VAPE code, which this repo is basically a C++ port of
- Fahed Alhanaee: detecting cones in 3D point cloud (see the "open3d" branch)

The current maintainer of the repo is Matt Young.

## Building and running
Install dependencies:

- Install Eigen3: `sudo apt install libeigen3-dev`
- Install [rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools): `sudo apt install ros-noetic-rviz-visual-tools`
- Install ddynamic_reconfigure: `sudo apt install ros-noetic-ddynamic-reconfigure`
- Install OpenCV. Either use the system package or [compile from source](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html). No
contrib module is required.
- _(Not required)_ Install Open3D. The recommended way is to
[click here](https://github.com/isl-org/Open3D/releases/download/v0.15.1/open3d-devel-linux-x86_64-cxx11-abi-0.15.1.tar.xz)
to download a compiled build (30MB LZMA TAR). Then:
    1. Extract the contained folder to a directory of your choosing, I use ~/build
    2. Copy open3d/lib/libOpen3D.so to /usr/local/lib (this and the below may require a root file window)
    3. Copy directory open3d/lib/cmake/Open3D/ to /usr/local/lib/cmake/
    4. Copy directory open3d/include/open3d/ to /usr/local/include/
    5. Run `sudo ldconfig`

You can edit parameters in config/lidar_processing.yaml, it's all documented inline.

To run the pipeline use `roslaunch lidar_processing lidar_processing.launch`