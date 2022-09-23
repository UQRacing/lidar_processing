# LiDAR Processing
This is the new and improved LiDAR processing pipeline. It takes in a LiDAR point cloud, camera
parameters, and spits out a depth image that can be used in combination with a 2D cone detector
to estimate 3D cone positions. The code uses OpenCV, and many optimisations have been tried in
an effort to have it be ideally as fast as a "no-op"; that is, it runs as fast as the LiDAR can (10-20 Hz).

The pipeline includes various features such as morphological operators, advanced image inpainting and
colour or greyscale depth publishing and various debugging features. This can all be controlled from
a single YAML file.

The previous LiDAR processing approach was to detect cones directly in the point cloud. This approach 
may still be used if the 2D cone detector is too inaccurate or slow.

**Next version authors (2022-):**

- Matt Young (m.young2@uqconnect.edu.au): depth map generation code and optimisations
- Tom Day: original Python VAPE code, which this repo is basically a C++ port of
- Fahed Alhanaee: detecting cones in 3D point cloud (see the "open3d" branch)

**Original authors (2020-2021):**

- Riley Bowyer (riley.d.bowyer@gmail.com)
- Caleb Aitken (caleb@jasa.id.au)

The current maintainer of the repo is Matt Young.

## Building and running
Somewhat amazingly this project requires no dependencies outside of those shipped with ROS Noetic
by default.

You can edit parameters in config/lidar_processing.yaml, it's all documented inline. The main things you can do in there
are to change the topics that data gets published to, and enable/disable various pipeline features such as inpainting.

To run the pipeline use `roslaunch lidar_processing lidar_processing.launch`
