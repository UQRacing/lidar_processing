# LiDAR Cone Detection
This is the new and improved LiDAR cone detection code, based on the work originally done by Riley
Bowyer and Caleb Aitken in 2020-2021.

Compared to the original, the following changes have been made:

- Does not implement the custom UQR point cloud data type, it uses ROS & Open3D types directly
- Only supports the LeiShen LiDAR currently
- Uses Cilantro or Open3D instead of PCL, for a better API and more performance

However, the general algorithm of ground plane segmentation to detect the cones is mostly left unchanged.

**Original authors:**

- Riley Bowyer (riley.d.bowyer@gmail.com)
- Caleb Aitken (TODO)

**Next version authors:**

- Matt Young (m.young2@uqconnect.edu.au)
- more to come :)

The current maintainer is Matt Young.

## Building and running
Install dependencies:

- Install Eigen3: `sudo apt install libeigen3-dev`
- Install Cilantro (point cloud processing library): build from the source available at https://github.com/kzampog/cilantro
    - Make sure to compile and `sudo make install` Pangolin first. Check Pangolin is compiled with Eigen enabled.
    - When finished building Cilantro, run `sudo make install` and then `sudo ldconfig`.
    - If you're not using Unix Makefiles as the CMake build target (i.e. you're using Ninja), just do `sudo ninja install`
    or equivalent.