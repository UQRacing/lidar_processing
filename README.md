# LiDAR Processing
This is the new and improved LiDAR processing pipeline for UQRacing's autonomous vehicle. 
It takes in a LiDAR point cloud, camera parameters, and spits out a depth image that can be used in 
combination with a 2D cone detector to estimate 3D cone positions. The code uses OpenCV, and many 
optimisations have been tried in an effort to have it be ideally as fast as a "no-op"; that is, 
it runs as fast as the LiDAR can (10-20 Hz).

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

## Hardware support and system integration
The LiDAR you use must have a ROS-compatible driver that outputs PointCloud2 messages,
and the camera also needs a ROS-compatible driver.

The following is the hardware we currently use:

- LiDAR: LeiShen CH128X1 (hybrid solid-state)
- Camera: FLIR Blackfly BFLY-PGE-23S6C-C
- Camera lens: Tamron 8mm C-mount fixed focus lens

If you're using a FLIR, I recommend you use [spinnaker_camera_driver](http://wiki.ros.org/spinnaker_camera_driver).

### Configuring the pipeline
You can edit parameters in config/lidar_processing.yaml, it's all documented inline. The main things 
you can do in there are to change the topics that data gets published to, enable/disable various 
pipeline features such as inpainting, and calibrate the various camera matrices. 

This should help you configure and optimise the pipeline to new devices and setups without modifying
the code.

### Intrinsic matrix and distortion calibration
The LiDAR pipeline requires an `image_geomtry::PinholeCameraModel`, usually transmitted on a topic
like `/camera/color/camera_info`, to function. Some cameras like the late Intel RealSense can calibrate
these automatically, otherwise you will need to do it yourself.

First, you will need to print off the OpenCV chessboard, which you can get [here](https://github.com/opencv/opencv/blob/4.x/doc/pattern.png).

Then, you can use ROS to calibrate using the chessboard. Here are some resources on that:

- http://wiki.ros.org/camera_calibration
- http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

_ProTip(TM):_ Make sure you change the `--size` parameter to 9x6 and the `--square` size to however big the
square is on paper. The method for calibration is basically to wave around the checkerboard in front
of the camera until the distortion goes away.

If you're using spinnaker_camera_driver, and have clicked "Commit" (or whatever it is) in the camera
calibration GUI, the camera_info topic should be automatically generated.

You probably will also want to publish the distortion-corrected camera frame, and use that in lidar_processing
instead of the default uncorrected image. Here are some links on that:

- http://wiki.ros.org/image_proc
- https://answers.ros.org/question/208724/how-to-run-image_proc-with-roslaunch/

### Extrinsic matrix calibration
Currently, this is just guess and check I think. In future we would like to auto calibrate this using
some method.

## Building and running
Somewhat amazingly this project requires no dependencies outside of those shipped with ROS Noetic
by default. You can just build it with `catkin build`.

To run the pipeline use `roslaunch lidar_processing lidar_processing.launch`

## Licence
This project is licenced under the Mozilla Public Licence v2.0.

We also use the following third-party libraries:

- [https://github.com/martinus/robin-hood-hashing](Robin Hood Hashing): MIT licence
- [https://github.com/yuki-koyama/tinycolormap](tinycolormap): MIT licence
