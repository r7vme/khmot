# khmot

[![CircleCI](https://circleci.com/gh/r7vme/khmot.svg?style=svg&circle-token=1e8d4b00566db51b2cba5178e5281a2c1093b789)](https://circleci.com/gh/r7vme/khmot)

Multiple 2D Object Tracker based on Hungarian algorithm (data association) and Kalman filter (motion estimation).

This ROS node allows to feed raw 3D bounding boxes (e.g. from one or multiple perception nodes) and will produce world state, where every object is tracked by Kalman filter.

Features:
- Input `khmot_msgs/BoundingBoxWithCovarianceArray`.
- Output `jsk_recognition_msgs/BoundingBoxArray` (has RVIZ plugin)
- Globally optimal data association. [See why Hungarian with Mahalanobis distance costs is globally optimal](http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-15-dataassociation.pdf).
- Constant velocity omnidirectional linear Kalman filter. Also non-omnidirectional model (cars) supported.
- False positive filtering by using probabation period.
- Only 2D is supported (In general, it's quite straight forward to support 3D, may be added later).
- ROS1 or stand-alone C++ library.

TODO:
- [github issues](https://github.com/r7vme/khmot/issues)

# Build

To build only C++ library (w/o ROS).

```
cd khmot
mkdir build
cd build
cmake ..
make -DDISABLE_ROS=TRUE
./main
```
Example code in `main.cpp` or `tracker_test.cpp`.

To build ROS version of `khmot`
```
wstool set src/khmot --git https://github.com/r7vme/khmot -v master
wstool update src/khmot
rosdep install --from-paths src/khmot --ignore-src -y -r
catkin_make
roslaunch khmot khmot.launch
```

# Usage

Tracker and Kalman filter require fine-tuning. If you ever used [robot_localization](http://docs.ros.org/melodic/api/robot_localization/html/index.html), you know that it requires proper configuration and input data with meaningful covariance. I recommend first to read [robot_localization wiki](http://docs.ros.org/melodic/api/robot_localization/html/index.html) and ideally successfully integrate for your robot.

Anyway main point that your data should have good covariance and configuration options like `mahalanobis_thresh` matter a lot. All options desribed in [launch file](khmot/launch/khmot.launch).

Important limitations:
- `khmot` uses ONLY part of covariance matrix. diagonal values for `x`, `y`, `yaw`. (to be fixed)
- `khmot` does not allow to configure process noise matrix. (to be fixed)

## Tracker

Tracker allows to track multiple objects (e.g. 3D bounding boxes) in 2D plane. As input user provides array (`khmot_msgs/BoundingBoxWithCovarianceArray`) of all observed objects. Tracker uses `x`, `y` and `yaw` values for motion estimation and data association. It also estimates dimentions (`h`, `w`, `l`) by using Exponential Moving Average (EMA). As output user gets array of bounding boxes (`jsk_recognition_msgs/BoundingBoxArray`) (aka world state). NOTE: `jsk_recognition_msgs/BoundingBoxArray` can be visualized in RVIZ, this was the main reason to use these messages.

On every update call to tracker it does following:
1. Predicts state for all existing tracks.
2. Computes cost matrix (Mahalanobis distance) of size `number of tracks` x `number of observations` based on `x`, `y`, `yaw` and covariances.
3. Estimates best associations (with Hungarian alghoritm). Then discards associations, that have cost more than threshhold (configurable).
4. Corrects state with new observations and creates new tracks if needed
5. Removes old tracks if track did not receive observation for some time (configurable).
6. To filter false positives it waits 5 observations by defaults and only after sets `valid=true` on track.

## Kalman filter

Uses omnidirectional constant velocity model for motion estimation. Optionally supports non-omnidirectional motion (omnidirectional = false). Please make sure to properly set covariance matrix. In simple case, just set diagonal values in covariance matrix. For instance, if only X, Y, Yaw are supplied set first three diagonal values. Value depends on your error model, but for simplicity you can assume gaussian with e.g. 50cm variance, which will result in covariance value 0.25 (0.5 * 0.5). If you estimate only cars set omnidirectional to false.

# Credits

- [ROS robot_localization](https://github.com/cra-ros-pkg/robot_localization) for state-of-the-art Kalman filter implementation.
- [Data association techniques overview](http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-15-dataassociation.pdf)
- [Kalman filter overview](http://www.sci.utah.edu/~gerig/CS6320-S2013/Materials/MI37slides-Kalman.pdf)
- [Hungarian Alghoritm implementation by Cong Ma](https://github.com/mcximing/hungarian-algorithm-cpp)

# DISCLAIMER

This repo is MVP that i've created for my own robot, but aiming to make it usable for the ROS community.
