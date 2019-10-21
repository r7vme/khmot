# khmot

[![CircleCI](https://circleci.com/gh/r7vme/khmot.svg?style=svg&circle-token=1e8d4b00566db51b2cba5178e5281a2c1093b789)](https://circleci.com/gh/r7vme/khmot)

Multiple Object Tracker based on Hungarian algorithm (data association) and Kalman filter (motion estimation).

## Tracker

Tracker allows to track multiple objects in 2D plane. As input user provides array of all observed objects. Object can be described with coordinates X and Y, orientation Yaw and optionally first derivatives (velocities). As output user gets estimated state (from Kalman filter) for all tracks (objects).

On every update call to tracker it does following:
1. Predicts state for all existing tracks.
2. Computes cost matrix (Mahalanobis distance) of size (number of tracks, number of observations).
3. Estimates best associations (with Hungarian alghoritm). Then discards associations, that have cost more than threshhold (configurable).
4. Corrects state with new observations and creates new tracks if needed
5. Removes old tracks if track did not receive observation for some time (configurable).

## Kalman filter

Uses omnidirectional constant velocity model for motion estimation. Optionally supports non-omnidirectional motion (omnidirectional = false). Please make sure to properly set covariance matrix. In simple case, just set diagonal values in covariance matrix. For instance, if only X, Y, Yaw are supplied set first three diagonal values. Value depends on your error model, but for simplicity you can assume gaussian with e.g. 50cm variance, which will result in covariance value 0.25 (0.5 * 0.5). If you estimate only cars set omnidirectional to false.

# Usage

For examples please consult `main.cpp` or `tracker_test.cpp`. In the future, this library probably will have ROS support.

```
cd khmot
mkdir build
cd build
cmake ..
make -DDISABLE_ROS=TRUE
./main
```

# Credits

- [Data association techniques overview](http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-15-dataassociation.pdf)
- [Kalman filter overview](http://www.sci.utah.edu/~gerig/CS6320-S2013/Materials/MI37slides-Kalman.pdf)
- [Hungarian Alghoritm implementation by Cong Ma](https://github.com/mcximing/hungarian-algorithm-cpp)
