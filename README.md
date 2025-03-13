# orb_slam3_ros

This package provides 2 simple ROS2 nodes for [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3):
* [orb_slam3_ros_mono](src/mono.cpp) supports `ORB_SLAM3::System::MONOCULAR`
* [orb_slam3_ros_mono_imu](src/mono.cpp) supports `ORB_SLAM3::System::IMU_MONOCULAR`

This is a work-in-progress.

Requires Ubuntu 22.04 and ROS Humble.

Tested with data from 2 sources:
* A Gazebo Harmonic simulation with a simulated down-facing camera and a simulated IMU
* Images taken by a down-facing GoPro Hero2 ~1m above the benthos in the Salish Sea

A [fork of the ORB_SLAM3 repo](https://github.com/clydemcqueen/ORB_SLAM3/tree/ros2_humble) is included as a git submodule.
This fork has a few significant changes:
* The dependency on Pangolin is removed to simplify the installation
* The `CMakeLists.txt` is rewritten to build just the ORB_SLAM3 library, and to simplify the build process
* A few crashing bugs are fixed (but there are more to find and fix)

Caveats:
* There are no stereo nodes
* The IMU_MONOCULAR case works, but the resulting scale is way off
* The ORB_SLAM3 build generates a bunch of warnings that are suppressed
* ORB_SLAM3 leaks RAM, see the [TODOs in Map.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Map.cc#L103) for example
