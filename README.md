# orb_slam3_ros

This package provides 2 simple monocular ROS2 drivers for [ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3):
* [orb_slam3_ros_mono](src/mono.cpp) supports `ORB_SLAM3::System::MONOCULAR`
* [orb_slam3_ros_mono_imu](src/mono.cpp) supports `ORB_SLAM3::System::IMU_MONOCULAR`

## Prerequisites

Requires Ubuntu 22.04 and ROS2 Humble.

ORB_SLAM3 and Pangolin v0.6 are included as git submodules, so you do not need to install them.

## Install

~~~
cd ~/workspaces/colcon_ws/src
git clone --recurse-submodules https://github.com/clydemcqueen/orb_slam3_ros.git
cd orb_slam3_ros/modules/ORB_SLAM3/Vocabulary
tar -xvf ORBvoc.txt.tar.gz
~~~

## Build

This will use Colcon to build everything, including Pangolin, ORB_SLAM3, g2o, and DBoW2:
~~~
cd ~/workspaces/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
~~~

## Test with the EuRoC MAV dataset

Download a rosbag2 from the [EuRoC MAV site](https://docs.openvins.com/gs-datasets.html), e.g., "Vicon Room 1 01"
in V1_01_easy. Unzip it.

In terminal 1:
~~~
cd ~/workspaces/colcon_ws
source install/setup.bash
ros2 launch orb_slam3_ros euroc_mav.py use_imu_data:=True show_viewer:=True
~~~

In terminal 2:
~~~
cd ~/path/to/bag
source /opt/ros/humble/setup.bash
ros2 bag play V1_01_easy
~~~

## Caveats

* There are no stereo nodes
* ORB_SLAM3 leaks RAM, see the [TODOs in Map.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Map.cc#L103) for an example
