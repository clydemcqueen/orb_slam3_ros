# To build:
# docker build --no-cache --tag orb_slam3_ros:latest .

FROM osrf/ros:humble-desktop AS base

RUN apt-get update && apt-get upgrade -y

WORKDIR /work/colcon_ws/src

RUN git clone --recurse-submodules https://github.com/clydemcqueen/orb_slam3_ros.git

WORKDIR /work/colcon_ws/src/orb_slam3_ros/modules/ORB_SLAM3/Vocabulary

RUN tar -xvf ORBvoc.txt.tar.gz

WORKDIR /work/colcon_ws

RUN rosdep install -y --from-paths . --ignore-src

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# Smoke test (does it run):
# docker run -it orb_slam3_ros:latest /bin/bash
# source install/setup.bash
# ros2 run orb_slam3_ros orb_slam3_ros_mono
