cmake_minimum_required(VERSION 3.12)
project(orb_slam3_ros)

# Hack: g2o crashes if CMAKE_BUILD_TYPE is Release, but not for Debug (slow) or RelWithDebInfo (fast)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

message(STATUS "${PROJECT_NAME} build type: " ${CMAKE_BUILD_TYPE})

# Also build Pangolin
add_subdirectory(modules/Pangolin)

# Also build ORB_SLAM3 and ORB_SLAM3/Thirdparty
add_subdirectory(modules/ORB_SLAM3)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Turn off pedantic for now
  # add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Pangolin REQUIRED)

# Some of the ORB_SLAM3 headers generate warnings, disable these for now
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-parentheses")

# Mono driver
add_executable(orb_slam3_ros_mono
  src/mono.cpp
  src/util.cpp)

target_include_directories(orb_slam3_ros_mono
  PRIVATE
  ${DBoW2_SOURCE_DIR}
  ${ORB_SLAM3_SOURCE_DIR}
  ${ORB_SLAM3_SOURCE_DIR}/include
  ${ORB_SLAM3_SOURCE_DIR}/include/CameraModels
  ${Sophus_SOURCE_DIR}
)

target_link_libraries(orb_slam3_ros_mono
  PRIVATE
  cv_bridge::cv_bridge
  Eigen3::Eigen
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  ORB_SLAM3
)

# Mono IMU driver
add_executable(orb_slam3_ros_mono_imu
  src/mono_imu.cpp
  src/util.cpp)

target_include_directories(orb_slam3_ros_mono_imu
  PRIVATE
  ${DBoW2_SOURCE_DIR}
  ${ORB_SLAM3_SOURCE_DIR}
  ${ORB_SLAM3_SOURCE_DIR}/include
  ${ORB_SLAM3_SOURCE_DIR}/include/CameraModels
  ${Sophus_SOURCE_DIR}
)

target_link_libraries(orb_slam3_ros_mono_imu
  PRIVATE
  cv_bridge::cv_bridge
  Eigen3::Eigen
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  ORB_SLAM3
)

# libORB_SLAM3.so is built in colcon_ws/src/orb_slam3_ros/modules/ORB_SLAM3
# libg2o.so       is built in colcon_ws/src/orb_slam3_ros/modules/ORB_SLAM3/Thirdparty/g2o/lib
# libDBoW2.so     is built in colcon_ws/src/orb_slam3_ros/modules/ORB_SLAM3/Thirdparty/DBoW2/lib

# libpangolin.so  is installed to colcon_ws/install/orb_slam3_ros/lib

# Use a hook to set up LD_LIBRARY_PATH so the linker can find the so files
ament_environment_hooks(hooks/setup.dsv.in)

install(TARGETS orb_slam3_ros_mono orb_slam3_ros_mono_imu
  DESTINATION lib/${PROJECT_NAME})

# Copy several directories
install(DIRECTORY launch param rviz DESTINATION share/${PROJECT_NAME})

ament_package()
