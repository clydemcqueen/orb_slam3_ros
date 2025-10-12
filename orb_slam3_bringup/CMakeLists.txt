cmake_minimum_required(VERSION 3.12)
project(orb_slam3_bringup)

find_package(ament_cmake REQUIRED)

# Copy several directories
install(DIRECTORY launch param rviz DESTINATION share/${PROJECT_NAME})

ament_package()
