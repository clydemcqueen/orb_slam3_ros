#pragma once

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "sophus/se3.hpp"
#include "../modules/ORB_SLAM3/include/MapPoint.h"

double seconds(const builtin_interfaces::msg::Time &stamp);

geometry_msgs::msg::Pose to_msg(Sophus::SE3f T);

sensor_msgs::msg::PointCloud2 to_msg(
    builtin_interfaces::msg::Time stamp,
    const std::string &frame_id,
    const std::vector<ORB_SLAM3::MapPoint *> &map_points);