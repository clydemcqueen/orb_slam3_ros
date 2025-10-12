#include "util.hpp"

#include "rclcpp/time.hpp"

double seconds(const builtin_interfaces::msg::Time& stamp)
{
    const rclcpp::Time time(stamp);
    return time.seconds();
}

geometry_msgs::msg::Pose to_msg(Sophus::SE3f T)
{
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = T.translation().x();
    pose_msg.position.y = T.translation().y();
    pose_msg.position.z = T.translation().z();
    Eigen::Quaternionf q(T.unit_quaternion());
    pose_msg.orientation.x = q.x();
    pose_msg.orientation.y = q.y();
    pose_msg.orientation.z = q.z();
    pose_msg.orientation.w = q.w();
    return pose_msg;
}

sensor_msgs::msg::PointCloud2 to_msg(
    const builtin_interfaces::msg::Time stamp,
    const std::string& frame_id,
    const std::vector<ORB_SLAM3::MapPoint*>& map_points)
{
    // Count up the # of valid points
    int num_valid_points = 0;
    for (const auto mp : map_points) {
        if (mp != nullptr) {
            num_valid_points++;
        }
    }

    // Create a 1D point cloud: width = num_valid_points, height = 1
    // Each point has 3 channels: x, y, z
    constexpr int num_channels = 3;
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_id;
    cloud.height = 1;
    cloud.width = num_valid_points;
    cloud.fields.resize(num_channels);
    cloud.is_bigendian = false;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.data.resize(cloud.row_step * cloud.height);
    cloud.is_dense = true;

    // Describe the fields
    const std::string channel_id[] = {"x", "y", "z"};
    for (int i = 0; i < num_channels; i++) {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud.fields[i].count = 1;
    }

    // Walk map_points again, this time copying data
    unsigned char* cloud_data_ptr = &cloud.data[0];
    for (const auto mp : map_points) {
        if (mp != nullptr) {
            const float* eigen_data_ptr = mp->GetWorldPos().data();
            memcpy(cloud_data_ptr, eigen_data_ptr, cloud.point_step);
            cloud_data_ptr += cloud.point_step;
        }
    }

    return cloud;
}
