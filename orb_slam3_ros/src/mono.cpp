#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orb_slam3_msgs/msg/slam_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Heads up: modules/ORB_SLAM3/include/System.h and friends bring in the std namespace
#include "include/System.h"

#include "util.hpp"

/**
 * @class MonoNode
 * @brief Simple ROS2 driver for the ORB_SLAM3::System::MONOCULAR case
 */
class MonoNode final : public rclcpp::Node
{
    // Declare subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    // Declare publications
    rclcpp::Publisher<orb_slam3_msgs::msg::SlamStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    // Declare and retrieve ROS parameters
    std::string voc_file_ = declare_parameter<std::string>("voc_file",
        "install/orb_slam3_ros/share/orb_slam3_ros/Vocabulary/ORBvoc.txt");
    std::string settings_file_ = declare_parameter<std::string>("settings_file",
        "install/orb_slam3_bringup/share/orb_slam3_bringup/param/euroc_mav.yaml");
    std::string world_frame_id_ = declare_parameter<std::string>("world_frame_id",
        "world");

    // Start ORB_SLAM3
    ORB_SLAM3::System slam_{voc_file_, settings_file_, ORB_SLAM3::System::MONOCULAR, false};

    // Keep track of the SLAM state
    int tracking_state_{};
    unsigned long map_id_{};

public:

    MonoNode() :
        Node("orb_slam3_mono_node")
    {
        status_pub_ = create_publisher<orb_slam3_msgs::msg::SlamStatus>("slam_status", 10);
        cam_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void { process_image(msg); });
    }

    void process_image(const sensor_msgs::msg::Image::SharedPtr image_msg)
    {
        const auto image_timestamp = seconds(image_msg->header.stamp);

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(image_msg);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Send the image, get the transform from the camera frame to the world frame (aka Tcw)
        // const auto start = std::chrono::high_resolution_clock::now();
        const auto t_cam_world = slam_.TrackMonocular(cv_ptr->image, image_timestamp);
        // const auto end = std::chrono::high_resolution_clock::now();
        // const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

        const auto tracking_state = slam_.GetTrackingState();
        const auto map_id = slam_.GetMapID();
        const auto map_changed = slam_.MapChanged();

        if (tracking_state != tracking_state_) {
            RCLCPP_INFO(get_logger(), "Tracking state changed from %d to %d", tracking_state_, tracking_state);
            tracking_state_ = tracking_state;
        }

        if (map_id != map_id_) {
            RCLCPP_INFO(get_logger(), "Map id changed from %ld to %ld", map_id_, map_id);
            map_id_ = map_id;
        }

        if (map_changed) {
            RCLCPP_INFO(get_logger(), "Map changed");
        }

        if (tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
            // Publish results
            publish_pose(image_msg->header.stamp, t_cam_world);  // TODO always publish?
            publish_map(image_msg->header.stamp);
        }

        // TODO refactor

        orb_slam3_msgs::msg::SlamStatus status_msg;
        status_msg.header.stamp = image_msg->header.stamp;
        status_msg.header.frame_id = world_frame_id_;

        status_msg.tracking_state = static_cast<int8_t>(tracking_state);
        status_msg.map_id = map_id;
        status_msg.map_changed = map_changed;

        // Invert Tcw to get the pose of the camera in the world frame
        const auto t_world_cam = t_cam_world.inverse();
        status_msg.pose = to_msg(t_world_cam);

        // Be sure to send tracked points, smaller
        const auto tracked_points = slam_.GetTrackedMapPoints();
        const auto msg = to_msg(image_msg->header.stamp, world_frame_id_, tracked_points);
        status_msg.tracked_points = msg;

        status_pub_->publish(status_msg);
    }

    void publish_pose(const builtin_interfaces::msg::Time stamp, const Sophus::SE3f& t_cam_world) const
    {
        // Invert Tcw to get the pose of the camera in the world frame
        const auto t_world_cam = t_cam_world.inverse();

        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = world_frame_id_;
        pose_msg.header.stamp = stamp;
        pose_msg.pose = to_msg(t_world_cam);
        cam_pub_->publish(pose_msg);
    }

    void publish_map(const builtin_interfaces::msg::Time stamp)
    {
        // Publish the current map, this will include all points in the map, not just tracked points
        // const auto map_points = slam_.GetTrackedMapPoints();
        const auto map_points = slam_.GetCurrentMapPoints();
        const auto msg = to_msg(stamp, world_frame_id_, map_points);
        map_pub_->publish(msg);
    }
};

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<MonoNode>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
