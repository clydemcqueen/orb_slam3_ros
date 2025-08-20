#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "image_geometry/stereo_camera_model.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

// Heads up: modules/ORB_SLAM3/include/System.h and friends bring in the std namespace
#include "include/System.h"

#include "util.hpp"

/**
 * @class StereoNode
 * @brief Simple ROS2 driver for the ORB_SLAM3::System::STEREO case
 * TODO we can share some code here...
 */
class StereoNode final : public rclcpp::Node
{
    // Left and right camera models
    image_geometry::PinholeCameraModel l_model_;
    image_geometry::PinholeCameraModel r_model_;

    // Left and right camera info messages
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr l_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr r_info_sub_;

    // Subscribe to the left and right image messages using a message filter
    message_filters::Subscriber<sensor_msgs::msg::Image> l_image_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> r_image_sub_;
    using sync_policy = message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
    using synchronizer = message_filters::Synchronizer<sync_policy>;
    std::shared_ptr<synchronizer> sync_;

    // Declare publications
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    // Declare and retrieve ROS parameters
    std::string voc_file_ = declare_parameter<std::string>("voc_file",
        "src/orb_slam3_ros/modules/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    std::string settings_file_ = declare_parameter<std::string>("settings_file",
        "no default");
    std::string world_frame_id_ = declare_parameter<std::string>("world_frame_id",
        "world");
    bool show_viewer_ = declare_parameter<bool>("show_viewer",
        false);

    // Start ORB_SLAM3
    ORB_SLAM3::System slam_{voc_file_, settings_file_, ORB_SLAM3::System::STEREO, show_viewer_};

    // Keep track of the SLAM state
    int tracking_state_{};

public:

    StereoNode() : Node("orb_slam3_stereo_node"),
                   l_image_sub_(this, "left/image_raw"),
                   r_image_sub_(this, "right/image_raw")
    {
        cam_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);

        l_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("left/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr l_info_msg)
            {
                l_model_.fromCameraInfo(l_info_msg);
            });

        r_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("right/camera_info", 10,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr r_info_msg)
            {
                r_model_.fromCameraInfo(r_info_msg);
            });

        sync_ = std::make_shared<synchronizer>(sync_policy(10), l_image_sub_, r_image_sub_);
        sync_->registerCallback(&StereoNode::process_image_pair, this);
    }

    void process_image_pair(
        const sensor_msgs::msg::Image::SharedPtr l_image_msg,
        const sensor_msgs::msg::Image::SharedPtr r_image_msg)
    {
        if (!l_model_.initialized() || !r_model_.initialized()) {
            RCLCPP_ERROR(get_logger(), "Camera models not initialized, dropping stereo frames");
            return;
        }

        const auto l_timestamp = seconds(l_image_msg->header.stamp);
        const auto r_timestamp = seconds(r_image_msg->header.stamp);

        if (fabs(l_timestamp - r_timestamp) > 0.01) {
            RCLCPP_ERROR(get_logger(), "timestamp delta %g exceeds 0.1s", l_timestamp - r_timestamp);
        }

        cv_bridge::CvImageConstPtr l_cv_ptr;
        cv_bridge::CvImageConstPtr r_cv_ptr;
        try {
            l_cv_ptr = cv_bridge::toCvShare(l_image_msg);
            r_cv_ptr = cv_bridge::toCvShare(r_image_msg);
        }
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // Always rectify the images
        // TODO optimize this
        image_geometry::StereoCameraModel model;
        model.fromCameraInfo(l_model_.cameraInfo(), r_model_.cameraInfo());
        cv::Mat l_rectified, r_rectified;
        model.left().rectifyImage(l_cv_ptr->image, l_rectified);
        model.right().rectifyImage(r_cv_ptr->image, r_rectified);

        // Send the images, get the transform from the camera frame to the world frame (aka Tcw)
        // const auto t_cam_world = slam_.TrackStereo(l_cv_ptr->image, r_cv_ptr->image, l_timestamp);
        const auto t_cam_world = slam_.TrackStereo(l_rectified, r_rectified, l_timestamp);

        const auto tracking_state = slam_.GetTrackingState();

        if (tracking_state != tracking_state_) {
            RCLCPP_INFO(get_logger(), "Tracking state changed from %d to %d", tracking_state_, tracking_state);
            tracking_state_ = tracking_state;
        }

        if (tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
            // Publish results
            publish_pose(l_image_msg->header.stamp, t_cam_world);
            publish_map(l_image_msg->header.stamp);
        }
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
        const auto map_points = slam_.GetTrackedMapPoints();
        const auto msg = to_msg(stamp, world_frame_id_, map_points);
        map_pub_->publish(msg);
    }
};

int main(const int argc, char** argv)
{
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<StereoNode>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
