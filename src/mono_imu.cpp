#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// Heads up: modules/ORB_SLAM3/include/System.h and friends bring in the std namespace
#include "include/System.h"

#include "util.hpp"

/**
 * @class MonoImuNode
 * @brief ROS2 driver for the ORB_SLAM3::System::IMU_MONOCULAR case
 */
class MonoImuNode final : public rclcpp::Node
{
    // Declare subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Declare publications
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;

    // Declare and retrieve ROS parameters
    std::string voc_file_ = declare_parameter<std::string>("voc_file",
        "src/orb_slam3_ros/modules/ORB_SLAM3/Vocabulary/ORBvoc.txt");
    std::string settings_file_ = declare_parameter<std::string>("settings_file",
        "install/orb_slam3_ros/share/orb_slam3_ros/param/euroc_mav.yaml");
    std::string world_frame_id_ = declare_parameter<std::string>("world_frame_id",
        "world");
    bool ignore_imu_header_ = declare_parameter<bool>("ignore_imu_header",
        false);
    bool show_viewer_ = declare_parameter<bool>("show_viewer",
        false);

    // Start ORB_SLAM3
    ORB_SLAM3::System slam_{voc_file_, settings_file_, ORB_SLAM3::System::IMU_MONOCULAR, show_viewer_};

    // Keep track of SLAM state
    int tracking_state_{};

    // Split the work into 2 threads to avoid losing IMU messages:
    //      The main thread will listen for incoming messages and quickly update the internal state
    //      The SLAM thread will call ORB_SLAM3, which may take a while

    std::mutex mutex_;
    bool stop_slam_ = false;
    std::condition_variable condition_;

    // Keep just one image: if a 2nd image appears while the SLAM thread is busy, discard the older image
    sensor_msgs::msg::Image::SharedPtr image_;
    double prev_image_timestamp_{};

    // Keep all the incoming IMU messages
    std::queue<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::queue<double> imu_timestamps_;

    // Start the SLAM thread
    std::thread slam_thread_{&MonoImuNode::slam_thread_loop, this};

public:

    MonoImuNode() :
        Node("orb_slam3_mono_imu_node")
    {
        cam_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
            {
                std::lock_guard lock(mutex_);
                image_ = msg;

                // Wake up the SLAM thread
                condition_.notify_one();
            });

        // ArduSub publishes IMU data "best effort"
        auto imu_qos = rclcpp::QoS(10);
        imu_qos.best_effort();

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", imu_qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
            {
                std::lock_guard lock(mutex_);
                imu_queue_.push(msg);
                if (ignore_imu_header_) {
                    // ArduPilot DDS header.stamp can't be trusted, use the message arrival time instead
                    imu_timestamps_.push(seconds(now()));
                } else {
                    imu_timestamps_.push(seconds(msg->header.stamp));
                }
            });

        RCLCPP_INFO(get_logger(), "ORB_SLAM3 mono_imu node running");
    }

    ~MonoImuNode() override
    {
        std::unique_lock lock(mutex_);

        // Wake up the SLAM thread
        stop_slam_ = true;
        condition_.notify_one();
        lock.unlock();

        // Wait for the SLAM thread to finish
        if (slam_thread_.joinable()) {
            slam_thread_.join();
        }
    }

    void slam_thread_loop()
    {
        RCLCPP_INFO(get_logger(), "Start SLAM thread");

        while (true) {
            std::unique_lock lock(mutex_);
            condition_.wait(lock, [this] { return image_ != nullptr || stop_slam_; });

            if (stop_slam_) {
                // Write a bunch of debug data
                slam_.SaveDebugData(0);
                break;
            }

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(image_);
            }
            catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                continue;
            }

            auto image_stamp = image_->header.stamp;
            auto image_s = seconds(image_stamp);

            // Copy the IMU messages into the format that ORB_SLAM3 expects
            std::vector<ORB_SLAM3::IMU::Point> imu_points;
            while (!imu_queue_.empty()) {
                const auto imu_msg = imu_queue_.front();
                const auto imu_s = imu_timestamps_.front();

                if (imu_s > image_s) {
                    // RCLCPP_INFO(get_logger(), "IMU timestamp %g is too new, wait for next image", imu_timestamp);
                    break;
                }

                imu_queue_.pop();
                imu_timestamps_.pop();

                if (imu_s <= prev_image_timestamp_) {
                    // RCLCPP_INFO(get_logger(), "IMU timestamp %g is too old, discarding", imu_timestamp);
                    continue;
                }

                imu_points.emplace_back(
                    static_cast<float>(imu_msg->linear_acceleration.x),
                    static_cast<float>(imu_msg->linear_acceleration.y),
                    static_cast<float>(imu_msg->linear_acceleration.z),
                    static_cast<float>(imu_msg->angular_velocity.x),
                    static_cast<float>(imu_msg->angular_velocity.y),
                    static_cast<float>(imu_msg->angular_velocity.z),
                    imu_s);
            }

            image_ = nullptr;
            prev_image_timestamp_ = image_s;

            // Unblock the subscription thread
            lock.unlock();

            if (imu_points.empty()) {
                // TODO track down crash
                RCLCPP_INFO(get_logger(), "No imu measurements, drop image to avoid crash");
                continue;
            }

            // Track motion, this might be long-running
            // auto start = std::chrono::high_resolution_clock::now();
            const auto t_cam_world = slam_.TrackMonocular(cv_ptr->image, image_s, imu_points);
            // auto end = std::chrono::high_resolution_clock::now();
            // const auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

            const auto tracking_state = slam_.GetTrackingState();

            if (tracking_state != tracking_state_) {
                RCLCPP_INFO(get_logger(), "Tracking state changed from %d to %d", tracking_state_, tracking_state);
                tracking_state_ = tracking_state;
            }

            if (tracking_state == ORB_SLAM3::Tracking::eTrackingState::OK) {
                // Publish results
                publish_pose(image_stamp, t_cam_world);
                publish_map(image_stamp);
            }
        }

        RCLCPP_INFO(this->get_logger(), "End SLAM thread");
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
    const auto node = std::make_shared<MonoImuNode>();
    spin(node);
    rclcpp::shutdown();
    return 0;
}
