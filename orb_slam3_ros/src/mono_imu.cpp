#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <thread>

#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "orb_slam3_msgs/msg/slam_status.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_srvs/srv/empty.hpp"

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

    // Declare services
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;

    // Declare publications
    rclcpp::Publisher<orb_slam3_msgs::msg::SlamStatus>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr cam_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

    // Declare and retrieve ROS parameters
    std::string voc_file_ = declare_parameter<std::string>("voc_file",
        "install/orb_slam3_ros/share/orb_slam3_ros/Vocabulary/ORBvoc.txt");
    std::string settings_file_ = declare_parameter<std::string>("settings_file",
        "install/orb_slam3_bringup/share/orb_slam3_bringup/param/euroc_mav.yaml");
    std::string world_frame_id_ = declare_parameter<std::string>("world_frame_id",
        "world");
    bool ignore_imu_header_ = declare_parameter<bool>("ignore_imu_header",
        false);
    bool enable_slam_ = declare_parameter<bool>("enable_slam", true);
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Start ORB_SLAM3
    ORB_SLAM3::System slam_{voc_file_, settings_file_, ORB_SLAM3::System::IMU_MONOCULAR, false};

    // Split the work into 2 threads to avoid losing IMU messages:
    //      The main thread will listen for incoming messages and quickly update the internal state
    //      The SLAM thread will call ORB_SLAM3, which may take a while

    std::mutex mutex_;
    bool stop_slam_ = false;
    std::condition_variable condition_;

    // Protected by mutex_: keep track of SLAM state and reset/disable epochs
    int tracking_state_{};
    unsigned long map_id_{};
    bool imu_initialized_{};
    uint64_t generation_{};

    // Keep just one image: if a 2nd image appears while the SLAM thread is busy, discard the older image
    sensor_msgs::msg::Image::SharedPtr image_;
    double prev_image_timestamp_{};

    // Keep all the incoming IMU messages
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_queue_;
    std::deque<double> imu_timestamps_;

    // SLAM worker thread (started at the end of the constructor once publishers are ready)
    std::thread slam_thread_;

    void clear_queues_locked()
    {
        image_ = nullptr;
        prev_image_timestamp_ = 0.0;
        imu_queue_ = {};
        imu_timestamps_ = {};
    }

public:

    MonoImuNode() :
        Node("orb_slam3_mono_imu_node")
    {
        status_pub_ = create_publisher<orb_slam3_msgs::msg::SlamStatus>("slam_status", 10);
        cam_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("camera_pose", 10);
        map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("map_points", 10);
        image_pub_ = create_publisher<sensor_msgs::msg::Image>("annotated_image", 10);

        param_callback_handle_ = add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) -> rcl_interfaces::msg::SetParametersResult {
                return parameters_callback(params);
            });

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) -> void
            {
                std::lock_guard lock(mutex_);
                if (!enable_slam_) {
                    return;
                }
                image_ = msg;

                // Wake up the SLAM thread
                condition_.notify_one();
            });

        // Subscribe best-effort with enough depth to buffer high-rate IMU bursts
        auto imu_qos = rclcpp::QoS(100);
        imu_qos.best_effort();

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "imu", imu_qos,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) -> void
            {
                std::lock_guard lock(mutex_);
                if (!enable_slam_) {
                    return;
                }
                const double imu_s = ignore_imu_header_ ? seconds(now()) : seconds(msg->header.stamp);
                imu_queue_.push_back(msg);
                imu_timestamps_.push_back(imu_s);

                if (image_ != nullptr && imu_s >= seconds(image_->header.stamp)) {
                    condition_.notify_one();
                }
            });

        reset_service_ = create_service<std_srvs::srv::Empty>(
            "reset_slam",
            [this](const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                std::shared_ptr<std_srvs::srv::Empty::Response> response) -> void { reset(request, response); });

        slam_thread_ = std::thread{&MonoImuNode::slam_thread_loop, this};

        RCLCPP_INFO(get_logger(), "ORB_SLAM3 mono_imu node ready (enable_slam=%s)", enable_slam_ ? "true" : "false");
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

    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            if (param.get_name() == "enable_slam") {
                if (param.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = "enable_slam must be a boolean";
                    return result;
                }
                const bool new_val = param.as_bool();
                std::lock_guard lock(mutex_);
                if (new_val != enable_slam_) {
                    RCLCPP_INFO(get_logger(), "enable_slam changed from %s to %s",
                        enable_slam_ ? "true" : "false", new_val ? "true" : "false");
                    enable_slam_ = new_val;
                    if (!enable_slam_) {
                        ++generation_;
                        imu_initialized_ = false;
                        clear_queues_locked();
                        slam_.Reset();
                        if (tracking_state_ != orb_slam3_msgs::msg::SlamStatus::TRACKING_NO_IMAGES_YET) {
                            RCLCPP_INFO(get_logger(), "Tracking state changed from %d to %d",
                                tracking_state_, orb_slam3_msgs::msg::SlamStatus::TRACKING_NO_IMAGES_YET);
                            tracking_state_ = orb_slam3_msgs::msg::SlamStatus::TRACKING_NO_IMAGES_YET;

                            orb_slam3_msgs::msg::SlamStatus status_msg;
                            status_msg.header.stamp = now();
                            status_msg.header.frame_id = world_frame_id_;
                            status_msg.tracking_state = orb_slam3_msgs::msg::SlamStatus::TRACKING_NO_IMAGES_YET;
                            status_msg.map_id = map_id_;
                            status_msg.map_changed = false;
                            status_msg.imu_initialized = false;
                            status_pub_->publish(status_msg);
                        }
                    }
                }
            }
        }
        return result;
    }

    void reset(
        const std::shared_ptr<std_srvs::srv::Empty::Request>&,
        const std::shared_ptr<std_srvs::srv::Empty::Response>&)
    {
        RCLCPP_INFO(get_logger(), "Reset SLAM");
        std::lock_guard lock(mutex_);
        ++generation_;
        imu_initialized_ = false;
        clear_queues_locked();
        slam_.Reset();
    }

    void slam_thread_loop()
    {
        RCLCPP_INFO(get_logger(), "Start SLAM thread");

        while (true) {
            std::unique_lock lock(mutex_);
            condition_.wait(lock, [this] { return (enable_slam_ && image_ != nullptr) || stop_slam_; });

            if (stop_slam_) {
                break;
            }

            // Wait briefly if the matching IMU sample at/after image timestamp is still in flight
            if (imu_timestamps_.empty() || imu_timestamps_.back() < seconds(image_->header.stamp)) {
                condition_.wait_for(lock, std::chrono::milliseconds(50), [this] {
                    return stop_slam_ || !enable_slam_ || image_ == nullptr ||
                        (!imu_timestamps_.empty() && imu_timestamps_.back() >= seconds(image_->header.stamp));
                });
            }

            if (stop_slam_) {
                break;
            }

            if (!enable_slam_ || image_ == nullptr) {
                continue;
            }

            auto image_msg = image_;
            image_ = nullptr;

            const auto image_stamp = image_msg->header.stamp;
            const auto image_s = seconds(image_stamp);
            const auto gen = generation_;

            cv_bridge::CvImageConstPtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvShare(image_msg);
            }
            catch (cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
                continue;
            }

            // Discard stale IMU samples (or pre-startup backlog before the first tracked image)
            const double min_imu_s = prev_image_timestamp_ > 0.0 ? prev_image_timestamp_ : (image_s - 0.2);
            while (!imu_timestamps_.empty() && imu_timestamps_.front() <= min_imu_s) {
                imu_queue_.pop_front();
                imu_timestamps_.pop_front();
            }

            size_t avail_imu = 0;
            for (const double ts : imu_timestamps_) {
                if (ts > image_s) {
                    break;
                }
                ++avail_imu;
            }

            if (avail_imu < 2) {
                lock.unlock();
                RCLCPP_INFO(get_logger(), "Insufficient imu measurements (%zu), drop image", avail_imu);
                continue;
            }

            // Copy the IMU messages into the format that ORB_SLAM3 expects
            std::vector<ORB_SLAM3::IMU::Point> imu_points;
            imu_points.reserve(avail_imu);
            for (size_t i = 0; i < avail_imu; ++i) {
                const auto imu_msg = imu_queue_.front();
                const auto imu_s = imu_timestamps_.front();
                imu_queue_.pop_front();
                imu_timestamps_.pop_front();

                imu_points.emplace_back(
                    static_cast<float>(imu_msg->linear_acceleration.x),
                    static_cast<float>(imu_msg->linear_acceleration.y),
                    static_cast<float>(imu_msg->linear_acceleration.z),
                    static_cast<float>(imu_msg->angular_velocity.x),
                    static_cast<float>(imu_msg->angular_velocity.y),
                    static_cast<float>(imu_msg->angular_velocity.z),
                    imu_s);
            }

            prev_image_timestamp_ = image_s;

            // Unblock the subscription thread
            lock.unlock();

            // Send the image and IMU measurements, get the transform from camera frame to world frame (aka Tcw)
            const auto t_cam_world = slam_.TrackMonocular(cv_ptr->image, image_s, imu_points);

            // Get additional state information
            const auto tracking_state = slam_.GetTrackingState();
            const auto map_id = slam_.GetMapID();
            const auto map_changed = slam_.MapChanged();
            const auto imu_initialized = slam_.GetTimeFromIMUInit() > 0.0;
            const auto tracked_points = slam_.GetTrackedMapPoints();
            const auto map_points = slam_.GetCurrentMapPoints();
            const auto annotated_frame = slam_.DrawFrame();

            {
                std::lock_guard state_lock(mutex_);
                if (stop_slam_ || !enable_slam_ || generation_ != gen) {
                    continue;
                }

                if (tracking_state != tracking_state_) {
                    RCLCPP_INFO(get_logger(), "Tracking state changed from %d to %d", tracking_state_, tracking_state);
                    tracking_state_ = tracking_state;
                }

                if (map_id != map_id_) {
                    RCLCPP_INFO(get_logger(), "Map id changed from %ld to %ld", map_id_, map_id);
                    map_id_ = map_id;
                }

                if (imu_initialized != imu_initialized_) {
                    RCLCPP_INFO(get_logger(), "IMU initialized changed from %s to %s",
                        imu_initialized_ ? "true" : "false", imu_initialized ? "true" : "false");
                    imu_initialized_ = imu_initialized;
                }
            }

            if (map_changed) {
                RCLCPP_INFO(get_logger(), "Map changed");
            }

            // Invert Tcw to get the pose of the camera in the world frame
            const auto t_world_cam = t_cam_world.inverse();

            // Publish a status message, this includes the camera pose and the tracked map points
            orb_slam3_msgs::msg::SlamStatus status_msg;
            status_msg.header.stamp = image_stamp;
            status_msg.header.frame_id = world_frame_id_;
            status_msg.tracking_state = static_cast<int8_t>(tracking_state);
            status_msg.map_id = map_id;
            status_msg.map_changed = map_changed;
            status_msg.imu_initialized = imu_initialized;
            status_msg.pose = to_msg(t_world_cam);
            status_msg.tracked_points = to_msg(image_stamp, world_frame_id_, tracked_points);
            status_pub_->publish(status_msg);

            // Publish the camera pose for rviz
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.frame_id = world_frame_id_;
            pose_msg.header.stamp = image_stamp;
            pose_msg.pose = to_msg(t_world_cam);
            cam_pub_->publish(pose_msg);

            // Publish the entire map for rviz
            map_pub_->publish(to_msg(image_stamp, world_frame_id_, map_points));

            // Publish the annotated image
            auto annotated_image_msg = cv_bridge::CvImage(image_msg->header, "bgr8", annotated_frame).toImageMsg();
            image_pub_->publish(*annotated_image_msg);
        }

        RCLCPP_INFO(this->get_logger(), "End SLAM thread");
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
