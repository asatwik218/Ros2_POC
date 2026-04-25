#include "cynlr_camera/stereo_camera_node.hpp"

#include <cmath>

namespace cynlr::camera {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

StereoCameraNode::StereoCameraNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("stereo_camera_node", options)
{
    declare_parameters();
}

StereoCameraNode::~StereoCameraNode() {
    if (camera_open_) close_camera();
}

void StereoCameraNode::declare_parameters() {
    this->declare_parameter("camera_index",    0);
    this->declare_parameter("publish_rate",    30.0);
    this->declare_parameter("frame_id_left",   std::string(""));
    this->declare_parameter("frame_id_right",  std::string(""));
    this->declare_parameter("calibration_url", std::string(""));
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

CallbackReturn StereoCameraNode::on_configure(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Configuring...");

    camera_index_ = static_cast<int>(this->get_parameter("camera_index").as_int());
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    const std::string default_left  = "camera_" + std::to_string(camera_index_) + "_left_optical";
    const std::string default_right = "camera_" + std::to_string(camera_index_) + "_right_optical";

    frame_id_left_  = this->get_parameter("frame_id_left").as_string();
    frame_id_right_ = this->get_parameter("frame_id_right").as_string();
    if (frame_id_left_.empty())  frame_id_left_  = default_left;
    if (frame_id_right_.empty()) frame_id_right_ = default_right;

    const std::string ns = "camera_" + std::to_string(camera_index_);
    const std::string cal_url = this->get_parameter("calibration_url").as_string();

    cinfo_left_  = std::make_shared<camera_info_manager::CameraInfoManager>(
        this->get_node_base_interface().get(), ns + "_left", cal_url);
    cinfo_right_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        this->get_node_base_interface().get(), ns + "_right", cal_url);

    const auto reliable_qos = rclcpp::QoS(1).reliable();
    const auto image_qos    = rclcpp::QoS(1).best_effort().durability_volatile();

    pub_left_image_  = this->create_publisher<sensor_msgs::msg::Image>(ns + "/left/image_raw",   image_qos);
    pub_right_image_ = this->create_publisher<sensor_msgs::msg::Image>(ns + "/right/image_raw",  image_qos);
    pub_left_info_   = this->create_publisher<sensor_msgs::msg::CameraInfo>(ns + "/left/camera_info",  reliable_qos);
    pub_right_info_  = this->create_publisher<sensor_msgs::msg::CameraInfo>(ns + "/right/camera_info", reliable_qos);
    pub_points_      = this->create_publisher<sensor_msgs::msg::PointCloud2>(ns + "/points",     image_qos);

    if (!open_camera()) {
        RCLCPP_ERROR(get_logger(), "Failed to open camera %d", camera_index_);
        return CallbackReturn::FAILURE;
    }
    camera_open_ = true;

    RCLCPP_INFO(get_logger(), "Configured (camera_index=%d, rate=%.1f Hz)", camera_index_, publish_rate_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn StereoCameraNode::on_activate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Activating...");

    pub_left_image_->on_activate();
    pub_right_image_->on_activate();
    pub_left_info_->on_activate();
    pub_right_info_->on_activate();
    pub_points_->on_activate();

    publish_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_),
        [this]() { publish_frame(); });

    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn StereoCameraNode::on_deactivate(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    publish_timer_.reset();
    pub_left_image_->on_deactivate();
    pub_right_image_->on_deactivate();
    pub_left_info_->on_deactivate();
    pub_right_info_->on_deactivate();
    pub_points_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

CallbackReturn StereoCameraNode::on_cleanup(const rclcpp_lifecycle::State&) {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    if (camera_open_) {
        close_camera();
        camera_open_ = false;
    }
    pub_left_image_.reset();  pub_right_image_.reset();
    pub_left_info_.reset();   pub_right_info_.reset();
    pub_points_.reset();
    cinfo_left_.reset();      cinfo_right_.reset();
    return CallbackReturn::SUCCESS;
}

CallbackReturn StereoCameraNode::on_shutdown(const rclcpp_lifecycle::State&) {
    if (camera_open_) { close_camera(); camera_open_ = false; }
    return CallbackReturn::SUCCESS;
}

CallbackReturn StereoCameraNode::on_error(const rclcpp_lifecycle::State&) {
    RCLCPP_ERROR(get_logger(), "Error state entered");
    if (camera_open_) { close_camera(); camera_open_ = false; }
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Publish loop
// ---------------------------------------------------------------------------

void StereoCameraNode::publish_frame() {
    sensor_msgs::msg::Image left_img, right_img;
    sensor_msgs::msg::PointCloud2 cloud;

    if (!grab_frame(left_img, right_img, cloud)) return;

    const auto now = this->now();
    left_img.header.stamp  = now;
    left_img.header.frame_id = frame_id_left_;
    right_img.header.stamp = now;
    right_img.header.frame_id = frame_id_right_;
    cloud.header.stamp     = now;
    cloud.header.frame_id  = frame_id_left_;  // cloud in left camera frame

    auto left_info  = cinfo_left_->getCameraInfo();
    auto right_info = cinfo_right_->getCameraInfo();
    left_info.header  = left_img.header;
    right_info.header = right_img.header;

    pub_left_image_->publish(left_img);
    pub_right_image_->publish(right_img);
    pub_left_info_->publish(left_info);
    pub_right_info_->publish(right_info);
    pub_points_->publish(cloud);
}

// ---------------------------------------------------------------------------
// Default (synthetic) implementations — override for real hardware
// ---------------------------------------------------------------------------

bool StereoCameraNode::open_camera() {
    RCLCPP_INFO(get_logger(), "Synthetic camera — override open_camera() for real hardware");
    return true;
}

void StereoCameraNode::close_camera() {}

bool StereoCameraNode::grab_frame(
    sensor_msgs::msg::Image& left_image,
    sensor_msgs::msg::Image& right_image,
    sensor_msgs::msg::PointCloud2& cloud)
{
    // Produce a minimal valid Image (1×1 mono8) so the topic exists
    constexpr uint32_t W = 1, H = 1;
    auto fill = [&](sensor_msgs::msg::Image& img) {
        img.width    = W;
        img.height   = H;
        img.encoding = "mono8";
        img.step     = W;
        img.data.assign(W * H, 128);
    };
    fill(left_image);
    fill(right_image);

    // Empty point cloud
    cloud.height = 1;
    cloud.width  = 0;
    cloud.is_dense = true;

    return true;
}

} // namespace cynlr::camera
