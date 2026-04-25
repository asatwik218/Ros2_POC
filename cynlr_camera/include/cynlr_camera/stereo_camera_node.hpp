#pragma once

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

namespace cynlr::camera {

// StereoCameraNode — lifecycle node for a single stereo pair.
//
// One instance per physical stereo camera. Managed by the system launch file.
// The actual camera SDK calls go in the protected virtual open/grab/close methods
// so vendor-specific subclasses can be swapped in without changing the ROS layer.
//
// Publishes (when active):
//   /{ns}/left/image_raw          sensor_msgs/Image
//   /{ns}/right/image_raw         sensor_msgs/Image
//   /{ns}/left/camera_info        sensor_msgs/CameraInfo
//   /{ns}/right/camera_info       sensor_msgs/CameraInfo
//   /{ns}/points                  sensor_msgs/PointCloud2  (at ~publish_rate Hz)
//
// Parameters:
//   camera_index    – integer index for multi-camera setups (default 0)
//   publish_rate    – Hz for image+cloud publishing (default 30.0)
//   frame_id_left   – TF frame for left camera (default "camera_{idx}_left_optical")
//   frame_id_right  – TF frame for right camera (default "camera_{idx}_right_optical")
//   calibration_url – camera_info_manager URL for intrinsics (default "")

class StereoCameraNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit StereoCameraNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~StereoCameraNode() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State&) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State&) override;

protected:
    // Override these in vendor-specific subclasses.
    // Default implementations produce synthetic (checkerboard / zero) data for testing.
    virtual bool open_camera();
    virtual void close_camera();

    // Fills left_image, right_image, cloud with latest data.
    // Called at publish_rate Hz from the active timer.
    // Returns false if the frame is not ready (skips this tick).
    virtual bool grab_frame(
        sensor_msgs::msg::Image& left_image,
        sensor_msgs::msg::Image& right_image,
        sensor_msgs::msg::PointCloud2& cloud);

private:
    void declare_parameters();
    void publish_frame();

    // Publishers (lifecycle-managed)
    using ImagePub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>;
    using InfoPub  = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::CameraInfo>;
    using CloudPub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>;

    std::shared_ptr<ImagePub> pub_left_image_;
    std::shared_ptr<ImagePub> pub_right_image_;
    std::shared_ptr<InfoPub>  pub_left_info_;
    std::shared_ptr<InfoPub>  pub_right_info_;
    std::shared_ptr<CloudPub> pub_points_;

    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_left_;
    std::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_right_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    int    camera_index_{0};
    double publish_rate_{30.0};
    std::string frame_id_left_;
    std::string frame_id_right_;
    bool   camera_open_{false};
};

} // namespace cynlr::camera
