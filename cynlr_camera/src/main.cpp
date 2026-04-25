#include "rclcpp/rclcpp.hpp"
#include "cynlr_camera/stereo_camera_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cynlr::camera::StereoCameraNode>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
