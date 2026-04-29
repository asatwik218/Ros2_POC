#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

#include "cynlr_arm_node/cynlr_arm_node.hpp"

// Container executable that runs ControllerManager + one CynlrArmNode per arm
// in the same process so they share the CynlrArmRegistry singleton.
//
// Usage: cynlr_main <prefix1> <prefix2> ... [--ros-args ...]
//
// Arm prefixes (e.g. "arm_left_") are taken from positional arguments before
// "--ros-args". The launch file passes them via the `arguments` field.

int main(int argc, char** argv)
{
    // Collect positional args (arm prefixes) before ROS2 consumes argv
    std::vector<std::string> prefixes;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--ros-args") break;
        prefixes.push_back(arg);
    }

    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // ControllerManager reads robot_description and controller YAML from ROS parameters
    // passed by the launch file (parameters=[robot_description, ParameterFile(ctrl_yaml)])
    auto cm = std::make_shared<controller_manager::ControllerManager>(
        executor, "controller_manager");
    executor->add_node(cm);

    // One CynlrArmNode per arm — each polls CynlrArmRegistry until the hardware
    // plugin activates and registers its handle
    for (const auto& prefix : prefixes) {
        executor->add_node(std::make_shared<cynlr_arm_node::CynlrArmNode>(prefix));
    }

    executor->spin();
    rclcpp::shutdown();
    return 0;
}
