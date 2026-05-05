#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <controller_manager/controller_manager.hpp>

#include "cynlr_arm_node/cynlr_arm_node.hpp"

// Container executable: ControllerManager + one CynlrArmNode per arm.
// All share the same process so they share the CynlrArmRegistry singleton.
//
// Arm prefixes are read from the 'arm_prefixes' ROS parameter on controller_manager,
// set by the launch file via the generated controller YAML.
//
// Architecture mirrors ros2_control_node (Jazzy):
//   cm_executor (main thread)  — handles all ROS callbacks (services, topics)
//   cm_update_thread           — calls cm->read/update/write at the configured rate
//   arm_thread                 — spins arm node executor (publishers, services, actions)
//
// Keeping cm_update_thread completely separate from cm_executor is essential:
// switch_controller service blocks waiting for update() to process the switch;
// if both ran on the same executor, the service callback would starve the update.

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // CM executor handles ROS callbacks; the update loop runs in its own thread
    auto cm_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto cm = std::make_shared<controller_manager::ControllerManager>(
        cm_executor, "controller_manager");
    cm_executor->add_node(cm);

    // Read arm prefixes from the controller_manager parameter (set by launch file).
    // Using a ROS parameter avoids fragile positional-arg parsing across launch backends.
    if (!cm->has_parameter("arm_prefixes")) {
        cm->declare_parameter("arm_prefixes", std::vector<std::string>{});
    }
    const auto prefixes = cm->get_parameter("arm_prefixes").as_string_array();
    RCLCPP_INFO(cm->get_logger(), "cynlr_main: arm_prefixes = [%s]",
        [&]{ std::string s; for (auto& p : prefixes) s += p + " "; return s; }().c_str());

    // Dedicated RT-style update thread — equivalent to ros2_control_node's cm_thread
    std::thread cm_update_thread([cm]() {
        cm->get_clock()->wait_until_started();

        const auto period =
            std::chrono::nanoseconds(1'000'000'000 / cm->get_update_rate());

        auto next_iter = std::chrono::steady_clock::now() + period;
        rclcpp::Time prev_time = cm->get_trigger_clock()->now();

        while (rclcpp::ok()) {
            const rclcpp::Time now      = cm->get_trigger_clock()->now();
            const rclcpp::Duration dt   = now - prev_time;
            prev_time = now;

            cm->read(now, dt);
            cm->update(now, dt);
            cm->write(now, dt);

            std::this_thread::sleep_until(next_iter);
            next_iter += period;
        }
    });

    // Arm nodes spin in their own executor — isolated from CM to avoid callback interference.
    // Keep shared_ptrs in arm_nodes so the executor's weak_ptr references stay valid.
    auto arm_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    std::vector<std::shared_ptr<cynlr_arm_node::CynlrArmNode>> arm_nodes;
    for (const auto& prefix : prefixes) {
        try {
            auto node = std::make_shared<cynlr_arm_node::CynlrArmNode>(prefix);
            arm_nodes.push_back(node);
            arm_executor->add_node(node);
            RCLCPP_INFO(cm->get_logger(), "cynlr_main: created arm node [%s]",
                node->get_name());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(cm->get_logger(),
                "cynlr_main: failed to create CynlrArmNode for [%s]: %s",
                prefix.c_str(), e.what());
        }
    }
    std::thread arm_thread([&arm_executor]() { arm_executor->spin(); });

    cm_executor->spin();  // blocks until shutdown

    arm_executor->cancel();
    arm_thread.join();
    cm_update_thread.join();

    rclcpp::shutdown();
    return 0;
}
