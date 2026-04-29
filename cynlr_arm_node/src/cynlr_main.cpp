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
// Usage: cynlr_main <prefix1> <prefix2> ... [--ros-args ...]
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
    // Collect positional args (arm prefixes) before ROS2 consumes argv
    std::vector<std::string> prefixes;
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--ros-args") break;
        prefixes.push_back(arg);
    }

    rclcpp::init(argc, argv);

    // CM executor handles ROS callbacks; the update loop runs in its own thread
    auto cm_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto cm = std::make_shared<controller_manager::ControllerManager>(
        cm_executor, "controller_manager");
    cm_executor->add_node(cm);

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

    // Arm nodes spin in their own executor — isolated from CM to avoid callback interference
    auto arm_executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    for (const auto& prefix : prefixes) {
        arm_executor->add_node(std::make_shared<cynlr_arm_node::CynlrArmNode>(prefix));
    }
    std::thread arm_thread([&arm_executor]() { arm_executor->spin(); });

    cm_executor->spin();  // blocks until shutdown

    arm_executor->cancel();
    arm_thread.join();
    cm_update_thread.join();

    rclcpp::shutdown();
    return 0;
}
