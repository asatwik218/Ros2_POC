#pragma once

#include <array>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cynlr_arm_interfaces/msg/cartesian_command.hpp"

namespace cynlr_arm_controllers {

/**
 * Escape hatch #3 — Cartesian command controller.
 *
 * Claims the prefix+"cartesian_cmd" GPIO command interfaces (13 doubles:
 * 7 pose + 6 wrench) and writes CartesianCommand messages from a ROS topic
 * directly to them at 1kHz.
 *
 * The hardware plugin's write() picks these up and calls:
 *   arm_->stream_command({CARTESIAN_MOTION_FORCE, pose, wrench})
 *
 * Use for: force-controlled insertion, visual servoing, Cartesian learning policies.
 *
 * Subscribe topic: /{prefix}cartesian_cmd  (cynlr_arm_interfaces/msg/CartesianCommand)
 */
class CynlrCartesianController : public controller_interface::ControllerInterface
{
public:
    controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;
    controller_interface::InterfaceConfiguration state_interface_configuration()
        const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State& previous_state) override;

private:
    static constexpr int kCartSize = 13; // 7 pose + 6 wrench

    std::string prefix_;

    realtime_tools::RealtimeBuffer<cynlr_arm_interfaces::msg::CartesianCommand> cmd_buf_;

    rclcpp::Subscription<cynlr_arm_interfaces::msg::CartesianCommand>::SharedPtr sub_;
};

} // namespace cynlr_arm_controllers
