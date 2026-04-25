#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cynlr_arm_interfaces/msg/joint_command.hpp"

namespace cynlr_arm_controllers {

/**
 * Escape hatch #1 — Direct joint command controller.
 *
 * Claims position + velocity command interfaces for one arm and forwards
 * JointCommand messages from a ROS topic directly to them at 1kHz.
 *
 * Your custom planner or learning policy publishes to:
 *   /{prefix}joint_cmd_direct  (cynlr_arm_interfaces/msg/JointCommand)
 *
 * No trajectory interpolation, no MoveIt, no IK. Your code is responsible
 * for generating smooth, safe commands.
 *
 * Activate by switching controllers:
 *   ros2 control switch_controllers \
 *     --deactivate arm_left_jt_controller \
 *     --activate   arm_left_direct_cmd
 */
class CynlrDirectCommandController : public controller_interface::ControllerInterface
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
    static constexpr int kDOF = 7;

    std::string prefix_;
    std::vector<std::string> joint_names_; // set from parameters

    // Lock-free: subscriber (NRT) writes, update() (RT) reads
    realtime_tools::RealtimeBuffer<cynlr_arm_interfaces::msg::JointCommand> cmd_buf_;

    rclcpp::Subscription<cynlr_arm_interfaces::msg::JointCommand>::SharedPtr sub_;
};

} // namespace cynlr_arm_controllers
