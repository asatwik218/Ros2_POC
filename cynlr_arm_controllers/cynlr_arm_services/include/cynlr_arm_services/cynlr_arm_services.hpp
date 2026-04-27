#pragma once

#include <cstring>
#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/types.hpp"
#include "cynlr_arm_interfaces/srv/trigger.hpp"
#include "cynlr_arm_interfaces/srv/set_tool.hpp"

namespace cynlr_arm_controllers {

/**
 * Non-RT arm services controller.
 *
 * Hosts ROS2 services for operations that are too slow or non-deterministic
 * for the 500Hz RT loop: clear_fault, set_tool, zero_ft_sensor.
 *
 * Recovers the raw ArmInterface* from the hardware interface via the same
 * bit_cast pointer trick as CynlrNrtPassthroughController. Claims no command
 * interfaces and has a no-op update() — it is a broadcaster-style plugin that
 * exists purely to host service servers on the controller_manager's executor.
 */
class CynlrArmServices : public controller_interface::ControllerInterface
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
    using TriggerSrv = cynlr_arm_interfaces::srv::Trigger;
    using SetToolSrv = cynlr_arm_interfaces::srv::SetTool;

    std::string prefix_;

    // Raw pointer to ArmInterface, recovered via bit_cast from the state interface
    cynlr::arm::ArmInterface* arm_ptr_{nullptr};

    rclcpp::Service<TriggerSrv>::SharedPtr srv_clear_fault_;
    rclcpp::Service<SetToolSrv>::SharedPtr srv_set_tool_;
    rclcpp::Service<TriggerSrv>::SharedPtr srv_zero_ft_sensor_;

    void handle_clear_fault(
        const std::shared_ptr<TriggerSrv::Request> req,
        std::shared_ptr<TriggerSrv::Response> res);
    void handle_set_tool(
        const std::shared_ptr<SetToolSrv::Request> req,
        std::shared_ptr<SetToolSrv::Response> res);
    void handle_zero_ft_sensor(
        const std::shared_ptr<TriggerSrv::Request> req,
        std::shared_ptr<TriggerSrv::Response> res);
};

} // namespace cynlr_arm_controllers
