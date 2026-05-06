#include "cynlr_cartesian_controller/cynlr_cartesian_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cynlr_arm_controllers {

controller_interface::CallbackReturn CynlrCartesianController::on_init()
{
    auto_declare<std::string>("prefix", "");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrCartesianController::on_configure(
    const rclcpp_lifecycle::State&)
{
    prefix_ = get_node()->get_parameter("prefix").as_string();

    if (prefix_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'prefix' must not be empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    sub_ = get_node()->create_subscription<cynlr_arm_interfaces::msg::CartesianCommand>(
        prefix_ + "cartesian_cmd",
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](const cynlr_arm_interfaces::msg::CartesianCommand::SharedPtr msg) {
            cmd_buf_.writeFromNonRT(*msg);
        });

    RCLCPP_INFO(get_node()->get_logger(),
        "[%s] CartesianController configured. Listening on '%scartesian_cmd'",
        prefix_.c_str(), prefix_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrCartesianController::on_activate(
    const rclcpp_lifecycle::State&)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrCartesianController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    // NaN-fill command interfaces so the hardware plugin stops sending
    for (auto& ci : command_interfaces_)
        (void)ci.set_value(std::numeric_limits<double>::quiet_NaN());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CynlrCartesianController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i = 0; i < 7; ++i)
        cfg.names.push_back(prefix_ + "cartesian_cmd/pose_" + std::to_string(i));
    for (int i = 0; i < 6; ++i)
        cfg.names.push_back(prefix_ + "cartesian_cmd/wrench_" + std::to_string(i));
    return cfg;
}

controller_interface::InterfaceConfiguration
CynlrCartesianController::state_interface_configuration() const
{
    return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::return_type CynlrCartesianController::update(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    const auto* cmd = cmd_buf_.readFromRT();
    if (!cmd) return controller_interface::return_type::OK;

    // command_interfaces_ layout: [pose_0..pose_6, wrench_0..wrench_5]
    for (int i = 0; i < 7; ++i)
        (void)command_interfaces_[i].set_value(cmd->cartesian_pose[i]);
    for (int i = 0; i < 6; ++i)
        (void)command_interfaces_[7 + i].set_value(cmd->wrench[i]);

    return controller_interface::return_type::OK;
}

} // namespace cynlr_arm_controllers

PLUGINLIB_EXPORT_CLASS(
    cynlr_arm_controllers::CynlrCartesianController,
    controller_interface::ControllerInterface)
