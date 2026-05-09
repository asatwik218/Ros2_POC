#include "cynlr_direct_command_controller/cynlr_direct_command_controller.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace cynlr_arm_controllers {

controller_interface::CallbackReturn CynlrDirectCommandController::on_init()
{
    auto_declare<std::string>("prefix", "");
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>{});
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrDirectCommandController::on_configure(
    const rclcpp_lifecycle::State&)
{
    prefix_      = get_node()->get_parameter("prefix").as_string();
    joint_names_ = get_node()->get_parameter("joints").as_string_array();

    if (joint_names_.size() != kDOF) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[%s] Expected 7 joints in 'joints' parameter, got %zu",
            prefix_.c_str(), joint_names_.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    // Subscribe to direct joint commands with low-latency QoS
    sub_ = get_node()->create_subscription<cynlr_arm_interfaces::msg::JointCommand>(
        prefix_ + "joint_cmd_direct",
        rclcpp::QoS(1).best_effort().durability_volatile(),
        [this](const cynlr_arm_interfaces::msg::JointCommand::SharedPtr msg) {
            cmd_buf_.writeFromNonRT(*msg);
        });

    RCLCPP_INFO(get_node()->get_logger(),
        "[%s] DirectCommandController configured. Listening on '%sjoint_cmd_direct'",
        prefix_.c_str(), prefix_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrDirectCommandController::on_activate(
    const rclcpp_lifecycle::State&)
{
    // Initialise the buffer with a zero-velocity command at current positions
    // (command_interfaces_ are now valid — read initial positions from state_interfaces_)
    cynlr_arm_interfaces::msg::JointCommand init_cmd{};
    for (int i = 0; i < kDOF; ++i) {
        auto value = state_interfaces_[i].get_optional();

        if (!value.has_value())
        {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Failed to read state interface %d",
                i);

            return controller_interface::CallbackReturn::ERROR;
        }

        init_cmd.joint_position[i] = *value;
        init_cmd.joint_velocity[i] = 0.0;
    }
    cmd_buf_.writeFromNonRT(init_cmd);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrDirectCommandController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CynlrDirectCommandController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& j : joint_names_) {
        cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
        cfg.names.push_back(j + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return cfg;
}

controller_interface::InterfaceConfiguration
CynlrDirectCommandController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto& j : joint_names_)
        cfg.names.push_back(j + "/" + hardware_interface::HW_IF_POSITION);
    return cfg;
}

controller_interface::return_type CynlrDirectCommandController::update(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    const auto* cmd = cmd_buf_.readFromRT();
    if (!cmd) return controller_interface::return_type::OK;

    // command_interfaces_ layout: [pos_0, vel_0, pos_1, vel_1, ..., pos_6, vel_6]
    for (int i = 0; i < kDOF; ++i) {
        (void)command_interfaces_[i * 2].set_value(cmd->joint_position[i]);
        (void)command_interfaces_[i * 2 + 1].set_value(cmd->joint_velocity[i]);
    }

    return controller_interface::return_type::OK;
}

} // namespace cynlr_arm_controllers

PLUGINLIB_EXPORT_CLASS(
    cynlr_arm_controllers::CynlrDirectCommandController,
    controller_interface::ControllerInterface)
