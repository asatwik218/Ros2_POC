#include "cynlr_arm_services/cynlr_arm_services.hpp"

#include <cstring>
#include <functional>

#include <pluginlib/class_list_macros.hpp>

namespace cynlr_arm_controllers {

namespace {

// Same bit_cast as the hardware interface — reinterprets memory without UB.
template<class To, class From>
std::enable_if_t<sizeof(To) == sizeof(From)
                     && std::is_trivially_copyable_v<From>
                     && std::is_trivially_copyable_v<To>,
    To>
bit_cast(const From& src) noexcept
{
    static_assert(std::is_trivially_constructible_v<To>);
    To dst;
    std::memcpy(&dst, &src, sizeof(To));
    return dst;
}

} // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

controller_interface::CallbackReturn CynlrArmServices::on_init()
{
    auto_declare<std::string>("prefix", "");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrArmServices::on_configure(
    const rclcpp_lifecycle::State&)
{
    prefix_ = get_node()->get_parameter("prefix").as_string();
    if (prefix_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'prefix' must not be empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    auto node = get_node();

    srv_clear_fault_ = node->create_service<TriggerSrv>(
        prefix_ + "clear_fault",
        [this](const std::shared_ptr<TriggerSrv::Request> req,
               std::shared_ptr<TriggerSrv::Response> res) {
            handle_clear_fault(req, res);
        });

    srv_set_tool_ = node->create_service<SetToolSrv>(
        prefix_ + "set_tool",
        [this](const std::shared_ptr<SetToolSrv::Request> req,
               std::shared_ptr<SetToolSrv::Response> res) {
            handle_set_tool(req, res);
        });

    srv_zero_ft_sensor_ = node->create_service<TriggerSrv>(
        prefix_ + "zero_ft_sensor",
        [this](const std::shared_ptr<TriggerSrv::Request> req,
               std::shared_ptr<TriggerSrv::Response> res) {
            handle_zero_ft_sensor(req, res);
        });

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrArmServices::on_activate(
    const rclcpp_lifecycle::State&)
{
    for (const auto& si : state_interfaces_) {
        if (si.get_name() == prefix_ + "cynlr_arm_ctrl/arm_interface_ptr") {
            double raw = si.get_value();
            arm_ptr_ = bit_cast<cynlr::arm::ArmInterface*>(raw);
            break;
        }
    }

    if (!arm_ptr_) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[%s] Could not find arm_interface_ptr state interface", prefix_.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrArmServices::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    arm_ptr_ = nullptr;
    return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
CynlrArmServices::command_interface_configuration() const
{
    return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
CynlrArmServices::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.push_back(prefix_ + "cynlr_arm_ctrl/arm_interface_ptr");
    return cfg;
}

// ---------------------------------------------------------------------------
// Update — no-op; this plugin only hosts service servers
// ---------------------------------------------------------------------------

controller_interface::return_type CynlrArmServices::update(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Service handlers
// ---------------------------------------------------------------------------

void CynlrArmServices::handle_clear_fault(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    if (!arm_ptr_) {
        res->success = false;
        res->message = "Arm not available — controller not active";
        return;
    }
    auto result = arm_ptr_->clear_fault();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "Fault cleared";
}

void CynlrArmServices::handle_set_tool(
    const std::shared_ptr<SetToolSrv::Request> req,
    std::shared_ptr<SetToolSrv::Response> res)
{
    if (!arm_ptr_) {
        res->success = false;
        res->message = "Arm not available — controller not active";
        return;
    }
    cynlr::arm::ToolInfo tool{};
    tool.mass_kg = req->mass_kg;
    std::copy(req->com.begin(),       req->com.end(),       tool.com.begin());
    std::copy(req->inertia.begin(),   req->inertia.end(),   tool.inertia.begin());
    std::copy(req->tcp_pose.begin(),  req->tcp_pose.end(),  tool.tcp_pose.begin());

    auto result = arm_ptr_->set_tool(tool);
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "Tool set";
}

void CynlrArmServices::handle_zero_ft_sensor(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    if (!arm_ptr_) {
        res->success = false;
        res->message = "Arm not available — controller not active";
        return;
    }
    auto result = arm_ptr_->zero_ft_sensor();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "FT sensor zeroed";
}

} // namespace cynlr_arm_controllers

PLUGINLIB_EXPORT_CLASS(
    cynlr_arm_controllers::CynlrArmServices,
    controller_interface::ControllerInterface)
