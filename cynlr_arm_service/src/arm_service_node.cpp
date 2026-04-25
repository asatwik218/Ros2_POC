#include "cynlr_arm_service/arm_service_node.hpp"

#include <functional>
#include <spdlog/spdlog.h>
#include "cynlr_arm_core/arm_factory.hpp"
#include "cynlr_arm_core/capabilities/digital_io_controllable.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace cynlr::arm_service {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ArmServiceNode::ArmServiceNode(const rclcpp::NodeOptions& options)
    : rclcpp_lifecycle::LifecycleNode("arm_service_node", options)
{
    declare_parameters();
    mode_manager_ = std::make_unique<ModeManager>();
    mode_manager_->set_callback([this](ArmMode old_mode, ArmMode new_mode) {
        spdlog::info("ArmServiceNode: mode {} → {}",
            mode_manager_->mode_name(old_mode),
            mode_manager_->mode_name(new_mode));
    });
}

ArmServiceNode::~ArmServiceNode() = default;

void ArmServiceNode::declare_parameters() {
    this->declare_parameter("vendor",        "sim");
    this->declare_parameter("serial_number", "");
    this->declare_parameter("ip_address",    "");
    this->declare_parameter("num_joints",    7);
}

// ---------------------------------------------------------------------------
// Lifecycle: configure
// ---------------------------------------------------------------------------

CallbackReturn ArmServiceNode::on_configure(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Configuring...");

    cynlr::arm::ArmConfig config;
    config.vendor        = this->get_parameter("vendor").as_string();
    config.serial_number = this->get_parameter("serial_number").as_string();
    config.ip_address    = this->get_parameter("ip_address").as_string();
    config.num_joints    = static_cast<int>(this->get_parameter("num_joints").as_int());

    arm_ = cynlr::arm::create_arm(config);
    if (!arm_) {
        RCLCPP_ERROR(get_logger(), "Unknown vendor: '%s'", config.vendor.c_str());
        return CallbackReturn::FAILURE;
    }

    pub_fault_ = this->create_publisher<cynlr_arm_interfaces::msg::FaultEvent>(
        "fault", rclcpp::QoS(10).reliable().transient_local());

    pub_status_ = this->create_publisher<cynlr_arm_interfaces::msg::OperationalStatus>(
        "status", rclcpp::QoS(10).reliable().transient_local());

    srv_connect_ = this->create_service<ConnectSrv>("connect",
        std::bind(&ArmServiceNode::handle_connect, this, _1, _2));

    srv_disconnect_ = this->create_service<TriggerSrv>("disconnect",
        std::bind(&ArmServiceNode::handle_disconnect, this, _1, _2));

    srv_enable_ = this->create_service<TriggerSrv>("enable",
        std::bind(&ArmServiceNode::handle_enable, this, _1, _2));

    srv_stop_ = this->create_service<TriggerSrv>("stop",
        std::bind(&ArmServiceNode::handle_stop, this, _1, _2));

    srv_clear_fault_ = this->create_service<TriggerSrv>("clear_fault",
        std::bind(&ArmServiceNode::handle_clear_fault, this, _1, _2));

    srv_set_tool_ = this->create_service<SetToolSrv>("set_tool",
        std::bind(&ArmServiceNode::handle_set_tool, this, _1, _2));

    srv_zero_ft_ = this->create_service<TriggerSrv>("zero_ft_sensor",
        std::bind(&ArmServiceNode::handle_zero_ft, this, _1, _2));

    srv_auto_recovery_ = this->create_service<TriggerSrv>("auto_recovery",
        std::bind(&ArmServiceNode::handle_auto_recovery, this, _1, _2));

    srv_set_digital_out_ = this->create_service<SetDigitalOutSrv>("set_digital_outputs",
        std::bind(&ArmServiceNode::handle_set_digital_outputs, this, _1, _2));

    srv_get_digital_in_ = this->create_service<GetDigitalInSrv>("get_digital_inputs",
        std::bind(&ArmServiceNode::handle_get_digital_inputs, this, _1, _2));

    RCLCPP_INFO(get_logger(), "Configured (vendor=%s)", config.vendor.c_str());
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: activate / deactivate
// ---------------------------------------------------------------------------

CallbackReturn ArmServiceNode::on_activate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Activating...");
    pub_fault_->on_activate();
    pub_status_->on_activate();

    fault_poll_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        [this]() {
            publish_status();
            if (arm_ && arm_->has_fault()) handle_fault();
        });

    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmServiceNode::on_deactivate(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Deactivating...");
    fault_poll_timer_.reset();
    pub_fault_->on_deactivate();
    pub_status_->on_deactivate();
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Lifecycle: cleanup / shutdown / error
// ---------------------------------------------------------------------------

CallbackReturn ArmServiceNode::on_cleanup(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Cleaning up...");
    if (arm_ && arm_->is_connected()) {
        (void)arm_->stop();
        (void)arm_->disconnect();
    }
    arm_.reset();

    srv_connect_.reset();      srv_disconnect_.reset();
    srv_enable_.reset();       srv_stop_.reset();
    srv_clear_fault_.reset();  srv_set_tool_.reset();
    srv_zero_ft_.reset();      srv_auto_recovery_.reset();
    srv_set_digital_out_.reset(); srv_get_digital_in_.reset();
    pub_fault_.reset();        pub_status_.reset();

    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmServiceNode::on_shutdown(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_INFO(get_logger(), "Shutting down...");
    if (arm_ && arm_->is_connected()) {
        (void)arm_->stop();
        (void)arm_->disconnect();
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn ArmServiceNode::on_error(const rclcpp_lifecycle::State& /*state*/) {
    RCLCPP_ERROR(get_logger(), "Error state entered");
    return CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

void ArmServiceNode::publish_status() {
    cynlr_arm_interfaces::msg::OperationalStatus msg{};
    msg.header.stamp = this->now();
    ArmMode mode     = mode_manager_->current();
    msg.mode         = static_cast<uint8_t>(mode);
    if (arm_) {
        msg.connected   = arm_->is_connected();
        msg.operational = (mode == ArmMode::ENABLED);
    }
    pub_status_->publish(msg);
}

void ArmServiceNode::handle_fault() {
    if (mode_manager_->current() == ArmMode::FAULT) return;
    mode_manager_->transition(ArmMode::FAULT);

    cynlr_arm_interfaces::msg::FaultEvent msg{};
    msg.header.stamp = this->now();
    msg.fault        = true;
    msg.clearable    = true;
    msg.message      = "Arm fault detected";
    pub_fault_->publish(msg);

    RCLCPP_ERROR(get_logger(), "Arm entered FAULT state");
}

// ---------------------------------------------------------------------------
// Service handlers
// ---------------------------------------------------------------------------

void ArmServiceNode::handle_connect(
    const std::shared_ptr<ConnectSrv::Request> req,
    std::shared_ptr<ConnectSrv::Response> res)
{
    if (!mode_manager_->can_transition(ArmMode::CONNECTED)) {
        res->success = false;
        res->message = "Cannot connect in mode: " + mode_manager_->mode_name(mode_manager_->current());
        return;
    }

    cynlr::arm::ArmConfig cfg;
    cfg.vendor        = req->vendor.empty()        ? this->get_parameter("vendor").as_string()        : req->vendor;
    cfg.serial_number = req->serial_number.empty() ? this->get_parameter("serial_number").as_string() : req->serial_number;
    cfg.ip_address    = req->ip_address.empty()    ? this->get_parameter("ip_address").as_string()    : req->ip_address;
    cfg.num_joints    = req->num_joints > 0        ? req->num_joints : static_cast<int>(this->get_parameter("num_joints").as_int());

    auto result = arm_->connect(cfg);
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    mode_manager_->transition(ArmMode::CONNECTED);
    res->success = true;
    res->message = "Connected";
}

void ArmServiceNode::handle_disconnect(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    auto result = arm_->disconnect();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    mode_manager_->transition(ArmMode::IDLE);
    res->success = true;
    res->message = "Disconnected";
}

void ArmServiceNode::handle_enable(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    if (!mode_manager_->can_transition(ArmMode::ENABLED)) {
        res->success = false;
        res->message = "Cannot enable in mode: " + mode_manager_->mode_name(mode_manager_->current());
        return;
    }
    auto result = arm_->enable();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    mode_manager_->transition(ArmMode::ENABLED);
    res->success = true;
    res->message = "Enabled";
}

void ArmServiceNode::handle_stop(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    auto result = arm_->stop();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "Stopped";
}

void ArmServiceNode::handle_clear_fault(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    if (mode_manager_->current() != ArmMode::FAULT) {
        res->success = false;
        res->message = "Not in FAULT mode";
        return;
    }
    auto result = arm_->clear_fault();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    mode_manager_->transition(ArmMode::CONNECTED);
    res->success = true;
    res->message = "Fault cleared — call enable to resume";
}

void ArmServiceNode::handle_set_tool(
    const std::shared_ptr<SetToolSrv::Request> req,
    std::shared_ptr<SetToolSrv::Response> res)
{
    cynlr::arm::ToolInfo tool{};
    tool.mass_kg = req->mass_kg;
    std::copy(req->com.begin(),      req->com.end(),      tool.com.begin());
    std::copy(req->inertia.begin(),  req->inertia.end(),  tool.inertia.begin());
    std::copy(req->tcp_pose.begin(), req->tcp_pose.end(), tool.tcp_pose.begin());

    auto result = arm_->set_tool(tool);
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "Tool set";
}

void ArmServiceNode::handle_zero_ft(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    auto result = arm_->zero_ft_sensor();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "FT sensor zeroed";
}

void ArmServiceNode::handle_auto_recovery(
    const std::shared_ptr<TriggerSrv::Request> /*req*/,
    std::shared_ptr<TriggerSrv::Response> res)
{
    auto result = arm_->run_auto_recovery();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    mode_manager_->transition(ArmMode::CONNECTED);
    res->success = true;
    res->message = "Auto recovery complete — call enable to resume";
}

void ArmServiceNode::handle_set_digital_outputs(
    const std::shared_ptr<SetDigitalOutSrv::Request> req,
    std::shared_ptr<SetDigitalOutSrv::Response> res)
{
    auto* dio = dynamic_cast<cynlr::arm::DigitalIOControllable*>(arm_.get());
    if (!dio) {
        res->success = false;
        res->message = "Arm does not support digital I/O";
        return;
    }
    if (req->indices.size() != req->values.size()) {
        res->success = false;
        res->message = "indices and values size mismatch";
        return;
    }
    std::vector<std::pair<int, bool>> outputs;
    outputs.reserve(req->indices.size());
    for (size_t i = 0; i < req->indices.size(); ++i)
        outputs.emplace_back(static_cast<int>(req->indices[i]), req->values[i]);

    auto result = dio->set_digital_outputs(outputs);
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->message = "Digital outputs set";
}

void ArmServiceNode::handle_get_digital_inputs(
    const std::shared_ptr<GetDigitalInSrv::Request> /*req*/,
    std::shared_ptr<GetDigitalInSrv::Response> res)
{
    auto* dio = dynamic_cast<cynlr::arm::DigitalIOControllable*>(arm_.get());
    if (!dio) {
        res->success = false;
        res->message = "Arm does not support digital I/O";
        return;
    }
    auto result = dio->get_digital_inputs();
    if (!result) {
        res->success = false;
        res->message = result.error().message;
        return;
    }
    res->success = true;
    res->values  = *result;
}

} // namespace cynlr::arm_service
