#pragma once
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "cynlr_arm_interfaces/msg/fault_event.hpp"
#include "cynlr_arm_interfaces/msg/operational_status.hpp"
#include "cynlr_arm_interfaces/srv/connect.hpp"
#include "cynlr_arm_interfaces/srv/trigger.hpp"
#include "cynlr_arm_interfaces/srv/set_tool.hpp"
#include "cynlr_arm_interfaces/srv/set_digital_outputs.hpp"
#include "cynlr_arm_interfaces/srv/get_digital_inputs.hpp"

#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/types.hpp"
#include "cynlr_arm_service/mode_manager.hpp"

namespace cynlr::arm_service {

// Supervision node — fault monitoring and NRT arm configuration only.
//
// Motion control (RT streaming, JTC, NRT move_l/j/ptp) is owned by ros2_control:
//   - CynlrHardwareInterface       owns the 1kHz RT loop
//   - CynlrNrtPassthroughController hosts MoveL/MoveJ/MovePTP action servers
//   - CynlrStateBroadcaster         publishes ArmState at configurable rate
//
// This node holds a separate NRT ArmInterface connection used exclusively for
// NRT configuration commands (tool, FT zero, GPIO, fault recovery). The
// Flexiv SDK supports concurrent connections for NRT operations.

class ArmServiceNode : public rclcpp_lifecycle::LifecycleNode {
public:
    explicit ArmServiceNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions{});
    ~ArmServiceNode() override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State& state) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_error(const rclcpp_lifecycle::State& state) override;

private:
    using FaultPub  = rclcpp_lifecycle::LifecyclePublisher<cynlr_arm_interfaces::msg::FaultEvent>;
    using StatusPub = rclcpp_lifecycle::LifecyclePublisher<cynlr_arm_interfaces::msg::OperationalStatus>;

    using ConnectSrv       = cynlr_arm_interfaces::srv::Connect;
    using TriggerSrv       = cynlr_arm_interfaces::srv::Trigger;
    using SetToolSrv       = cynlr_arm_interfaces::srv::SetTool;
    using SetDigitalOutSrv = cynlr_arm_interfaces::srv::SetDigitalOutputs;
    using GetDigitalInSrv  = cynlr_arm_interfaces::srv::GetDigitalInputs;

    void declare_parameters();
    void publish_status();
    void handle_fault();

    void handle_connect(
        const std::shared_ptr<ConnectSrv::Request>,
        std::shared_ptr<ConnectSrv::Response>);
    void handle_disconnect(
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_enable(
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_stop(
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_clear_fault(
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_set_tool(
        const std::shared_ptr<SetToolSrv::Request>,
        std::shared_ptr<SetToolSrv::Response>);
    void handle_zero_ft(
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_auto_recovery(
        const std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_set_digital_outputs(
        const std::shared_ptr<SetDigitalOutSrv::Request>,
        std::shared_ptr<SetDigitalOutSrv::Response>);
    void handle_get_digital_inputs(
        const std::shared_ptr<GetDigitalInSrv::Request>,
        std::shared_ptr<GetDigitalInSrv::Response>);

    std::unique_ptr<cynlr::arm::ArmInterface> arm_;
    std::unique_ptr<ModeManager>              mode_manager_;

    std::shared_ptr<FaultPub>  pub_fault_;
    std::shared_ptr<StatusPub> pub_status_;

    rclcpp::Service<ConnectSrv>::SharedPtr       srv_connect_;
    rclcpp::Service<TriggerSrv>::SharedPtr       srv_disconnect_;
    rclcpp::Service<TriggerSrv>::SharedPtr       srv_enable_;
    rclcpp::Service<TriggerSrv>::SharedPtr       srv_stop_;
    rclcpp::Service<TriggerSrv>::SharedPtr       srv_clear_fault_;
    rclcpp::Service<SetToolSrv>::SharedPtr       srv_set_tool_;
    rclcpp::Service<TriggerSrv>::SharedPtr       srv_zero_ft_;
    rclcpp::Service<TriggerSrv>::SharedPtr       srv_auto_recovery_;
    rclcpp::Service<SetDigitalOutSrv>::SharedPtr srv_set_digital_out_;
    rclcpp::Service<GetDigitalInSrv>::SharedPtr  srv_get_digital_in_;

    rclcpp::TimerBase::SharedPtr fault_poll_timer_;
};

} // namespace cynlr::arm_service
