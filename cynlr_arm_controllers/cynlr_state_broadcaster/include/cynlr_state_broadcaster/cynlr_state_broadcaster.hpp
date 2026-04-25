#pragma once

#include <memory>
#include <string>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "cynlr_arm_core/types.hpp"
#include "cynlr_arm_interfaces/msg/arm_state.hpp"

namespace cynlr_arm_controllers {

class CynlrStateBroadcaster : public controller_interface::ControllerInterface
{
public:
    // Claims no command interfaces — read-only broadcaster
    controller_interface::InterfaceConfiguration command_interface_configuration()
        const override;

    // Claims prefix+"cynlr_arm_state/full_state_ptr" to get the ArmState pointer,
    // plus individual FT/TCP/joint state interfaces for topic publishing
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
    // Retrieved from the "full_state_ptr" state interface via bit_cast on on_activate
    cynlr::arm::ArmState* arm_state_ptr_{nullptr};

    std::string prefix_;         // e.g. "arm_left_"
    double publish_rate_{100.0}; // Hz for ArmState; FT/TCP publish every update()

    using ArmStatePub   = realtime_tools::RealtimePublisher<cynlr_arm_interfaces::msg::ArmState>;
    using PosePub       = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>;
    using WrenchPub     = realtime_tools::RealtimePublisher<geometry_msgs::msg::WrenchStamped>;

    std::shared_ptr<ArmStatePub> pub_arm_state_;
    std::shared_ptr<PosePub>     pub_tcp_pose_;
    std::shared_ptr<WrenchPub>   pub_ft_raw_;
    std::shared_ptr<WrenchPub>   pub_ext_wrench_tcp_;
    std::shared_ptr<WrenchPub>   pub_ext_wrench_world_;

    // Decimation counter for lower-rate ArmState publishing
    int update_count_{0};
    int publish_every_{1}; // computed from publish_rate_ and controller update_rate
};

} // namespace cynlr_arm_controllers
