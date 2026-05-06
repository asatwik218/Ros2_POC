#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#include "cynlr_arm_interfaces/msg/arm_state.hpp"
#include "cynlr_arm_interfaces/srv/trigger.hpp"
#include "cynlr_arm_interfaces/srv/set_tool.hpp"
#include "cynlr_arm_interfaces/srv/get_tool.hpp"
#include "cynlr_arm_interfaces/srv/is_motion_running.hpp"
#include "cynlr_arm_interfaces/action/move_l.hpp"
#include "cynlr_arm_interfaces/action/move_j.hpp"
#include "cynlr_arm_interfaces/action/move_ptp.hpp"

#include "cynlr_robot/cynlr_arm_handle.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr_arm_node {

// Plain rclcpp::Node per arm. Runs in the same process as ControllerManager
// (via cynlr_main) so it can access CynlrArmHandle from CynlrArmRegistry.
//
// On startup, polls the registry every 100ms until the hardware plugin activates
// and registers the handle, then sets up publishers / services / action servers.
class CynlrArmNode : public rclcpp::Node
{
public:
    explicit CynlrArmNode(const std::string& prefix);

private:
    using TriggerSrv         = cynlr_arm_interfaces::srv::Trigger;
    using SetToolSrv         = cynlr_arm_interfaces::srv::SetTool;
    using GetToolSrv         = cynlr_arm_interfaces::srv::GetTool;
    using IsMotionRunningSrv = cynlr_arm_interfaces::srv::IsMotionRunning;
    using MoveLAction   = cynlr_arm_interfaces::action::MoveL;
    using MoveJAction   = cynlr_arm_interfaces::action::MoveJ;
    using MovePTPAction = cynlr_arm_interfaces::action::MovePTP;

    std::string prefix_;
    std::shared_ptr<cynlr_robot::CynlrArmHandle> handle_;

    // Timers
    rclcpp::TimerBase::SharedPtr poll_timer_;    // polls registry until handle available
    rclcpp::TimerBase::SharedPtr publish_timer_; // 100 Hz state publishing

    // Publishers
    rclcpp::Publisher<cynlr_arm_interfaces::msg::ArmState>::SharedPtr pub_arm_state_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pub_tcp_pose_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr   pub_ft_raw_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr   pub_ext_wrench_tcp_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr   pub_ext_wrench_world_;

    // Services
    rclcpp::Service<TriggerSrv>::SharedPtr         srv_connect_;
    rclcpp::Service<TriggerSrv>::SharedPtr         srv_disconnect_;
    rclcpp::Service<TriggerSrv>::SharedPtr         srv_enable_;
    rclcpp::Service<TriggerSrv>::SharedPtr         srv_stop_;
    rclcpp::Service<TriggerSrv>::SharedPtr         srv_clear_fault_;
    rclcpp::Service<TriggerSrv>::SharedPtr         srv_zero_ft_sensor_;
    rclcpp::Service<SetToolSrv>::SharedPtr         srv_set_tool_;
    rclcpp::Service<SetToolSrv>::SharedPtr         srv_update_tool_;
    rclcpp::Service<GetToolSrv>::SharedPtr         srv_get_tool_;
    rclcpp::Service<IsMotionRunningSrv>::SharedPtr srv_is_motion_running_;

    // Action servers
    rclcpp_action::Server<MoveLAction>::SharedPtr   act_move_l_;
    rclcpp_action::Server<MoveJAction>::SharedPtr   act_move_j_;
    rclcpp_action::Server<MovePTPAction>::SharedPtr act_move_ptp_;

    std::atomic<bool> motion_running_{false};

    void try_setup();
    void publish_state();

    // Generic trigger handler: calls a handle function, fills success/message
    void handle_trigger(
        const std::function<cynlr::arm::Expected<void>()>& fn,
        const char* name,
        std::shared_ptr<TriggerSrv::Response> res);

    void handle_clear_fault(
        std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_zero_ft_sensor(
        std::shared_ptr<TriggerSrv::Request>,
        std::shared_ptr<TriggerSrv::Response>);
    void handle_set_tool(
        std::shared_ptr<SetToolSrv::Request>,
        std::shared_ptr<SetToolSrv::Response>);
    void handle_update_tool(
        std::shared_ptr<SetToolSrv::Request>,
        std::shared_ptr<SetToolSrv::Response>);
    void handle_get_tool(
        std::shared_ptr<GetToolSrv::Request>,
        std::shared_ptr<GetToolSrv::Response>);
    void handle_is_motion_running(
        std::shared_ptr<IsMotionRunningSrv::Request>,
        std::shared_ptr<IsMotionRunningSrv::Response>);

    // MoveL
    rclcpp_action::GoalResponse   move_l_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveLAction::Goal>);
    rclcpp_action::CancelResponse move_l_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>>);
    void move_l_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>>);
    void execute_move_l(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>>);

    // MoveJ
    rclcpp_action::GoalResponse   move_j_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveJAction::Goal>);
    rclcpp_action::CancelResponse move_j_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>>);
    void move_j_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>>);
    void execute_move_j(std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>>);

    // MovePTP
    rclcpp_action::GoalResponse   move_ptp_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const MovePTPAction::Goal>);
    rclcpp_action::CancelResponse move_ptp_cancel(std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>>);
    void move_ptp_accepted(std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>>);
    void execute_move_ptp(std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>>);
};

} // namespace cynlr_arm_node
