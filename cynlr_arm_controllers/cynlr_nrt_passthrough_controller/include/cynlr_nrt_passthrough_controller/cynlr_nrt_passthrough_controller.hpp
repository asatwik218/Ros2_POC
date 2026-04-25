#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/types.hpp"
#include "cynlr_arm_interfaces/action/move_l.hpp"
#include "cynlr_arm_interfaces/action/move_j.hpp"
#include "cynlr_arm_interfaces/action/move_ptp.hpp"

namespace cynlr_arm_controllers {

/**
 * Escape hatch #2 — NRT Passthrough Controller.
 *
 * Hosts MoveL, MoveJ, and MovePTP action servers that call the arm's own
 * built-in NRT trajectory planner directly, bypassing MoveIt.
 *
 * The arm pointer is recovered from the hardware interface via bit_cast
 * (same technique as CynlrStateBroadcaster for the ArmState).
 *
 * This controller claims NO command interfaces when idle — it co-exists
 * with other controllers. When a goal is received, it spawns a detached
 * thread that calls arm_->move_l/j/ptp(), polls is_motion_complete(),
 * and publishes feedback. The arm must NOT be in streaming mode while
 * an NRT move is running.
 */
class CynlrNrtPassthroughController : public controller_interface::ControllerInterface
{
public:
    using MoveLAction   = cynlr_arm_interfaces::action::MoveL;
    using MoveJAction   = cynlr_arm_interfaces::action::MoveJ;
    using MovePTPAction = cynlr_arm_interfaces::action::MovePTP;

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
    std::string prefix_;

    // Raw pointer to ArmInterface, recovered via bit_cast from the state interface
    cynlr::arm::ArmInterface* arm_ptr_{nullptr};

    // Tracks whether an NRT move is currently running (one at a time)
    std::atomic<bool> motion_running_{false};

    // Action servers
    rclcpp_action::Server<MoveLAction>::SharedPtr   act_move_l_;
    rclcpp_action::Server<MoveJAction>::SharedPtr   act_move_j_;
    rclcpp_action::Server<MovePTPAction>::SharedPtr act_move_ptp_;

    // --- MoveL handlers ---
    rclcpp_action::GoalResponse handle_move_l_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const MoveLAction::Goal> goal);
    rclcpp_action::CancelResponse handle_move_l_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle);
    void handle_move_l_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle);
    void execute_move_l(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle);

    // --- MoveJ handlers ---
    rclcpp_action::GoalResponse handle_move_j_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const MoveJAction::Goal> goal);
    rclcpp_action::CancelResponse handle_move_j_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle);
    void handle_move_j_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle);
    void execute_move_j(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle);

    // --- MovePTP handlers ---
    rclcpp_action::GoalResponse handle_move_ptp_goal(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const MovePTPAction::Goal> goal);
    rclcpp_action::CancelResponse handle_move_ptp_cancel(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle);
    void handle_move_ptp_accepted(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle);
    void execute_move_ptp(
        std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle);
};

} // namespace cynlr_arm_controllers
