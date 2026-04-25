#include "cynlr_nrt_passthrough_controller/cynlr_nrt_passthrough_controller.hpp"

#include <chrono>
#include <cstring>
#include <thread>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cynlr_arm_controllers {

namespace {

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

cynlr::arm::MotionParams params_from_goal(double vel_lin, double vel_joint,
                                          double acc_lin, double acc_joint)
{
    cynlr::arm::MotionParams p{};
    p.max_linear_vel  = vel_lin   > 0.0 ? vel_lin   : 0.1;
    p.max_joint_vel   = vel_joint > 0.0 ? vel_joint : 0.5;
    p.max_linear_acc  = acc_lin   > 0.0 ? acc_lin   : 1.0;
    p.max_joint_acc   = acc_joint > 0.0 ? acc_joint : 2.0;
    return p;
}

} // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

controller_interface::CallbackReturn CynlrNrtPassthroughController::on_init()
{
    auto_declare<std::string>("prefix", "");
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrNrtPassthroughController::on_configure(
    const rclcpp_lifecycle::State&)
{
    prefix_ = get_node()->get_parameter("prefix").as_string();

    if (prefix_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'prefix' must not be empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    auto node = get_node();

    act_move_l_ = rclcpp_action::create_server<MoveLAction>(
        node, prefix_ + "move_l",
        [this](auto uuid, auto goal) { return handle_move_l_goal(uuid, goal); },
        [this](auto handle) { return handle_move_l_cancel(handle); },
        [this](auto handle) { handle_move_l_accepted(handle); });

    act_move_j_ = rclcpp_action::create_server<MoveJAction>(
        node, prefix_ + "move_j",
        [this](auto uuid, auto goal) { return handle_move_j_goal(uuid, goal); },
        [this](auto handle) { return handle_move_j_cancel(handle); },
        [this](auto handle) { handle_move_j_accepted(handle); });

    act_move_ptp_ = rclcpp_action::create_server<MovePTPAction>(
        node, prefix_ + "move_ptp",
        [this](auto uuid, auto goal) { return handle_move_ptp_goal(uuid, goal); },
        [this](auto handle) { return handle_move_ptp_cancel(handle); },
        [this](auto handle) { handle_move_ptp_accepted(handle); });

    RCLCPP_INFO(node->get_logger(),
        "[%s] NrtPassthroughController configured", prefix_.c_str());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrNrtPassthroughController::on_activate(
    const rclcpp_lifecycle::State&)
{
    // Recover ArmInterface* from the state interface via bit_cast
    for (const auto& si : state_interfaces_) {
        if (si.get_name() == prefix_ + "cynlr_arm_ctrl/arm_interface_ptr") {
            arm_ptr_ = bit_cast<cynlr::arm::ArmInterface*>(si.get_value());
            break;
        }
    }
    if (!arm_ptr_) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[%s] arm_interface_ptr state interface not found", prefix_.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrNrtPassthroughController::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    arm_ptr_ = nullptr;
    return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface configuration — this controller claims no command interfaces.
// It accesses the arm directly through the recovered pointer.
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
CynlrNrtPassthroughController::command_interface_configuration() const
{
    return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
CynlrNrtPassthroughController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.push_back(prefix_ + "cynlr_arm_ctrl/arm_interface_ptr");
    return cfg;
}

// update() is a no-op — all work is done in detached threads spawned by the action servers
controller_interface::return_type CynlrNrtPassthroughController::update(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    return controller_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// MoveL
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CynlrNrtPassthroughController::handle_move_l_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const MoveLAction::Goal>)
{
    if (motion_running_.load()) {
        RCLCPP_WARN(get_node()->get_logger(), "[%s] Rejecting MoveL — motion already running",
            prefix_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CynlrNrtPassthroughController::handle_move_l_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>>)
{
    if (arm_ptr_) arm_ptr_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CynlrNrtPassthroughController::handle_move_l_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle)
{
    std::thread([this, handle]() { execute_move_l(handle); }).detach();
}

void CynlrNrtPassthroughController::execute_move_l(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle)
{
    motion_running_ = true;

    const auto& goal = *handle->get_goal();
    cynlr::arm::CartesianTarget target{};
    for (int i = 0; i < 7; ++i) target.pose[i] = goal.target_pose[i];

    auto params = params_from_goal(goal.max_linear_vel, 0.0, goal.max_linear_acc, 0.0);

    auto cmd_result = arm_ptr_->move_l(target, params);
    if (!cmd_result) {
        auto result = std::make_shared<MoveLAction::Result>();
        result->success = false;
        result->message = cmd_result.error().message;
        handle->abort(result);
        motion_running_ = false;
        return;
    }

    // Poll for completion and publish feedback
    auto feedback = std::make_shared<MoveLAction::Feedback>();
    while (rclcpp::ok()) {
        if (handle->is_canceling()) {
            auto result = std::make_shared<MoveLAction::Result>();
            result->success = false;
            result->message = "Cancelled";
            handle->canceled(result);
            motion_running_ = false;
            return;
        }

        auto st = arm_ptr_->get_state();
        if (st) {
            for (int i = 0; i < 7; ++i) feedback->current_pose[i] = st->tcp_pose[i];
            // Simple Euclidean distance for progress estimate
            double dx = st->tcp_pose[0] - goal.target_pose[0];
            double dy = st->tcp_pose[1] - goal.target_pose[1];
            double dz = st->tcp_pose[2] - goal.target_pose[2];
            feedback->distance_remaining = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        handle->publish_feedback(feedback);

        auto done = arm_ptr_->is_motion_complete();
        if (done && *done) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto result = std::make_shared<MoveLAction::Result>();
    result->success = true;
    auto st = arm_ptr_->get_state();
    if (st) for (int i = 0; i < 7; ++i) result->final_pose[i] = st->tcp_pose[i];
    handle->succeed(result);
    motion_running_ = false;
}

// ---------------------------------------------------------------------------
// MoveJ
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CynlrNrtPassthroughController::handle_move_j_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const MoveJAction::Goal>)
{
    if (motion_running_.load()) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CynlrNrtPassthroughController::handle_move_j_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>>)
{
    if (arm_ptr_) arm_ptr_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CynlrNrtPassthroughController::handle_move_j_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle)
{
    std::thread([this, handle]() { execute_move_j(handle); }).detach();
}

void CynlrNrtPassthroughController::execute_move_j(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle)
{
    motion_running_ = true;

    const auto& goal = *handle->get_goal();
    cynlr::arm::JointTarget target{};
    for (int i = 0; i < 7; ++i) target.positions[i] = goal.target_positions[i];

    auto params = params_from_goal(0.0, goal.max_joint_vel, 0.0, goal.max_joint_acc);

    auto cmd_result = arm_ptr_->move_j(target, params);
    if (!cmd_result) {
        auto result = std::make_shared<MoveJAction::Result>();
        result->success = false;
        result->message = cmd_result.error().message;
        handle->abort(result);
        motion_running_ = false;
        return;
    }

    auto feedback = std::make_shared<MoveJAction::Feedback>();
    while (rclcpp::ok()) {
        if (handle->is_canceling()) {
            auto result = std::make_shared<MoveJAction::Result>();
            result->success = false;
            result->message = "Cancelled";
            handle->canceled(result);
            motion_running_ = false;
            return;
        }

        auto st = arm_ptr_->get_state();
        if (st) {
            for (int i = 0; i < 7; ++i) {
                feedback->current_positions[i] = st->joint_positions[i];
                feedback->position_error[i] = goal.target_positions[i] - st->joint_positions[i];
            }
        }
        handle->publish_feedback(feedback);

        auto done = arm_ptr_->is_motion_complete();
        if (done && *done) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto result = std::make_shared<MoveJAction::Result>();
    result->success = true;
    auto st = arm_ptr_->get_state();
    if (st) for (int i = 0; i < 7; ++i) result->final_positions[i] = st->joint_positions[i];
    handle->succeed(result);
    motion_running_ = false;
}

// ---------------------------------------------------------------------------
// MovePTP
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CynlrNrtPassthroughController::handle_move_ptp_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const MovePTPAction::Goal>)
{
    if (motion_running_.load()) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CynlrNrtPassthroughController::handle_move_ptp_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>>)
{
    if (arm_ptr_) arm_ptr_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CynlrNrtPassthroughController::handle_move_ptp_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle)
{
    std::thread([this, handle]() { execute_move_ptp(handle); }).detach();
}

void CynlrNrtPassthroughController::execute_move_ptp(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle)
{
    motion_running_ = true;

    const auto& goal = *handle->get_goal();
    cynlr::arm::CartesianTarget target{};
    for (int i = 0; i < 7; ++i) target.pose[i] = goal.target_pose[i];

    auto params = params_from_goal(0.0, goal.max_joint_vel, 0.0, goal.max_joint_acc);

    auto cmd_result = arm_ptr_->move_ptp(target, params);
    if (!cmd_result) {
        auto result = std::make_shared<MovePTPAction::Result>();
        result->success = false;
        result->message = cmd_result.error().message;
        handle->abort(result);
        motion_running_ = false;
        return;
    }

    auto feedback = std::make_shared<MovePTPAction::Feedback>();
    while (rclcpp::ok()) {
        if (handle->is_canceling()) {
            auto result = std::make_shared<MovePTPAction::Result>();
            result->success = false;
            result->message = "Cancelled";
            handle->canceled(result);
            motion_running_ = false;
            return;
        }

        auto st = arm_ptr_->get_state();
        if (st) {
            for (int i = 0; i < 7; ++i) feedback->current_pose[i] = st->tcp_pose[i];
            double dx = st->tcp_pose[0] - goal.target_pose[0];
            double dy = st->tcp_pose[1] - goal.target_pose[1];
            double dz = st->tcp_pose[2] - goal.target_pose[2];
            feedback->distance_remaining = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        handle->publish_feedback(feedback);

        auto done = arm_ptr_->is_motion_complete();
        if (done && *done) break;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto result = std::make_shared<MovePTPAction::Result>();
    result->success = true;
    auto st = arm_ptr_->get_state();
    if (st) for (int i = 0; i < 7; ++i) result->final_pose[i] = st->tcp_pose[i];
    handle->succeed(result);
    motion_running_ = false;
}

} // namespace cynlr_arm_controllers

PLUGINLIB_EXPORT_CLASS(
    cynlr_arm_controllers::CynlrNrtPassthroughController,
    controller_interface::ControllerInterface)
