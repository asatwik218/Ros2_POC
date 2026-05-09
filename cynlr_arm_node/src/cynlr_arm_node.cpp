#include "cynlr_arm_node/cynlr_arm_node.hpp"

#include <chrono>
#include <cmath>
#include <thread>

#include "cynlr_robot/cynlr_arm_registry.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr_arm_node {

namespace {

cynlr::arm::MotionParams motion_params(double vel_lin, double vel_joint,
                                       double acc_lin, double acc_joint)
{
    cynlr::arm::MotionParams p{};
    p.max_linear_vel  = vel_lin   > 0.0 ? vel_lin   : 0.1;
    p.max_joint_vel   = vel_joint > 0.0 ? vel_joint : 0.5;
    p.max_linear_acc  = acc_lin   > 0.0 ? acc_lin   : 1.0;
    p.max_joint_acc   = acc_joint > 0.0 ? acc_joint : 2.0;
    return p;
}

cynlr::arm::ToolInfo tool_from_request(const cynlr_arm_interfaces::srv::SetTool::Request& req)
{
    cynlr::arm::ToolInfo tool{};
    tool.mass_kg = req.mass_kg;
    std::copy(req.com.begin(),      req.com.end(),      tool.com.begin());
    std::copy(req.inertia.begin(),  req.inertia.end(),  tool.inertia.begin());
    std::copy(req.tcp_pose.begin(), req.tcp_pose.end(), tool.tcp_pose.begin());
    return tool;
}

} // namespace

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

CynlrArmNode::CynlrArmNode(const std::string& prefix)
    : rclcpp::Node(prefix.substr(0, prefix.size() - 1) + "_arm_ops")
    , prefix_(prefix)
{
    poll_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { try_setup(); });
}

// ---------------------------------------------------------------------------
// try_setup
// ---------------------------------------------------------------------------

void CynlrArmNode::try_setup()
{
    handle_ = cynlr_robot::CynlrArmRegistry::instance().get(prefix_);
    if (!handle_) return;

    poll_timer_->cancel();

    // Publishers
    pub_arm_state_ = create_publisher<cynlr_arm_interfaces::msg::ArmState>(
        prefix_ + "arm_state", rclcpp::SystemDefaultsQoS());
    pub_tcp_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>(
        prefix_ + "tcp_pose", rclcpp::SystemDefaultsQoS());
    pub_ft_raw_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
        prefix_ + "ft_raw", rclcpp::SystemDefaultsQoS());
    pub_ext_wrench_tcp_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
        prefix_ + "ext_wrench_tcp", rclcpp::SystemDefaultsQoS());
    pub_ext_wrench_world_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
        prefix_ + "ext_wrench_world", rclcpp::SystemDefaultsQoS());

    // Lifecycle services
    srv_connect_ = create_service<TriggerSrv>(
        prefix_ + "connect",
        [this](std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
            handle_trigger(handle_->connect, "connect", res); });

    srv_disconnect_ = create_service<TriggerSrv>(
        prefix_ + "disconnect",
        [this](std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
            handle_trigger(handle_->disconnect, "disconnect", res); });

    srv_enable_ = create_service<TriggerSrv>(
        prefix_ + "enable",
        [this](std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
            handle_trigger(handle_->enable, "enable", res); });

    srv_stop_ = create_service<TriggerSrv>(
        prefix_ + "stop",
        [this](std::shared_ptr<TriggerSrv::Request>, std::shared_ptr<TriggerSrv::Response> res) {
            handle_trigger(handle_->stop_motion, "stop", res); });

    srv_clear_fault_ = create_service<TriggerSrv>(
        prefix_ + "clear_fault",
        [this](std::shared_ptr<TriggerSrv::Request> req,
               std::shared_ptr<TriggerSrv::Response> res) { handle_clear_fault(req, res); });

    srv_zero_ft_sensor_ = create_service<TriggerSrv>(
        prefix_ + "zero_ft_sensor",
        [this](std::shared_ptr<TriggerSrv::Request> req,
               std::shared_ptr<TriggerSrv::Response> res) { handle_zero_ft_sensor(req, res); });

    // Tool services
    srv_set_tool_ = create_service<SetToolSrv>(
        prefix_ + "set_tool",
        [this](std::shared_ptr<SetToolSrv::Request> req,
               std::shared_ptr<SetToolSrv::Response> res) { handle_set_tool(req, res); });

    srv_update_tool_ = create_service<SetToolSrv>(
        prefix_ + "update_tool",
        [this](std::shared_ptr<SetToolSrv::Request> req,
               std::shared_ptr<SetToolSrv::Response> res) { handle_update_tool(req, res); });

    srv_get_tool_ = create_service<GetToolSrv>(
        prefix_ + "get_tool",
        [this](std::shared_ptr<GetToolSrv::Request> req,
               std::shared_ptr<GetToolSrv::Response> res) { handle_get_tool(req, res); });

    srv_is_motion_running_ = create_service<IsMotionRunningSrv>(
        prefix_ + "is_motion_running",
        [this](std::shared_ptr<IsMotionRunningSrv::Request> req,
               std::shared_ptr<IsMotionRunningSrv::Response> res) {
            handle_is_motion_running(req, res); });

    // Action servers
    act_move_l_ = rclcpp_action::create_server<MoveLAction>(
        this, prefix_ + "move_l",
        [this](auto u, auto g) { return move_l_goal(u, g); },
        [this](auto h) { return move_l_cancel(h); },
        [this](auto h) { move_l_accepted(h); });

    act_move_j_ = rclcpp_action::create_server<MoveJAction>(
        this, prefix_ + "move_j",
        [this](auto u, auto g) { return move_j_goal(u, g); },
        [this](auto h) { return move_j_cancel(h); },
        [this](auto h) { move_j_accepted(h); });

    act_move_ptp_ = rclcpp_action::create_server<MovePTPAction>(
        this, prefix_ + "move_ptp",
        [this](auto u, auto g) { return move_ptp_goal(u, g); },
        [this](auto h) { return move_ptp_cancel(h); },
        [this](auto h) { move_ptp_accepted(h); });

    // 100 Hz state publishing
    publish_timer_ = create_wall_timer(
        std::chrono::milliseconds(10),
        [this]() { publish_state(); });

    RCLCPP_INFO(get_logger(), "[%s] CynlrArmNode ready", prefix_.c_str());
}

// ---------------------------------------------------------------------------
// State publishing
// ---------------------------------------------------------------------------

void CynlrArmNode::publish_state()
{
    auto s_opt = handle_->get_state();
    if (!s_opt) return;
    const auto& s = *s_opt;

    auto now = get_clock()->now();

    // TCP pose
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = now;
        msg.header.frame_id = "world";
        msg.pose.position.x    = s.tcp_pose[0];
        msg.pose.position.y    = s.tcp_pose[1];
        msg.pose.position.z    = s.tcp_pose[2];
        msg.pose.orientation.w = s.tcp_pose[3];
        msg.pose.orientation.x = s.tcp_pose[4];
        msg.pose.orientation.y = s.tcp_pose[5];
        msg.pose.orientation.z = s.tcp_pose[6];
        pub_tcp_pose_->publish(msg);
    }

    auto publish_wrench = [&](auto& pub, const std::array<double,6>& data,
                               const std::string& frame) {
        geometry_msgs::msg::WrenchStamped msg;
        msg.header.stamp    = now;
        msg.header.frame_id = frame;
        msg.wrench.force.x  = data[0]; msg.wrench.force.y  = data[1];
        msg.wrench.force.z  = data[2]; msg.wrench.torque.x = data[3];
        msg.wrench.torque.y = data[4]; msg.wrench.torque.z = data[5];
        pub->publish(msg);
    };

    publish_wrench(pub_ft_raw_,          s.ft_sensor_raw,      prefix_ + "flange_link");
    publish_wrench(pub_ext_wrench_tcp_,  s.ext_wrench_in_tcp,  prefix_ + "flange_link");
    publish_wrench(pub_ext_wrench_world_,s.ext_wrench_in_world, "world");

    // Full ArmState
    {
        cynlr_arm_interfaces::msg::ArmState msg;
        msg.header.stamp = now;
        for (int i = 0; i < 7; ++i) {
            msg.joint_positions[i]        = s.joint_positions[i];
            msg.joint_velocities[i]       = s.joint_velocities[i];
            msg.joint_torques[i]          = s.joint_torques[i];
            msg.joint_torques_external[i] = s.joint_torques_external[i];
            msg.tcp_pose[i]               = s.tcp_pose[i];
        }
        for (int i = 0; i < 6; ++i) {
            msg.tcp_velocity[i]        = s.tcp_velocity[i];
            msg.ft_sensor_raw[i]       = s.ft_sensor_raw[i];
            msg.ext_wrench_in_tcp[i]   = s.ext_wrench_in_tcp[i];
            msg.ext_wrench_in_world[i] = s.ext_wrench_in_world[i];
        }
        msg.fault          = s.fault;
        msg.operational    = s.operational;
        msg.estopped       = s.estopped;
        msg.motion_running = motion_running_.load();
        pub_arm_state_->publish(msg);
    }
}

// ---------------------------------------------------------------------------
// Service helpers
// ---------------------------------------------------------------------------

void CynlrArmNode::handle_trigger(
    const std::function<cynlr::arm::Expected<void>()>& fn,
    const char* name,
    std::shared_ptr<TriggerSrv::Response> res)
{
    if (!fn) {
        res->success = false;
        res->message = std::string(name) + ": not supported by this arm";
        return;
    }
    auto r = fn();
    res->success = static_cast<bool>(r);
    res->message = r ? std::string(name) + " OK" : r.error().message;
}

void CynlrArmNode::handle_clear_fault(
    std::shared_ptr<TriggerSrv::Request>,
    std::shared_ptr<TriggerSrv::Response> res)
{
    handle_trigger(handle_->clear_fault, "clear_fault", res);
}

void CynlrArmNode::handle_zero_ft_sensor(
    std::shared_ptr<TriggerSrv::Request>,
    std::shared_ptr<TriggerSrv::Response> res)
{
    handle_trigger(handle_->zero_ft_sensor, "zero_ft_sensor", res);
}

void CynlrArmNode::handle_set_tool(
    std::shared_ptr<SetToolSrv::Request> req,
    std::shared_ptr<SetToolSrv::Response> res)
{
    auto r = handle_->set_tool(tool_from_request(*req));
    res->success = static_cast<bool>(r);
    res->message = r ? "Tool set" : r.error().message;
}

void CynlrArmNode::handle_update_tool(
    std::shared_ptr<SetToolSrv::Request> req,
    std::shared_ptr<SetToolSrv::Response> res)
{
    auto r = handle_->update_tool(tool_from_request(*req));
    res->success = static_cast<bool>(r);
    res->message = r ? "Tool updated" : r.error().message;
}

void CynlrArmNode::handle_get_tool(
    std::shared_ptr<GetToolSrv::Request>,
    std::shared_ptr<GetToolSrv::Response> res)
{
    auto r = handle_->get_tool();
    if (!r) {
        res->success = false;
        res->message = r.error().message;
        return;
    }
    res->success  = true;
    res->message  = "OK";
    res->mass_kg  = r->mass_kg;
    std::copy(r->com.begin(),      r->com.end(),      res->com.begin());
    std::copy(r->inertia.begin(),  r->inertia.end(),  res->inertia.begin());
    std::copy(r->tcp_pose.begin(), r->tcp_pose.end(), res->tcp_pose.begin());
}

void CynlrArmNode::handle_is_motion_running(
    std::shared_ptr<IsMotionRunningSrv::Request>,
    std::shared_ptr<IsMotionRunningSrv::Response> res)
{
    res->running = motion_running_.load();
}

// ---------------------------------------------------------------------------
// MoveL
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CynlrArmNode::move_l_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveLAction::Goal>)
{
    if (motion_running_.load()) {
        RCLCPP_WARN(get_logger(), "[%s] Rejecting MoveL — motion already running", prefix_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CynlrArmNode::move_l_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>>)
{
    handle_->stop_motion();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CynlrArmNode::move_l_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle)
{
    std::thread([this, handle]() { execute_move_l(handle); }).detach();
}

void CynlrArmNode::execute_move_l(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveLAction>> handle)
{
    motion_running_ = true;
    const auto& goal = *handle->get_goal();

    cynlr::arm::CartesianTarget target{};
    for (int i = 0; i < 7; ++i) target.pose[i] = goal.target_pose[i];
    auto params = motion_params(goal.max_linear_vel, 0.0, goal.max_linear_acc, 0.0);

    // Non-blocking — returns immediately after issuing the command
    auto cmd = handle_->move_l(target, params);
    if (!cmd) {
        auto result = std::make_shared<MoveLAction::Result>();
        result->success = false;
        result->message = cmd.error().message;
        handle->abort(result);
        motion_running_ = false;
        return;
    }

    auto feedback = std::make_shared<MoveLAction::Feedback>();
    while (rclcpp::ok()) {
        if (handle->is_canceling()) {
            handle_->stop_motion();
            auto result = std::make_shared<MoveLAction::Result>();
            result->success = false; result->message = "Cancelled";
            handle->canceled(result);
            motion_running_ = false;
            return;
        }
        if (auto st = handle_->get_state()) {
            for (int i = 0; i < 7; ++i) feedback->current_pose[i] = st->tcp_pose[i];
            double dx = st->tcp_pose[0] - goal.target_pose[0];
            double dy = st->tcp_pose[1] - goal.target_pose[1];
            double dz = st->tcp_pose[2] - goal.target_pose[2];
            feedback->distance_remaining = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        handle->publish_feedback(feedback);
        auto done = handle_->is_motion_complete();
        if (done && *done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto result = std::make_shared<MoveLAction::Result>();
    result->success = true;
    if (auto st = handle_->get_state())
        for (int i = 0; i < 7; ++i) result->final_pose[i] = st->tcp_pose[i];
    handle->succeed(result);
    motion_running_ = false;
}

// ---------------------------------------------------------------------------
// MoveJ
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CynlrArmNode::move_j_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const MoveJAction::Goal>)
{
    if (motion_running_.load()) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CynlrArmNode::move_j_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>>)
{
    handle_->stop_motion();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CynlrArmNode::move_j_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle)
{
    std::thread([this, handle]() { execute_move_j(handle); }).detach();
}

void CynlrArmNode::execute_move_j(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveJAction>> handle)
{
    motion_running_ = true;
    const auto& goal = *handle->get_goal();

    cynlr::arm::JointTarget target{};
    for (int i = 0; i < 7; ++i) target.positions[i] = goal.target_positions[i];
    auto params = motion_params(0.0, goal.max_joint_vel, 0.0, goal.max_joint_acc);

    // Non-blocking
    auto cmd = handle_->move_j(target, params);
    if (!cmd) {
        auto result = std::make_shared<MoveJAction::Result>();
        result->success = false; result->message = cmd.error().message;
        handle->abort(result);
        motion_running_ = false;
        return;
    }

    auto feedback = std::make_shared<MoveJAction::Feedback>();
    while (rclcpp::ok()) {
        if (handle->is_canceling()) {
            handle_->stop_motion();
            auto result = std::make_shared<MoveJAction::Result>();
            result->success = false; result->message = "Cancelled";
            handle->canceled(result);
            motion_running_ = false;
            return;
        }
        if (auto st = handle_->get_state()) {
            for (int i = 0; i < 7; ++i) {
                feedback->current_positions[i] = st->joint_positions[i];
                feedback->position_error[i]    = goal.target_positions[i] - st->joint_positions[i];
            }
        }
        handle->publish_feedback(feedback);
        auto done = handle_->is_motion_complete();
        if (done && *done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto result = std::make_shared<MoveJAction::Result>();
    result->success = true;
    if (auto st = handle_->get_state())
        for (int i = 0; i < 7; ++i) result->final_positions[i] = st->joint_positions[i];
    handle->succeed(result);
    motion_running_ = false;
}

// ---------------------------------------------------------------------------
// MovePTP
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse CynlrArmNode::move_ptp_goal(
    const rclcpp_action::GoalUUID&, std::shared_ptr<const MovePTPAction::Goal>)
{
    if (motion_running_.load()) return rclcpp_action::GoalResponse::REJECT;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CynlrArmNode::move_ptp_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>>)
{
    handle_->stop_motion();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CynlrArmNode::move_ptp_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle)
{
    std::thread([this, handle]() { execute_move_ptp(handle); }).detach();
}

void CynlrArmNode::execute_move_ptp(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MovePTPAction>> handle)
{
    motion_running_ = true;
    const auto& goal = *handle->get_goal();

    cynlr::arm::CartesianTarget target{};
    for (int i = 0; i < 7; ++i) target.pose[i] = goal.target_pose[i];
    auto params = motion_params(0.0, goal.max_joint_vel, 0.0, goal.max_joint_acc);

    // Non-blocking
    auto cmd = handle_->move_ptp(target, params);
    if (!cmd) {
        auto result = std::make_shared<MovePTPAction::Result>();
        result->success = false; result->message = cmd.error().message;
        handle->abort(result);
        motion_running_ = false;
        return;
    }

    auto feedback = std::make_shared<MovePTPAction::Feedback>();
    while (rclcpp::ok()) {
        if (handle->is_canceling()) {
            handle_->stop_motion();
            auto result = std::make_shared<MovePTPAction::Result>();
            result->success = false; result->message = "Cancelled";
            handle->canceled(result);
            motion_running_ = false;
            return;
        }
        if (auto st = handle_->get_state()) {
            for (int i = 0; i < 7; ++i) feedback->current_pose[i] = st->tcp_pose[i];
            double dx = st->tcp_pose[0] - goal.target_pose[0];
            double dy = st->tcp_pose[1] - goal.target_pose[1];
            double dz = st->tcp_pose[2] - goal.target_pose[2];
            feedback->distance_remaining = std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        handle->publish_feedback(feedback);
        auto done = handle_->is_motion_complete();
        if (done && *done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    auto result = std::make_shared<MovePTPAction::Result>();
    result->success = true;
    if (auto st = handle_->get_state())
        for (int i = 0; i < 7; ++i) result->final_pose[i] = st->tcp_pose[i];
    handle->succeed(result);
    motion_running_ = false;
}

} // namespace cynlr_arm_node
