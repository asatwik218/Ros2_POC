#include "cynlr_state_broadcaster/cynlr_state_broadcaster.hpp"

#include <cstring>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cynlr_arm_controllers {

namespace {

// Same bit_cast as in the hardware interface and flexiv_robot_states.hpp
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

controller_interface::CallbackReturn CynlrStateBroadcaster::on_init()
{
    auto_declare<std::string>("prefix", "");
    auto_declare<double>("publish_rate", 100.0);
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State&)
{
    prefix_       = get_node()->get_parameter("prefix").as_string();
    publish_rate_ = get_node()->get_parameter("publish_rate").as_double();

    if (prefix_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Parameter 'prefix' must not be empty");
        return controller_interface::CallbackReturn::ERROR;
    }

    auto node = get_node();

    // Create realtime publishers
    pub_arm_state_ = std::make_shared<ArmStatePub>(
        node->create_publisher<cynlr_arm_interfaces::msg::ArmState>(
            prefix_ + "arm_state", rclcpp::SystemDefaultsQoS()));

    pub_tcp_pose_ = std::make_shared<PosePub>(
        node->create_publisher<geometry_msgs::msg::PoseStamped>(
            prefix_ + "tcp_pose", rclcpp::SystemDefaultsQoS()));

    pub_ft_raw_ = std::make_shared<WrenchPub>(
        node->create_publisher<geometry_msgs::msg::WrenchStamped>(
            prefix_ + "ft_raw", rclcpp::SystemDefaultsQoS()));

    pub_ext_wrench_tcp_ = std::make_shared<WrenchPub>(
        node->create_publisher<geometry_msgs::msg::WrenchStamped>(
            prefix_ + "ext_wrench_tcp", rclcpp::SystemDefaultsQoS()));

    pub_ext_wrench_world_ = std::make_shared<WrenchPub>(
        node->create_publisher<geometry_msgs::msg::WrenchStamped>(
            prefix_ + "ext_wrench_world", rclcpp::SystemDefaultsQoS()));

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrStateBroadcaster::on_activate(
    const rclcpp_lifecycle::State&)
{
    // Find the "full_state_ptr" state interface and recover the ArmState* via bit_cast
    for (const auto& si : state_interfaces_) {
        if (si.get_name() == prefix_ + "cynlr_arm_state/full_state_ptr") {
            double raw = si.get_value();
            arm_state_ptr_ = bit_cast<cynlr::arm::ArmState*>(raw);
            break;
        }
    }

    if (!arm_state_ptr_) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "[%s] Could not find full_state_ptr state interface", prefix_.c_str());
        return controller_interface::CallbackReturn::ERROR;
    }

    // Compute decimation ratio: how many update() calls per ArmState publish
    // update_rate comes from controller_manager yaml (e.g. 1000 Hz)
    // We want to publish at publish_rate_ Hz
    double update_rate = static_cast<double>(get_node()->get_parameter("update_rate").as_int());
    if (update_rate > 0.0 && publish_rate_ > 0.0)
        publish_every_ = std::max(1, static_cast<int>(update_rate / publish_rate_));
    else
        publish_every_ = 1;

    update_count_ = 0;

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CynlrStateBroadcaster::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    arm_state_ptr_ = nullptr;
    return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface configuration
// ---------------------------------------------------------------------------

controller_interface::InterfaceConfiguration
CynlrStateBroadcaster::command_interface_configuration() const
{
    // Read-only broadcaster — no command interfaces
    return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
CynlrStateBroadcaster::state_interface_configuration() const
{
    // Claim the full_state_ptr interface; individual doubles are read directly from
    // the ArmState struct after dereferencing the pointer
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.push_back(prefix_ + "cynlr_arm_state/full_state_ptr");
    return cfg;
}

// ---------------------------------------------------------------------------
// Update — called at controller_manager update_rate (1kHz by default)
// ---------------------------------------------------------------------------

controller_interface::return_type CynlrStateBroadcaster::update(
    const rclcpp::Time& time, const rclcpp::Duration&)
{
    if (!arm_state_ptr_)
        return controller_interface::return_type::OK;

    const auto& s = *arm_state_ptr_;

    // Publish TCP pose and wrenches every cycle (high frequency for servo/force loops)
    if (pub_tcp_pose_->trylock()) {
        auto& msg = pub_tcp_pose_->msg_;
        msg.header.stamp    = time;
        msg.header.frame_id = "world";
        msg.pose.position.x    = s.tcp_pose[0];
        msg.pose.position.y    = s.tcp_pose[1];
        msg.pose.position.z    = s.tcp_pose[2];
        msg.pose.orientation.w = s.tcp_pose[3];
        msg.pose.orientation.x = s.tcp_pose[4];
        msg.pose.orientation.y = s.tcp_pose[5];
        msg.pose.orientation.z = s.tcp_pose[6];
        pub_tcp_pose_->unlockAndPublish();
    }

    auto fill_wrench = [&](auto& pub, const std::array<double,6>& data,
                           const std::string& frame) {
        if (pub->trylock()) {
            auto& msg = pub->msg_;
            msg.header.stamp    = time;
            msg.header.frame_id = frame;
            msg.wrench.force.x  = data[0]; msg.wrench.force.y  = data[1];
            msg.wrench.force.z  = data[2]; msg.wrench.torque.x = data[3];
            msg.wrench.torque.y = data[4]; msg.wrench.torque.z = data[5];
            pub->unlockAndPublish();
        }
    };

    fill_wrench(pub_ft_raw_,          s.ft_sensor_raw,       prefix_ + "flange_link");
    fill_wrench(pub_ext_wrench_tcp_,  s.ext_wrench_in_tcp,   prefix_ + "flange_link");
    fill_wrench(pub_ext_wrench_world_,s.ext_wrench_in_world,  "world");

    // Publish full ArmState at the configured lower rate (default 100 Hz)
    if (++update_count_ >= publish_every_) {
        update_count_ = 0;

        if (pub_arm_state_->trylock()) {
            auto& msg = pub_arm_state_->msg_;
            msg.header.stamp = time;

            for (int i = 0; i < 7; ++i) {
                msg.joint_positions[i]         = s.joint_positions[i];
                msg.joint_velocities[i]        = s.joint_velocities[i];
                msg.joint_torques[i]           = s.joint_torques[i];
                msg.joint_torques_external[i]  = s.joint_torques_external[i];
                msg.tcp_pose[i]                = s.tcp_pose[i];
            }
            for (int i = 0; i < 6; ++i) {
                msg.tcp_velocity[i]          = s.tcp_velocity[i];
                msg.ft_sensor_raw[i]         = s.ft_sensor_raw[i];
                msg.ext_wrench_in_tcp[i]     = s.ext_wrench_in_tcp[i];
                msg.ext_wrench_in_world[i]   = s.ext_wrench_in_world[i];
            }
            msg.fault       = s.fault;
            msg.operational = s.operational;
            msg.estopped    = s.estopped;

            pub_arm_state_->unlockAndPublish();
        }
    }

    return controller_interface::return_type::OK;
}

} // namespace cynlr_arm_controllers

PLUGINLIB_EXPORT_CLASS(
    cynlr_arm_controllers::CynlrStateBroadcaster,
    controller_interface::ControllerInterface)
