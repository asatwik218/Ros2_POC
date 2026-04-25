#include "cynlr_hardware/cynlr_hardware_interface.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "cynlr_arm_core/arm_factory.hpp"
#include "cynlr_arm_core/error.hpp"

namespace cynlr_hardware {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

namespace {

// Same bit_cast as flexiv_robot_states.hpp — reinterprets memory without UB.
// Required because StateInterface only stores double*, but we need to pass pointers.
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

bool is_nan(double v) { return v != v; }

bool all_valid(const double* arr, int n)
{
    for (int i = 0; i < n; ++i)
        if (is_nan(arr[i])) return false;
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

hardware_interface::CallbackReturn CynlrHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        return hardware_interface::CallbackReturn::ERROR;

    auto& params = info_.hardware_parameters;

    auto require_param = [&](const std::string& key, std::string& out) -> bool {
        auto it = params.find(key);
        if (it == params.end()) {
            RCLCPP_FATAL(getLogger(), "Missing required hardware parameter '%s'", key.c_str());
            return false;
        }
        out = it->second;
        return true;
    };

    if (!require_param("prefix", prefix_)) return hardware_interface::CallbackReturn::ERROR;
    if (!require_param("vendor", arm_config_.vendor)) return hardware_interface::CallbackReturn::ERROR;
    if (!require_param("serial_number", arm_config_.serial_number))
        return hardware_interface::CallbackReturn::ERROR;

    // ip_address is optional (some vendors auto-discover)
    if (auto it = params.find("ip_address"); it != params.end())
        arm_config_.ip_address = it->second;

    arm_config_.num_joints = kDOF;

    // Pass any remaining params to the arm as vendor-specific key-value pairs
    for (auto& [k, v] : params) {
        if (k != "prefix" && k != "vendor" && k != "serial_number" && k != "ip_address")
            arm_config_.params[k] = v;
    }

    if (static_cast<int>(info_.joints.size()) != kDOF) {
        RCLCPP_FATAL(getLogger(), "[%s] Expected 7 joints, got %zu",
            prefix_.c_str(), info_.joints.size());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Validate each joint has position+velocity+effort command and state interfaces
    for (const auto& joint : info_.joints) {
        auto check = [&](const auto& interfaces, const char* kind) -> bool {
            if (interfaces.size() != 3) {
                RCLCPP_FATAL(getLogger(),
                    "[%s] Joint '%s' has %zu %s interfaces, expected 3",
                    prefix_.c_str(), joint.name.c_str(), interfaces.size(), kind);
                return false;
            }
            return true;
        };
        if (!check(joint.command_interfaces, "command")) return hardware_interface::CallbackReturn::ERROR;
        if (!check(joint.state_interfaces,   "state"))   return hardware_interface::CallbackReturn::ERROR;
    }

    // Create the arm object — does NOT connect yet, connection happens in on_activate
    arm_ = cynlr::arm::create_arm(arm_config_);
    if (!arm_) {
        RCLCPP_FATAL(getLogger(), "[%s] Unknown vendor '%s'",
            prefix_.c_str(), arm_config_.vendor.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }
    arm_raw_ptr_ = arm_.get();

    // Pre-fill command buffers with NaN — write() skips NaN commands
    cmd_pos_.fill(std::numeric_limits<double>::quiet_NaN());
    cmd_vel_.fill(std::numeric_limits<double>::quiet_NaN());
    cmd_eff_.fill(std::numeric_limits<double>::quiet_NaN());
    cmd_cart_.fill(std::numeric_limits<double>::quiet_NaN());
    gpio_out_.fill(std::numeric_limits<double>::quiet_NaN());
    last_gpio_out_.fill(std::numeric_limits<double>::quiet_NaN());

    RCLCPP_INFO(getLogger(), "[%s] Initialised (vendor=%s, sn=%s)",
        prefix_.c_str(), arm_config_.vendor.c_str(), arm_config_.serial_number.c_str());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CynlrHardwareInterface::on_activate(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(getLogger(), "[%s] Connecting and enabling arm...", prefix_.c_str());

    auto conn = arm_->connect(arm_config_);
    if (!conn) {
        RCLCPP_FATAL(getLogger(), "[%s] connect() failed: %s",
            prefix_.c_str(), conn.error().message.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    if (arm_->has_fault()) {
        RCLCPP_WARN(getLogger(), "[%s] Fault detected, attempting to clear...", prefix_.c_str());
        auto clr = arm_->clear_fault();
        if (!clr) {
            RCLCPP_FATAL(getLogger(), "[%s] clear_fault() failed: %s",
                prefix_.c_str(), clr.error().message.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }

    auto en = arm_->enable();
    if (!en) {
        RCLCPP_FATAL(getLogger(), "[%s] enable() failed: %s",
            prefix_.c_str(), en.error().message.c_str());
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Seed cmd_pos_ from current state so the arm holds position on first write
    auto st = arm_->get_state();
    if (st) {
        for (int i = 0; i < kDOF; ++i)
            cmd_pos_[i] = st->joint_positions[i];
        arm_state_ = *st;
    }

    RCLCPP_INFO(getLogger(), "[%s] Arm is active", prefix_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CynlrHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(getLogger(), "[%s] Stopping arm...", prefix_.c_str());

    if (active_mode_ != ActiveMode::NONE)
        arm_->stop_streaming();
    active_mode_ = ActiveMode::NONE;

    arm_->stop();
    arm_->disconnect();

    RCLCPP_INFO(getLogger(), "[%s] Arm stopped", prefix_.c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// Interface export
// ---------------------------------------------------------------------------

std::vector<hardware_interface::StateInterface>
CynlrHardwareInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> si;

    // Standard joint states
    for (int i = 0; i < kDOF; ++i) {
        si.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &hw_pos_[i]);
        si.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &hw_vel_[i]);
        si.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,   &hw_eff_[i]);
    }

    // FT sensor — 18 individual doubles (raw + ext_tcp + ext_world, 6 each)
    for (int i = 0; i < 6; ++i) {
        si.emplace_back(prefix_ + "ft_sensor",
            "raw_" + std::to_string(i), &arm_state_.ft_sensor_raw[i]);
        si.emplace_back(prefix_ + "ft_sensor",
            "ext_tcp_" + std::to_string(i), &arm_state_.ext_wrench_in_tcp[i]);
        si.emplace_back(prefix_ + "ft_sensor",
            "ext_world_" + std::to_string(i), &arm_state_.ext_wrench_in_world[i]);
    }

    // TCP pose (7) and velocity (6)
    for (int i = 0; i < 7; ++i)
        si.emplace_back(prefix_ + "tcp", "pose_" + std::to_string(i), &arm_state_.tcp_pose[i]);
    for (int i = 0; i < 6; ++i)
        si.emplace_back(prefix_ + "tcp", "vel_" + std::to_string(i),  &arm_state_.tcp_velocity[i]);

    // Full ArmState struct — pointer stored via bit_cast trick.
    // CynlrStateBroadcaster reads this and casts it back to ArmState*.
    si.emplace_back(prefix_ + "cynlr_arm_state", "full_state_ptr",
        reinterpret_cast<double*>(&arm_state_ptr_));

    // Raw ArmInterface* — CynlrNrtPassthroughController reads this to call NRT moves directly.
    si.emplace_back(prefix_ + "cynlr_arm_ctrl", "arm_interface_ptr",
        reinterpret_cast<double*>(&arm_raw_ptr_));

    // GPIO inputs
    for (int i = 0; i < kIOPorts; ++i)
        si.emplace_back(prefix_ + "gpio", "in_" + std::to_string(i), &gpio_in_[i]);

    return si;
}

std::vector<hardware_interface::CommandInterface>
CynlrHardwareInterface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> ci;

    // Standard joint commands
    for (int i = 0; i < kDOF; ++i) {
        ci.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_POSITION, &cmd_pos_[i]);
        ci.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &cmd_vel_[i]);
        ci.emplace_back(info_.joints[i].name,
            hardware_interface::HW_IF_EFFORT,   &cmd_eff_[i]);
    }

    // Cartesian command (escape hatch #3) — 7 pose + 6 wrench doubles
    for (int i = 0; i < 7; ++i)
        ci.emplace_back(prefix_ + "cartesian_cmd",
            "pose_" + std::to_string(i), &cmd_cart_[i]);
    for (int i = 0; i < 6; ++i)
        ci.emplace_back(prefix_ + "cartesian_cmd",
            "wrench_" + std::to_string(i), &cmd_cart_[7 + i]);

    // GPIO outputs
    for (int i = 0; i < kIOPorts; ++i)
        ci.emplace_back(prefix_ + "gpio", "out_" + std::to_string(i), &gpio_out_[i]);

    return ci;
}

// ---------------------------------------------------------------------------
// Mode switching
// ---------------------------------------------------------------------------

hardware_interface::return_type CynlrHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
    pending_start_ = ActiveMode::NONE;
    pending_stop_  = false;

    // Count how many of our joints appear in stop_interfaces
    int stop_count = 0;
    for (const auto& key : stop_interfaces) {
        for (const auto& joint : info_.joints) {
            if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION
                || key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY
                || key == joint.name + "/" + hardware_interface::HW_IF_EFFORT) {
                ++stop_count;
                break;
            }
        }
    }
    // Check for cartesian stop
    bool cartesian_stopping = false;
    for (const auto& key : stop_interfaces) {
        if (key.rfind(prefix_ + "cartesian_cmd/", 0) == 0) {
            cartesian_stopping = true;
            break;
        }
    }
    if (stop_count > 0 || cartesian_stopping)
        pending_stop_ = true;

    // Detect which mode is being started
    int pos_count = 0, vel_count = 0, eff_count = 0;
    bool cartesian_starting = false;

    for (const auto& key : start_interfaces) {
        for (const auto& joint : info_.joints) {
            if (key == joint.name + "/" + hardware_interface::HW_IF_POSITION) ++pos_count;
            if (key == joint.name + "/" + hardware_interface::HW_IF_VELOCITY) ++vel_count;
            if (key == joint.name + "/" + hardware_interface::HW_IF_EFFORT)   ++eff_count;
        }
        if (key.rfind(prefix_ + "cartesian_cmd/", 0) == 0) cartesian_starting = true;
    }

    // Reject mixed-mode or partial-joint requests
    auto non_zero = [](int a, int b, int c) { return (a > 0) + (b > 0) + (c > 0); };
    if (non_zero(pos_count, vel_count, eff_count) > 1) {
        RCLCPP_ERROR(getLogger(), "[%s] Mixed mode switch rejected", prefix_.c_str());
        return hardware_interface::return_type::ERROR;
    }
    if (pos_count > 0 && pos_count != kDOF) {
        RCLCPP_ERROR(getLogger(), "[%s] Partial joint switch rejected (%d/%d joints)",
            prefix_.c_str(), pos_count, kDOF);
        return hardware_interface::return_type::ERROR;
    }
    if (vel_count > 0 && vel_count != kDOF) {
        RCLCPP_ERROR(getLogger(), "[%s] Partial joint switch rejected (%d/%d joints)",
            prefix_.c_str(), vel_count, kDOF);
        return hardware_interface::return_type::ERROR;
    }
    if (eff_count > 0 && eff_count != kDOF) {
        RCLCPP_ERROR(getLogger(), "[%s] Partial joint switch rejected (%d/%d joints)",
            prefix_.c_str(), eff_count, kDOF);
        return hardware_interface::return_type::ERROR;
    }

    if (pos_count == kDOF)     pending_start_ = ActiveMode::POSITION;
    else if (vel_count == kDOF) pending_start_ = ActiveMode::VELOCITY;
    else if (eff_count == kDOF) pending_start_ = ActiveMode::EFFORT;
    else if (cartesian_starting) pending_start_ = ActiveMode::CARTESIAN;

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CynlrHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>&, const std::vector<std::string>&)
{
    if (pending_stop_ && active_mode_ != ActiveMode::NONE) {
        arm_->stop_streaming();
        active_mode_ = ActiveMode::NONE;
    }

    if (pending_start_ == ActiveMode::POSITION) {
        cmd_pos_.fill(std::numeric_limits<double>::quiet_NaN()); // clear stale commands
        arm_->start_streaming(cynlr::arm::StreamMode::JOINT_POSITION);
        active_mode_ = ActiveMode::POSITION;

    } else if (pending_start_ == ActiveMode::VELOCITY) {
        cmd_vel_.fill(std::numeric_limits<double>::quiet_NaN());
        arm_->start_streaming(cynlr::arm::StreamMode::JOINT_POSITION); // velocity via pos+vel
        active_mode_ = ActiveMode::VELOCITY;

    } else if (pending_start_ == ActiveMode::EFFORT) {
        cmd_eff_.fill(std::numeric_limits<double>::quiet_NaN());
        arm_->start_streaming(cynlr::arm::StreamMode::JOINT_TORQUE);
        active_mode_ = ActiveMode::EFFORT;

    } else if (pending_start_ == ActiveMode::CARTESIAN) {
        cmd_cart_.fill(std::numeric_limits<double>::quiet_NaN());
        arm_->start_streaming(cynlr::arm::StreamMode::CARTESIAN_MOTION_FORCE);
        active_mode_ = ActiveMode::CARTESIAN;
    }

    pending_start_ = ActiveMode::NONE;
    pending_stop_  = false;

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// Control loop — called at 1kHz by the controller manager
// ---------------------------------------------------------------------------

hardware_interface::return_type CynlrHardwareInterface::read(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    auto result = arm_->get_state();
    if (!result)
        return hardware_interface::return_type::OK; // keep last-known values on transient failure

    arm_state_ = *result;

    for (int i = 0; i < kDOF; ++i) {
        hw_pos_[i] = arm_state_.joint_positions[i];
        hw_vel_[i] = arm_state_.joint_velocities[i];
        hw_eff_[i] = arm_state_.joint_torques[i];
    }

    // Read GPIO inputs if the arm supports digital IO
    if (auto* dio = dynamic_cast<cynlr::arm::DigitalIOControllable*>(arm_.get())) {
        auto inputs = dio->get_digital_inputs();
        if (inputs) {
            for (int i = 0; i < kIOPorts && i < static_cast<int>(inputs->size()); ++i)
                gpio_in_[i] = (*inputs)[i] ? 1.0 : 0.0;
        }
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CynlrHardwareInterface::write(
    const rclcpp::Time&, const rclcpp::Duration&)
{
    using cynlr::arm::StreamCommand;
    using cynlr::arm::StreamMode;

    switch (active_mode_) {

    case ActiveMode::POSITION: {
        if (!all_valid(cmd_pos_.data(), kDOF)) break;
        StreamCommand cmd{};
        cmd.mode = StreamMode::JOINT_POSITION;
        for (int i = 0; i < kDOF; ++i) {
            cmd.joint_position[i] = cmd_pos_[i];
            cmd.joint_velocity[i] = is_nan(cmd_vel_[i]) ? 0.0 : cmd_vel_[i];
        }
        arm_->stream_command(cmd);
        break;
    }

    case ActiveMode::VELOCITY: {
        if (!all_valid(cmd_vel_.data(), kDOF)) break;
        // Velocity mode: send current position + desired velocity to let arm's
        // internal trajectory generator handle the integration
        StreamCommand cmd{};
        cmd.mode = StreamMode::JOINT_POSITION;
        for (int i = 0; i < kDOF; ++i) {
            cmd.joint_position[i] = hw_pos_[i];
            cmd.joint_velocity[i] = cmd_vel_[i];
        }
        arm_->stream_command(cmd);
        break;
    }

    case ActiveMode::EFFORT: {
        if (!all_valid(cmd_eff_.data(), kDOF)) break;
        StreamCommand cmd{};
        cmd.mode = StreamMode::JOINT_TORQUE;
        for (int i = 0; i < kDOF; ++i)
            cmd.joint_torque[i] = cmd_eff_[i];
        arm_->stream_command(cmd);
        break;
    }

    case ActiveMode::CARTESIAN: {
        if (is_nan(cmd_cart_[0])) break; // check first element as sentinel
        StreamCommand cmd{};
        cmd.mode = StreamMode::CARTESIAN_MOTION_FORCE;
        for (int i = 0; i < 7; ++i) cmd.cartesian_pose[i] = cmd_cart_[i];
        for (int i = 0; i < 6; ++i) cmd.wrench[i]         = cmd_cart_[7 + i];
        arm_->stream_command(cmd);
        break;
    }

    default:
        break;
    }

    // GPIO writes — only send when values have changed to avoid hammering the arm
    if (auto* dio = dynamic_cast<cynlr::arm::DigitalIOControllable*>(arm_.get())) {
        std::vector<std::pair<int, bool>> changes;
        for (int i = 0; i < kIOPorts; ++i) {
            if (is_nan(gpio_out_[i])) continue;
            bool val = gpio_out_[i] > 0.5;
            bool last = !is_nan(last_gpio_out_[i]) && last_gpio_out_[i] > 0.5;
            if (is_nan(last_gpio_out_[i]) || val != last) {
                changes.emplace_back(i, val);
                last_gpio_out_[i] = gpio_out_[i];
            }
        }
        if (!changes.empty())
            dio->set_digital_outputs(changes);
    }

    return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------

rclcpp::Logger CynlrHardwareInterface::getLogger()
{
    return rclcpp::get_logger("CynlrHardwareInterface");
}

} // namespace cynlr_hardware

PLUGINLIB_EXPORT_CLASS(
    cynlr_hardware::CynlrHardwareInterface,
    hardware_interface::SystemInterface)
