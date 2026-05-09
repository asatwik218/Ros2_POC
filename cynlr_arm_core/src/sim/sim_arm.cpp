#include "sim/sim_arm.hpp"
#include <thread>

namespace cynlr::arm {

Expected<void> SimArm::connect(const ArmConfig& config) {
    std::lock_guard lock(state_mutex_);
    config_ = config;
    connected_ = true;
    state_ = ArmState{};
    // Parse optional sim params
    if (auto it = config.params.find("noise_stddev"); it != config.params.end()) {
        noise_stddev_ = std::stod(it->second);
    }
    if (auto it = config.params.find("latency_ms"); it != config.params.end()) {
        latency_ms_ = std::stod(it->second);
    }
    return {};
}

Expected<void> SimArm::disconnect() {
    std::lock_guard lock(state_mutex_);
    connected_ = false;
    enabled_ = false;
    streaming_ = false;
    return {};
}

Expected<void> SimArm::enable() {
    std::lock_guard lock(state_mutex_);
    if (!connected_) {
        return make_error(ErrorCode::NOT_CONNECTED, "SimArm: not connected");
    }
    if (has_fault_) {
        return make_error(ErrorCode::FAULT, "SimArm: fault active, clear before enabling");
    }
    enabled_ = true;
    state_.operational = true;
    return {};
}

Expected<void> SimArm::stop() {
    std::lock_guard lock(state_mutex_);
    streaming_ = false;
    motion_complete_ = true;
    nrt_active_ = false;
    return {};
}

Expected<ArmState> SimArm::get_state() {
    std::lock_guard lock(state_mutex_);
    if (!connected_) {
        return make_error(ErrorCode::NOT_CONNECTED, "SimArm: not connected");
    }
    ArmState s = state_;
    // Apply optional Gaussian noise to joint positions
    if (noise_stddev_ > 0.0) {
        std::normal_distribution<double> noise(0.0, noise_stddev_);
        for (auto& q : s.joint_positions) {
            q += noise(rng_);
        }
    }
    // Simulate latency
    if (latency_ms_ > 0.0) {
        std::this_thread::sleep_for(
            std::chrono::microseconds(static_cast<long>(latency_ms_ * 1000)));
    }
    return s;
}

bool SimArm::is_connected() const {
    std::lock_guard lock(state_mutex_);
    return connected_;
}

bool SimArm::has_fault() const {
    std::lock_guard lock(state_mutex_);
    return has_fault_;
}

Expected<void> SimArm::clear_fault() {
    std::lock_guard lock(state_mutex_);
    if (!has_fault_) {
        return make_error(ErrorCode::INVALID_ARGUMENT, "SimArm: no fault to clear");
    }
    has_fault_ = false;
    state_.fault = false;
    state_.operational = enabled_;
    return {};
}

Expected<void> SimArm::run_auto_recovery() {
    std::lock_guard lock(state_mutex_);
    has_fault_ = false;
    state_.fault = false;
    state_.operational = enabled_;
    return {};
}

Expected<void> SimArm::move_l(const CartesianTarget& target, const MotionParams&) {
    std::lock_guard lock(state_mutex_);
    if (!enabled_) return make_error(ErrorCode::NOT_ENABLED, "SimArm: not enabled");
    state_.tcp_pose = target.pose;
    motion_complete_ = false;
    nrt_active_ = true;
    motion_start_ = std::chrono::steady_clock::now();
    return {};
}

Expected<void> SimArm::move_j(const JointTarget& target, const MotionParams&) {
    std::lock_guard lock(state_mutex_);
    if (!enabled_) return make_error(ErrorCode::NOT_ENABLED, "SimArm: not enabled");
    state_.joint_positions = target.positions;
    motion_complete_ = false;
    nrt_active_ = true;
    motion_start_ = std::chrono::steady_clock::now();
    return {};
}

Expected<void> SimArm::move_ptp(const CartesianTarget& target, const MotionParams&) {
    std::lock_guard lock(state_mutex_);
    if (!enabled_) return make_error(ErrorCode::NOT_ENABLED, "SimArm: not enabled");
    state_.tcp_pose = target.pose;
    motion_complete_ = false;
    nrt_active_ = true;
    motion_start_ = std::chrono::steady_clock::now();
    return {};
}

Expected<bool> SimArm::is_motion_complete() {
    std::lock_guard lock(state_mutex_);
    if (!motion_complete_ &&
        std::chrono::steady_clock::now() >= motion_start_ + motion_delay_) {
        motion_complete_ = true;
        nrt_active_ = false;
    }
    return motion_complete_;
}

Expected<void> SimArm::start_streaming(StreamMode mode) {
    std::lock_guard lock(state_mutex_);
    if (!enabled_) return make_error(ErrorCode::NOT_ENABLED, "SimArm: not enabled");
    stream_mode_ = mode;
    streaming_ = true;
    return {};
}

Expected<void> SimArm::stream_command(const StreamCommand& cmd) {
    std::lock_guard lock(state_mutex_);
    if (!streaming_) return make_error(ErrorCode::NOT_ENABLED, "SimArm: not streaming");
    switch (cmd.mode) {
        case StreamMode::JOINT_POSITION:
            state_.joint_positions = cmd.joint_position;
            state_.joint_velocities = cmd.joint_velocity;
            break;
        case StreamMode::JOINT_TORQUE:
            state_.joint_torques = cmd.joint_torque;
            break;
        case StreamMode::CARTESIAN_MOTION_FORCE:
            state_.tcp_pose = cmd.cartesian_pose;
            break;
    }
    return {};
}

Expected<void> SimArm::stop_streaming() {
    std::lock_guard lock(state_mutex_);
    streaming_ = false;
    return {};
}

Expected<void> SimArm::set_tool(const ToolInfo& tool) {
    std::lock_guard lock(state_mutex_);
    cached_tool_ = tool;
    return {};
}

Expected<ToolInfo> SimArm::get_tool() const {
    std::lock_guard lock(state_mutex_);
    return cached_tool_;
}

Expected<void> SimArm::zero_ft_sensor() {
    std::lock_guard lock(state_mutex_);
    state_.ft_sensor_raw = {};
    return {};
}

std::vector<std::string> SimArm::supported_features() const {
    return {"force_control", "null_space", "streaming_1khz"};
}

// ForceControllable
Expected<void> SimArm::set_force_control_axis(const ForceAxisConfig&) { return {}; }
Expected<void> SimArm::set_force_control_frame(const CartesianPose&) { return {}; }
Expected<void> SimArm::set_passive_force_control(bool) { return {}; }

Expected<void> SimArm::move_hybrid_motion_force(
    const CartesianTarget& pose_target,
    const std::array<double, 6>&,  // wrench_setpoint ignored in sim (no contact model)
    const MotionParams&)
{
    std::lock_guard lock(state_mutex_);
    if (!enabled_) return make_error(ErrorCode::NOT_ENABLED, "SimArm: not enabled");
    state_.tcp_pose = pose_target.pose;
    motion_complete_ = false;
    nrt_active_ = true;
    motion_start_ = std::chrono::steady_clock::now();
    return {};
}

// NullSpaceConfigurable
Expected<void> SimArm::set_null_space_posture(const JointState&) { return {}; }
Expected<void> SimArm::set_null_space_objectives(const NullSpaceWeights&) { return {}; }

// Test helpers
void SimArm::inject_fault() {
    std::lock_guard lock(state_mutex_);
    has_fault_ = true;
    state_.fault = true;
    state_.operational = false;
}

void SimArm::set_noise_stddev(double stddev) {
    std::lock_guard lock(state_mutex_);
    noise_stddev_ = stddev;
}

void SimArm::set_latency_ms(double ms) {
    std::lock_guard lock(state_mutex_);
    latency_ms_ = ms;
}

} // namespace cynlr::arm
