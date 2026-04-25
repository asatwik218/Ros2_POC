#include "flexiv/flexiv_arm.hpp"
#include <flexiv/rdk/robot.hpp>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <map>
#include <thread>
#include <chrono>

namespace cynlr::arm {

FlexivArm::FlexivArm() = default;
FlexivArm::~FlexivArm() = default;

Expected<void> FlexivArm::connect(const ArmConfig& config) {
    try {
        config_ = config;
        // Pass local_ip as network_interface_whitelist so RDK binds to the correct interface.
        // If ip_address is empty, RDK auto-selects (may fail on multi-homed / WSL2 hosts).
        std::vector<std::string> iface_whitelist;
        if (!config.ip_address.empty()) {
            iface_whitelist.push_back(config.ip_address);
        }
        robot_ = std::make_unique<flexiv::rdk::Robot>(config.serial_number, iface_whitelist);
        spdlog::info("FlexivArm: connected to {}", config.serial_number);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::connect failed: ") + e.what());
    }
}

Expected<void> FlexivArm::disconnect() {
    robot_.reset();
    return {};
}

Expected<void> FlexivArm::enable() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->Enable();
        // Poll until operational or timeout
        int retries = 100;
        while (!robot_->operational() && retries-- > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (!robot_->operational()) {
            return make_error(ErrorCode::TIMEOUT, "FlexivArm: enable timed out");
        }
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::enable failed: ") + e.what());
    }
}

Expected<void> FlexivArm::stop() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->Stop();
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::stop failed: ") + e.what());
    }
}

Expected<ArmState> FlexivArm::get_state() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        auto rdk_state = robot_->states();
        ArmState s;
        for (size_t i = 0; i < 7 && i < rdk_state.q.size(); ++i)
            s.joint_positions[i] = rdk_state.q[i];
        for (size_t i = 0; i < 7 && i < rdk_state.dq.size(); ++i)
            s.joint_velocities[i] = rdk_state.dq[i];
        for (size_t i = 0; i < 7 && i < rdk_state.tau.size(); ++i)
            s.joint_torques[i] = rdk_state.tau[i];
        for (size_t i = 0; i < 7 && i < rdk_state.tau_ext.size(); ++i)
            s.joint_torques_external[i] = rdk_state.tau_ext[i];
        for (size_t i = 0; i < 7; ++i)
            s.tcp_pose[i] = rdk_state.tcp_pose[i];
        for (size_t i = 0; i < 6; ++i)
            s.tcp_velocity[i] = rdk_state.tcp_vel[i];
        for (size_t i = 0; i < 6; ++i)
            s.ft_sensor_raw[i] = rdk_state.ft_sensor_raw[i];
        for (size_t i = 0; i < 6; ++i)
            s.ext_wrench_in_tcp[i] = rdk_state.ext_wrench_in_tcp[i];
        for (size_t i = 0; i < 6; ++i)
            s.ext_wrench_in_world[i] = rdk_state.ext_wrench_in_world[i];
        s.fault = robot_->fault();
        s.operational = robot_->operational();
        s.estopped = !robot_->estop_released();
        return s;
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::get_state failed: ") + e.what());
    }
}

bool FlexivArm::is_connected() const {
    return robot_ && robot_->connected();
}

bool FlexivArm::has_fault() const {
    return robot_ && robot_->fault();
}

Expected<void> FlexivArm::clear_fault() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->ClearFault();
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::clear_fault failed: ") + e.what());
    }
}

Expected<void> FlexivArm::run_auto_recovery() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->RunAutoRecovery();
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::run_auto_recovery failed: ") + e.what());
    }
}

Expected<void> FlexivArm::move_l(const CartesianTarget& target, const MotionParams& params) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_CARTESIAN_MOTION_FORCE);
        // API: SendCartesianMotionForce(array<double,7> pose, array<double,6> wrench, ...)
        robot_->SendCartesianMotionForce(
            target.pose,
            std::array<double, flexiv::rdk::kCartDoF>{},
            {},
            params.max_linear_vel,
            params.max_angular_vel,
            params.max_linear_acc);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::MOTION_FAILED,
            std::string("FlexivArm::move_l failed: ") + e.what());
    }
}

Expected<void> FlexivArm::move_j(const JointTarget& target, const MotionParams& params) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->SwitchMode(flexiv::rdk::Mode::NRT_JOINT_POSITION);
        // API: SendJointPosition(positions, velocities, max_vel, max_acc) — 4 args
        std::vector<double> pos(target.positions.begin(), target.positions.end());
        std::vector<double> vel(7, 0.0);  // target velocity = 0 (let motion gen handle)
        std::vector<double> max_vel(7, params.max_joint_vel);
        std::vector<double> max_acc(7, params.max_joint_acc);
        robot_->SendJointPosition(pos, vel, max_vel, max_acc);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::MOTION_FAILED,
            std::string("FlexivArm::move_j failed: ") + e.what());
    }
}

Expected<void> FlexivArm::move_ptp(const CartesianTarget& target, const MotionParams& params) {
    return move_l(target, params);
}

Expected<bool> FlexivArm::is_motion_complete() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    return robot_->stopped();
}

Expected<void> FlexivArm::start_streaming(StreamMode mode) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
#ifndef __linux__
    if (mode == StreamMode::JOINT_TORQUE || mode == StreamMode::CARTESIAN_MOTION_FORCE) {
        return make_error(ErrorCode::UNSUPPORTED,
            "RT torque/Cartesian streaming only available on Linux");
    }
#endif
    try {
        stream_mode_ = mode;
        switch (mode) {
            case StreamMode::JOINT_POSITION:
                robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_POSITION);
                break;
            case StreamMode::JOINT_TORQUE:
                robot_->SwitchMode(flexiv::rdk::Mode::RT_JOINT_TORQUE);
                break;
            case StreamMode::CARTESIAN_MOTION_FORCE:
                robot_->SwitchMode(flexiv::rdk::Mode::RT_CARTESIAN_MOTION_FORCE);
                break;
        }
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::MODE_SWITCH_FAILED,
            std::string("FlexivArm::start_streaming failed: ") + e.what());
    }
}

Expected<void> FlexivArm::stream_command(const StreamCommand& cmd) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        switch (cmd.mode) {
            case StreamMode::JOINT_POSITION: {
                std::vector<double> pos(cmd.joint_position.begin(), cmd.joint_position.end());
                std::vector<double> vel(cmd.joint_velocity.begin(), cmd.joint_velocity.end());
                std::vector<double> acc(cmd.joint_acceleration.begin(), cmd.joint_acceleration.end());
                robot_->StreamJointPosition(pos, vel, acc);
                break;
            }
            case StreamMode::JOINT_TORQUE: {
                std::vector<double> tau(cmd.joint_torque.begin(), cmd.joint_torque.end());
                robot_->StreamJointTorque(tau, true);
                break;
            }
            case StreamMode::CARTESIAN_MOTION_FORCE:
                // API: StreamCartesianMotionForce(array<double,7> pose, array<double,6> wrench, ...)
                robot_->StreamCartesianMotionForce(cmd.cartesian_pose, cmd.wrench);
                break;
        }
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::stream_command failed: ") + e.what());
    }
}

Expected<void> FlexivArm::stop_streaming() {
    return stop();
}

Expected<void> FlexivArm::set_tool(const ToolInfo& tool) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        (void)tool;
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::set_tool failed: ") + e.what());
    }
}

Expected<void> FlexivArm::zero_ft_sensor() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        // ZeroFTSensor is a primitive in RDK 1.9, not a direct method
        robot_->ExecutePrimitive("ZeroFTSensor", {});
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION,
            std::string("FlexivArm::zero_ft_sensor failed: ") + e.what());
    }
}

std::vector<std::string> FlexivArm::supported_features() const {
    return {"force_control", "impedance", "null_space", "digital_io", "streaming_1khz"};
}

// ForceControllable
Expected<void> FlexivArm::set_force_control_axis(const ForceAxisConfig& config) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        // API: SetForceControlAxis(array<bool,6>, optional array<double,3>)
        robot_->SetForceControlAxis(config.enabled);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

Expected<void> FlexivArm::set_force_control_frame(const CartesianPose& frame) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        // API: SetForceControlFrame(CoordType, array<double,7>)
        robot_->SetForceControlFrame(flexiv::rdk::CoordType::WORLD, frame.pose);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

Expected<void> FlexivArm::set_passive_force_control(bool enable) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        robot_->SetPassiveForceControl(enable);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

// ImpedanceConfigurable
Expected<void> FlexivArm::set_cartesian_impedance(const ImpedanceParams& params) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        // API: SetCartesianImpedance(array<double,6>, array<double,6>)
        robot_->SetCartesianImpedance(params.stiffness, params.damping);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

Expected<void> FlexivArm::set_joint_impedance(const JointImpedanceParams& params) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        std::vector<double> stiffness(params.stiffness.begin(), params.stiffness.end());
        std::vector<double> damping(params.damping.begin(), params.damping.end());
        robot_->SetJointImpedance(stiffness, damping);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

// NullSpaceConfigurable
Expected<void> FlexivArm::set_null_space_posture(const JointState& posture) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        std::vector<double> pos(posture.positions.begin(), posture.positions.end());
        robot_->SetNullSpacePosture(pos);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

Expected<void> FlexivArm::set_null_space_objectives(const NullSpaceWeights& weights) {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        // API: SetNullSpaceObjectives(double linear_manip, double angular_manip, double ref_tracking)
        robot_->SetNullSpaceObjectives(weights.weights[0], weights.weights[1], weights.weights[2]);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

// DigitalIOControllable
Expected<void> FlexivArm::set_digital_outputs(
    const std::vector<std::pair<int, bool>>& outputs)
{
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        // API: SetDigitalOutputs(map<unsigned int, bool>)
        std::map<unsigned int, bool> out_map;
        for (const auto& [port, value] : outputs) {
            out_map[static_cast<unsigned int>(port)] = value;
        }
        robot_->SetDigitalOutputs(out_map);
        return {};
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

Expected<std::vector<bool>> FlexivArm::get_digital_inputs() {
    if (!robot_) return make_error(ErrorCode::NOT_CONNECTED, "FlexivArm: not connected");
    try {
        auto inputs = robot_->digital_inputs();
        return std::vector<bool>(inputs.begin(), inputs.end());
    } catch (const std::exception& e) {
        return make_error(ErrorCode::SDK_EXCEPTION, e.what());
    }
}

} // namespace cynlr::arm
