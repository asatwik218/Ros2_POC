#pragma once
#include <atomic>
#include <chrono>
#include <mutex>
#include <random>
#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/capabilities/force_controllable.hpp"
#include "cynlr_arm_core/capabilities/null_space_configurable.hpp"

namespace cynlr::arm {

class CYNLR_ARM_CORE_EXPORT SimArm : public ArmInterface,
               public ForceControllable,
               public NullSpaceConfigurable {
public:
    // ArmInterface
    Expected<void> connect(const ArmConfig& config) override;
    Expected<void> disconnect() override;
    Expected<void> enable() override;
    Expected<void> stop() override;
    Expected<ArmState> get_state() override;
    bool is_connected() const override;
    bool has_fault() const override;
    Expected<void> clear_fault() override;
    Expected<void> run_auto_recovery() override;
    Expected<void> move_l(const CartesianTarget& target, const MotionParams& params) override;
    Expected<void> move_j(const JointTarget& target, const MotionParams& params) override;
    Expected<void> move_ptp(const CartesianTarget& target, const MotionParams& params) override;
    Expected<bool> is_motion_complete() override;
    Expected<void> start_streaming(StreamMode mode) override;
    Expected<void> stream_command(const StreamCommand& cmd) override;
    Expected<void> stop_streaming() override;
    Expected<void> set_tool(const ToolInfo& tool) override;
    Expected<void> zero_ft_sensor() override;
    std::vector<std::string> supported_features() const override;

    // ForceControllable
    Expected<void> set_force_control_axis(const ForceAxisConfig& config) override;
    Expected<void> set_force_control_frame(const CartesianPose& frame) override;
    Expected<void> set_passive_force_control(bool enable) override;

    // NullSpaceConfigurable
    Expected<void> set_null_space_posture(const JointState& posture) override;
    Expected<void> set_null_space_objectives(const NullSpaceWeights& weights) override;

    // Test helpers
    void inject_fault();
    void set_noise_stddev(double stddev);
    void set_latency_ms(double ms);

private:
    mutable std::mutex state_mutex_;
    ArmState state_{};
    ArmConfig config_{};
    bool connected_{false};
    bool enabled_{false};
    bool streaming_{false};
    bool has_fault_{false};
    bool motion_complete_{true};
    std::chrono::steady_clock::time_point motion_start_{};
    std::chrono::milliseconds motion_delay_{50};
    double noise_stddev_{0.0};
    double latency_ms_{0.0};
    StreamMode stream_mode_{StreamMode::JOINT_POSITION};
    std::mt19937 rng_{std::random_device{}()};
};

} // namespace cynlr::arm
