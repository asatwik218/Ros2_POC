#pragma once
#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/capabilities/digital_io_controllable.hpp"
#include "cynlr_arm_core/capabilities/force_controllable.hpp"
#include "cynlr_arm_core/capabilities/impedance_configurable.hpp"
#include "cynlr_arm_core/capabilities/null_space_configurable.hpp"

// Forward declare to avoid pulling flexiv headers into consumers
namespace flexiv::rdk { class Robot; }

namespace cynlr::arm {

class FlexivArm : public ArmInterface,
                   public ForceControllable,
                   public ImpedanceConfigurable,
                   public NullSpaceConfigurable,
                   public DigitalIOControllable {
public:
    FlexivArm();
    ~FlexivArm() override;

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
    bool nrt_active() const override { return nrt_active_.load(); }
    Expected<void> start_streaming(StreamMode mode) override;
    Expected<void> stream_command(const StreamCommand& cmd) override;
    Expected<void> stop_streaming() override;
    void set_intended_rt_mode(StreamMode mode) override { intended_rt_mode_ = mode; }
    Expected<void> set_tool(const ToolInfo& tool) override;
    Expected<void> update_tool(const ToolInfo& tool) override { return set_tool(tool); }
    Expected<ToolInfo> get_tool() const override;
    Expected<void> zero_ft_sensor() override;
    std::vector<std::string> supported_features() const override;

    // ForceControllable
    Expected<void> set_force_control_axis(const ForceAxisConfig& config) override;
    Expected<void> set_force_control_frame(const CartesianPose& frame) override;
    Expected<void> set_passive_force_control(bool enable) override;
    Expected<void> move_hybrid_motion_force(
        const CartesianTarget& pose_target,
        const std::array<double, 6>& wrench_setpoint,
        const MotionParams& params) override;

    // ImpedanceConfigurable
    Expected<void> set_cartesian_impedance(const ImpedanceParams& params) override;
    Expected<void> set_joint_impedance(const JointImpedanceParams& params) override;

    // NullSpaceConfigurable
    Expected<void> set_null_space_posture(const JointState& posture) override;
    Expected<void> set_null_space_objectives(const NullSpaceWeights& weights) override;

    // DigitalIOControllable
    Expected<void> set_digital_outputs(
        const std::vector<std::pair<int, bool>>& outputs) override;
    Expected<std::vector<bool>> get_digital_inputs() override;

private:
    std::unique_ptr<flexiv::rdk::Robot> robot_;
    ArmConfig config_;
    // RT mode the HW interface wants restored after any NRT move completes.
    // Updated by set_intended_rt_mode() from CynlrRobotInterface::perform_command_mode_switch.
    StreamMode intended_rt_mode_{StreamMode::JOINT_POSITION};
    // Set true while an NRT motion is executing so stream_command skips RT streaming
    std::atomic<bool> nrt_active_{false};
    // Timestamp of the most recent NRT Send call — used for the start-up grace period in
    // is_motion_complete() so we don't declare completion before the arm begins moving.
    std::chrono::steady_clock::time_point nrt_send_time_{};
    static constexpr std::chrono::milliseconds kNrtStartDelay{200};
    // Cached tool for get_tool()
    ToolInfo cached_tool_{};
    static constexpr const char* kManagedToolName = "CynlrTool";

    // Switch back to intended_rt_mode_ after NRT motion finishes (used inside is_motion_complete).
    void restore_rt_mode_locked();
};

} // namespace cynlr::arm
