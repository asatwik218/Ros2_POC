#pragma once
#include <string>
#include <vector>
#include "cynlr_arm_core/export.hpp"
#include "cynlr_arm_core/error.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr::arm {

class CYNLR_ARM_CORE_EXPORT ArmInterface {
public:
    virtual ~ArmInterface() = default;

    // Non-copyable, non-movable — owns hardware connection
    ArmInterface() = default;
    ArmInterface(const ArmInterface&) = delete;
    ArmInterface& operator=(const ArmInterface&) = delete;
    ArmInterface(ArmInterface&&) = delete;
    ArmInterface& operator=(ArmInterface&&) = delete;

    // --- Lifecycle ---
    virtual Expected<void> connect(const ArmConfig& config) = 0;
    virtual Expected<void> disconnect() = 0;
    virtual Expected<void> enable() = 0;
    virtual Expected<void> stop() = 0;

    // --- State ---
    virtual Expected<ArmState> get_state() = 0;
    virtual bool is_connected() const = 0;
    virtual bool has_fault() const = 0;
    virtual Expected<void> clear_fault() = 0;
    virtual Expected<void> run_auto_recovery() = 0;

    // --- Discrete motion (NRT, blocking — completes before returning) ---
    virtual Expected<void> move_l(const CartesianTarget& target, const MotionParams& params) = 0;
    virtual Expected<void> move_j(const JointTarget& target, const MotionParams& params) = 0;
    virtual Expected<void> move_ptp(const CartesianTarget& target, const MotionParams& params) = 0;
    // Returns true when current primitive/motion is complete
    virtual Expected<bool> is_motion_complete() = 0;
    // Returns true while an NRT motion is in progress (RT streaming is paused)
    virtual bool nrt_active() const { return false; }

    // --- RT streaming (1kHz) ---
    virtual Expected<void> start_streaming(StreamMode mode) = 0;
    virtual Expected<void> stream_command(const StreamCommand& cmd) = 0;
    virtual Expected<void> stop_streaming() = 0;

    // --- Tool ---
    virtual Expected<void> set_tool(const ToolInfo& tool) = 0;
    virtual Expected<void> zero_ft_sensor() = 0;

    // --- Capability introspection ---
    // Returns list of capability names this implementation supports.
    // Consumers may also use dynamic_cast for compile-time capability checks.
    virtual std::vector<std::string> supported_features() const = 0;
};

} // namespace cynlr::arm
