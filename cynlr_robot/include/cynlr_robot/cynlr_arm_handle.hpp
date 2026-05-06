#pragma once

#include <array>
#include <functional>
#include <optional>

#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr_robot {

// Facade exposed by CynlrRobotInterface to co-process nodes via CynlrArmRegistry.
// Controllers that need to issue commands or read state use this instead of holding
// a raw ArmInterface*. Only the hardware plugin populates it.
struct CynlrArmHandle {
    // State snapshot — calls through to the SDK (thread-safe)
    std::function<std::optional<cynlr::arm::ArmState>()> get_state;

    // --- Lifecycle ---
    std::function<cynlr::arm::Expected<void>()> connect;
    std::function<cynlr::arm::Expected<void>()> disconnect;
    std::function<cynlr::arm::Expected<void>()> enable;
    std::function<cynlr::arm::Expected<void>()> clear_fault;
    std::function<cynlr::arm::Expected<void>()> zero_ft_sensor;
    // stop_motion: aborts any in-flight NRT move (does not disconnect the arm).
    std::function<cynlr::arm::Expected<void>()> stop_motion;

    // --- Tool management ---
    std::function<cynlr::arm::Expected<void>(const cynlr::arm::ToolInfo&)> set_tool;
    std::function<cynlr::arm::Expected<void>(const cynlr::arm::ToolInfo&)> update_tool;
    std::function<cynlr::arm::Expected<cynlr::arm::ToolInfo>()>            get_tool;

    // --- NRT motion (NON-BLOCKING — returns immediately; poll is_motion_complete) ---
    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::CartesianTarget&,
        const cynlr::arm::MotionParams&)>  move_l;

    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::JointTarget&,
        const cynlr::arm::MotionParams&)>  move_j;

    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::CartesianTarget&,
        const cynlr::arm::MotionParams&)>  move_ptp;

    // Hybrid motion-force (ForceControllable required; null if unsupported)
    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::CartesianTarget&,
        const std::array<double, 6>&,
        const cynlr::arm::MotionParams&)>  move_hybrid_motion_force;

    std::function<std::optional<bool>()>   is_motion_complete;
    std::function<bool()>                  is_nrt_active;
};

} // namespace cynlr_robot
