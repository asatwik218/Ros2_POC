#pragma once

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

    // Arm lifecycle services
    std::function<cynlr::arm::Expected<void>()>                              clear_fault;
    std::function<cynlr::arm::Expected<void>(const cynlr::arm::ToolInfo&)>   set_tool;
    std::function<cynlr::arm::Expected<void>()>                              zero_ft_sensor;

    // NRT motion — blocking, call from a non-RT thread only
    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::CartesianTarget&,
        const cynlr::arm::MotionParams&)>  move_l;

    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::JointTarget&,
        const cynlr::arm::MotionParams&)>  move_j;

    std::function<cynlr::arm::Expected<void>(
        const cynlr::arm::CartesianTarget&,
        const cynlr::arm::MotionParams&)>  move_ptp;

    std::function<std::optional<bool>()>                  is_motion_complete;
    std::function<cynlr::arm::Expected<void>()>           stop;
};

} // namespace cynlr_robot
