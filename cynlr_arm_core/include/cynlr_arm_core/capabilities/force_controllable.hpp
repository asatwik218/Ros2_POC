#pragma once
#include <array>
#include "cynlr_arm_core/export.hpp"
#include "cynlr_arm_core/error.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr::arm {

class CYNLR_ARM_CORE_EXPORT ForceControllable {
public:
    virtual ~ForceControllable() = default;

    // Set which Cartesian axes use force control vs motion control
    virtual Expected<void> set_force_control_axis(const ForceAxisConfig& config) = 0;

    // Set the reference frame for force control
    virtual Expected<void> set_force_control_frame(const CartesianPose& frame) = 0;

    // Enable passive (open-loop) or active (closed-loop) force tracking
    virtual Expected<void> set_passive_force_control(bool enable) = 0;

    // NRT hybrid motion-force move (NON-BLOCKING — same async contract as ArmInterface::move_l).
    // Moves the TCP toward pose_target while applying wrench_setpoint on the force-controlled axes.
    // Axis selection and reference frame must be configured beforehand via set_force_control_axis /
    // set_force_control_frame / set_passive_force_control.
    // Poll ArmInterface::is_motion_complete() to detect completion; call ArmInterface::stop() to abort.
    virtual Expected<void> move_hybrid_motion_force(
        const CartesianTarget& pose_target,
        const std::array<double, 6>& wrench_setpoint,
        const MotionParams& params) = 0;
};

} // namespace cynlr::arm
