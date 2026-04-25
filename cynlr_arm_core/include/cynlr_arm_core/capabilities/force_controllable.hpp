#pragma once
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
};

} // namespace cynlr::arm
