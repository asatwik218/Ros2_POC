#pragma once
#include "cynlr_arm_core/export.hpp"
#include "cynlr_arm_core/error.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr::arm {

class CYNLR_ARM_CORE_EXPORT ImpedanceConfigurable {
public:
    virtual ~ImpedanceConfigurable() = default;

    // Set Cartesian impedance (stiffness + damping per axis)
    virtual Expected<void> set_cartesian_impedance(const ImpedanceParams& params) = 0;

    // Set joint-space impedance
    virtual Expected<void> set_joint_impedance(const JointImpedanceParams& params) = 0;
};

} // namespace cynlr::arm
