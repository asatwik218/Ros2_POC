#pragma once
#include "cynlr_arm_core/export.hpp"
#include "cynlr_arm_core/error.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr::arm {

class CYNLR_ARM_CORE_EXPORT NullSpaceConfigurable {
public:
    virtual ~NullSpaceConfigurable() = default;

    // Set reference joint posture for null-space control
    virtual Expected<void> set_null_space_posture(const JointState& posture) = 0;

    // Set per-joint weights for null-space optimisation
    virtual Expected<void> set_null_space_objectives(const NullSpaceWeights& weights) = 0;
};

} // namespace cynlr::arm
