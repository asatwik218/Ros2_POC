#pragma once
#include <memory>
#include "cynlr_arm_core/export.hpp"
#include "cynlr_arm_core/arm_interface.hpp"
#include "cynlr_arm_core/types.hpp"

namespace cynlr::arm {

// Creates an ArmInterface implementation based on config.vendor.
// Returns nullptr if vendor is unknown.
// Supported vendors: "flexiv", "sim"
CYNLR_ARM_CORE_EXPORT std::unique_ptr<ArmInterface> create_arm(const ArmConfig& config);

} // namespace cynlr::arm
