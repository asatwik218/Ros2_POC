#pragma once
#include <utility>
#include <vector>
#include "cynlr_arm_core/export.hpp"
#include "cynlr_arm_core/error.hpp"

namespace cynlr::arm {

class CYNLR_ARM_CORE_EXPORT DigitalIOControllable {
public:
    virtual ~DigitalIOControllable() = default;

    // Set digital output ports. Each pair: {port_index (0-17), value}
    virtual Expected<void> set_digital_outputs(
        const std::vector<std::pair<int, bool>>& outputs) = 0;

    // Read all 18 digital input ports
    virtual Expected<std::vector<bool>> get_digital_inputs() = 0;
};

} // namespace cynlr::arm
