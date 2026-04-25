#include "cynlr_arm_core/arm_factory.hpp"
#include "sim/sim_arm.hpp"
#include "flexiv/flexiv_arm.hpp"

namespace cynlr::arm {

std::unique_ptr<ArmInterface> create_arm(const ArmConfig& config) {
    if (config.vendor == "flexiv") {
        return std::make_unique<FlexivArm>();
    }
    if (config.vendor == "sim") {
        return std::make_unique<SimArm>();
    }
    return nullptr;
}

} // namespace cynlr::arm
