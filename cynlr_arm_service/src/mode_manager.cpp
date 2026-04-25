#include "cynlr_arm_service/mode_manager.hpp"

namespace cynlr::arm_service {

// Valid transitions:
//   IDLE      → CONNECTED
//   CONNECTED → IDLE (disconnect), ENABLED
//   ENABLED   → CONNECTED (stop), FAULT
//   any       → FAULT
//   FAULT     → CONNECTED (after clear_fault)
bool ModeManager::is_valid_transition(ArmMode from, ArmMode to) const {
    if (to == ArmMode::FAULT) return true;
    if (to == ArmMode::IDLE)  return true;

    switch (from) {
        case ArmMode::IDLE:
            return to == ArmMode::CONNECTED;
        case ArmMode::CONNECTED:
            return to == ArmMode::IDLE || to == ArmMode::ENABLED;
        case ArmMode::ENABLED:
            return to == ArmMode::CONNECTED;
        case ArmMode::FAULT:
            return to == ArmMode::CONNECTED;
        default:
            return false;
    }
}

bool ModeManager::transition(ArmMode target) {
    std::lock_guard lock(mutex_);
    if (!is_valid_transition(current_, target)) return false;
    ArmMode old = current_;
    current_ = target;
    if (callback_) callback_(old, target);
    return true;
}

ArmMode ModeManager::current() const {
    std::lock_guard lock(mutex_);
    return current_;
}

bool ModeManager::can_transition(ArmMode target) const {
    std::lock_guard lock(mutex_);
    return is_valid_transition(current_, target);
}

void ModeManager::set_callback(StateChangeCallback cb) {
    std::lock_guard lock(mutex_);
    callback_ = std::move(cb);
}

std::string ModeManager::mode_name(ArmMode mode) const {
    switch (mode) {
        case ArmMode::IDLE:      return "IDLE";
        case ArmMode::CONNECTED: return "CONNECTED";
        case ArmMode::ENABLED:   return "ENABLED";
        case ArmMode::FAULT:     return "FAULT";
        default:                 return "UNKNOWN";
    }
}

} // namespace cynlr::arm_service
