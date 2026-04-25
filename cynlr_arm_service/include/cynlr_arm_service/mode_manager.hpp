#pragma once
#include <functional>
#include <mutex>
#include <string>

namespace cynlr::arm_service {

enum class ArmMode {
    IDLE,
    CONNECTED,
    ENABLED,
    FAULT,
};

class ModeManager {
public:
    using StateChangeCallback = std::function<void(ArmMode old_mode, ArmMode new_mode)>;

    // Attempt a transition. Returns true if successful, false if invalid.
    bool transition(ArmMode target);

    ArmMode current() const;
    bool can_transition(ArmMode target) const;
    void set_callback(StateChangeCallback cb);
    std::string mode_name(ArmMode mode) const;

private:
    mutable std::mutex mutex_;
    ArmMode current_{ArmMode::IDLE};
    StateChangeCallback callback_;

    bool is_valid_transition(ArmMode from, ArmMode to) const;
};

} // namespace cynlr::arm_service
