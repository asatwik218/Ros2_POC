#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "cynlr_robot/cynlr_arm_handle.hpp"

namespace cynlr_robot {

// Process-global registry mapping arm prefix → CynlrArmHandle.
// CynlrRobotInterface::on_activate() registers; on_deactivate() unregisters.
// CynlrArmNode polls get() until the handle is available.
// All methods are thread-safe.
class CynlrArmRegistry {
public:
    static CynlrArmRegistry& instance();

    void register_arm(const std::string& prefix, std::shared_ptr<CynlrArmHandle> handle);
    void unregister_arm(const std::string& prefix);
    std::shared_ptr<CynlrArmHandle> get(const std::string& prefix);

private:
    std::mutex mutex_;
    std::map<std::string, std::shared_ptr<CynlrArmHandle>> handles_;
};

} // namespace cynlr_robot
