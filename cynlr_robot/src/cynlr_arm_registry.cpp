#include "cynlr_robot/cynlr_arm_registry.hpp"

namespace cynlr_robot {

CynlrArmRegistry& CynlrArmRegistry::instance()
{
    static CynlrArmRegistry inst;
    return inst;
}

void CynlrArmRegistry::register_arm(
    const std::string& prefix, std::shared_ptr<CynlrArmHandle> handle)
{
    std::lock_guard lock(mutex_);
    handles_[prefix] = std::move(handle);
}

void CynlrArmRegistry::unregister_arm(const std::string& prefix)
{
    std::lock_guard lock(mutex_);
    handles_.erase(prefix);
}

std::shared_ptr<CynlrArmHandle> CynlrArmRegistry::get(const std::string& prefix)
{
    std::lock_guard lock(mutex_);
    auto it = handles_.find(prefix);
    return it != handles_.end() ? it->second : nullptr;
}

} // namespace cynlr_robot
