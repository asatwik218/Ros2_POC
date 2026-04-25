#include <gtest/gtest.h>
#include "cynlr_arm_core/arm_factory.hpp"
#include "cynlr_arm_core/capabilities/force_controllable.hpp"
#include "cynlr_arm_core/capabilities/null_space_configurable.hpp"

using namespace cynlr::arm;

TEST(Factory, CreatesSimArm) {
    ArmConfig config{"sim", "", "", 7, {}};
    auto arm = create_arm(config);
    ASSERT_NE(arm, nullptr);
}

TEST(Factory, CreatesFlexivArm) {
    ArmConfig config{"flexiv", "", "", 7, {}};
    auto arm = create_arm(config);
    ASSERT_NE(arm, nullptr);
}

TEST(Factory, ReturnsNullForUnknownVendor) {
    ArmConfig config{"unknown_robot", "", "", 7, {}};
    auto arm = create_arm(config);
    EXPECT_EQ(arm, nullptr);
}

TEST(Factory, SimArmHasForceControllableCapability) {
    ArmConfig config{"sim", "", "", 7, {}};
    auto arm = create_arm(config);
    ASSERT_NE(arm, nullptr);
    auto* fc = dynamic_cast<ForceControllable*>(arm.get());
    EXPECT_NE(fc, nullptr);
}

TEST(Factory, SimArmHasNullSpaceCapability) {
    ArmConfig config{"sim", "", "", 7, {}};
    auto arm = create_arm(config);
    ASSERT_NE(arm, nullptr);
    auto* ns = dynamic_cast<NullSpaceConfigurable*>(arm.get());
    EXPECT_NE(ns, nullptr);
}

TEST(Factory, SimArmSupportedFeaturesNotEmpty) {
    ArmConfig config{"sim", "", "", 7, {}};
    auto arm = create_arm(config);
    ASSERT_NE(arm, nullptr);
    auto features = arm->supported_features();
    EXPECT_FALSE(features.empty());
}
