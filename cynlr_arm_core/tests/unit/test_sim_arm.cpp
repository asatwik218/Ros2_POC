#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "cynlr_arm_core/arm_factory.hpp"
#include "cynlr_arm_core/types.hpp"
// Internal header for test helpers (inject_fault, etc.)
#include "sim/sim_arm.hpp"

using namespace cynlr::arm;

static ArmConfig sim_config() {
    return ArmConfig{"sim", "", "", 7, {}};
}

TEST(SimArm, ConnectSucceeds) {
    auto arm = create_arm(sim_config());
    ASSERT_NE(arm, nullptr);
    auto result = arm->connect(sim_config());
    EXPECT_TRUE(result.has_value());
    EXPECT_TRUE(arm->is_connected());
}

TEST(SimArm, EnableRequiresConnect) {
    auto arm = create_arm(sim_config());
    auto result = arm->enable();
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error().code, ErrorCode::NOT_CONNECTED);
}

TEST(SimArm, EnableSucceedsAfterConnect) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    auto result = arm->enable();
    EXPECT_TRUE(result.has_value());
}

TEST(SimArm, GetStateAfterConnect) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();
    auto state = arm->get_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_FALSE(state->fault);
    EXPECT_TRUE(state->operational);
}

TEST(SimArm, FaultInjectionAndClear) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    EXPECT_FALSE(arm->has_fault());

    auto* sim = dynamic_cast<cynlr::arm::SimArm*>(arm.get());
    ASSERT_NE(sim, nullptr);
    sim->inject_fault();

    EXPECT_TRUE(arm->has_fault());
    auto state = arm->get_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_TRUE(state->fault);

    EXPECT_TRUE(arm->clear_fault().has_value());
    EXPECT_FALSE(arm->has_fault());
}

TEST(SimArm, MoveJUpdatesJointState) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    JointTarget target;
    target.positions[0] = 0.5;
    target.positions[1] = -0.3;
    MotionParams params;

    auto result = arm->move_j(target, params);
    ASSERT_TRUE(result.has_value());

    auto state = arm->get_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->joint_positions[0], 0.5, 1e-6);
    EXPECT_NEAR(state->joint_positions[1], -0.3, 1e-6);
}

TEST(SimArm, MoveLUpdatesTcpPose) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    CartesianTarget target;
    target.pose[0] = 0.5;  // x
    target.pose[1] = 0.2;  // y
    target.pose[2] = 0.3;  // z
    target.pose[3] = 1.0;  // qw
    MotionParams params;

    auto result = arm->move_l(target, params);
    ASSERT_TRUE(result.has_value());

    auto state = arm->get_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->tcp_pose[0], 0.5, 1e-6);
}

TEST(SimArm, StreamingJointPosition) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    ASSERT_TRUE(arm->start_streaming(StreamMode::JOINT_POSITION).has_value());

    StreamCommand cmd;
    cmd.mode = StreamMode::JOINT_POSITION;
    cmd.joint_position[0] = 1.2;
    cmd.joint_position[3] = -0.7;

    ASSERT_TRUE(arm->stream_command(cmd).has_value());

    auto state = arm->get_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->joint_positions[0], 1.2, 1e-6);
    EXPECT_NEAR(state->joint_positions[3], -0.7, 1e-6);
}

TEST(SimArm, StreamCommandFailsWhenNotStreaming) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();
    // Don't call start_streaming
    StreamCommand cmd;
    auto result = arm->stream_command(cmd);
    EXPECT_FALSE(result.has_value());
}

TEST(SimArm, StopStreamingWorks) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();
    (void)arm->start_streaming(StreamMode::JOINT_POSITION);
    ASSERT_TRUE(arm->stop_streaming().has_value());
    // After stopping, stream_command should fail
    StreamCommand cmd;
    auto result = arm->stream_command(cmd);
    EXPECT_FALSE(result.has_value());
}

TEST(SimArm, IsMotionCompleteAfterMove) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    JointTarget target;
    (void)arm->move_j(target, MotionParams{});

    // SimArm completes after a short delay — poll until done (max 500ms)
    bool complete = false;
    for (int i = 0; i < 50 && !complete; ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto result = arm->is_motion_complete();
        ASSERT_TRUE(result.has_value());
        complete = *result;
    }
    EXPECT_TRUE(complete);
}

TEST(SimArm, ZeroFtSensorClearsReadings) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();
    EXPECT_TRUE(arm->zero_ft_sensor().has_value());
}

TEST(SimArm, DisconnectPreventsGetState) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->disconnect();
    EXPECT_FALSE(arm->is_connected());
    auto state = arm->get_state();
    EXPECT_FALSE(state.has_value());
    EXPECT_EQ(state.error().code, ErrorCode::NOT_CONNECTED);
}
