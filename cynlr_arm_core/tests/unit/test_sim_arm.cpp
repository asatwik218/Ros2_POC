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

// ---------------------------------------------------------------------------
// Async NRT contract
// ---------------------------------------------------------------------------

TEST(SimArm, MoveJIsNonBlocking) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    JointTarget target;
    target.positions[0] = 1.0;

    auto t0 = std::chrono::steady_clock::now();
    auto result = arm->move_j(target, MotionParams{});
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);

    ASSERT_TRUE(result.has_value());
    // SimArm move_delay is 50ms; a non-blocking call should return in < 10ms
    EXPECT_LT(elapsed.count(), 30);
    // nrt_active must be true immediately after the call
    EXPECT_TRUE(arm->nrt_active());
}

TEST(SimArm, NrtActiveTogglesAroundMoveJ) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    EXPECT_FALSE(arm->nrt_active());

    JointTarget target;
    (void)arm->move_j(target, MotionParams{});
    EXPECT_TRUE(arm->nrt_active());

    // Poll until complete
    for (int i = 0; i < 100; ++i) {
        auto done = arm->is_motion_complete();
        ASSERT_TRUE(done.has_value());
        if (*done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_FALSE(arm->nrt_active());
}

TEST(SimArm, IsMotionCompleteReturnsFalseBeforeDelay) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    JointTarget target;
    (void)arm->move_j(target, MotionParams{});

    // Immediately after issuing: should not be complete yet
    auto done = arm->is_motion_complete();
    ASSERT_TRUE(done.has_value());
    EXPECT_FALSE(*done);
}

TEST(SimArm, StopAbortsMoveJ) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    JointTarget target;
    target.positions[3] = 1.5;
    (void)arm->move_j(target, MotionParams{});
    EXPECT_TRUE(arm->nrt_active());

    // Abort before the 50ms delay expires
    (void)arm->stop();

    EXPECT_FALSE(arm->nrt_active());
    auto done = arm->is_motion_complete();
    ASSERT_TRUE(done.has_value());
    EXPECT_TRUE(*done);  // stopped = complete
}

// ---------------------------------------------------------------------------
// Tool get / set / update
// ---------------------------------------------------------------------------

TEST(SimArm, ToolRoundtrip) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    ToolInfo t1;
    t1.mass_kg = 0.5;
    t1.com = {0.0, 0.0, 0.05};
    t1.tcp_pose[3] = 1.0;  // identity quat

    ASSERT_TRUE(arm->set_tool(t1).has_value());
    auto got = arm->get_tool();
    ASSERT_TRUE(got.has_value());
    EXPECT_NEAR(got->mass_kg, 0.5, 1e-9);
    EXPECT_NEAR(got->com[2], 0.05, 1e-9);
}

TEST(SimArm, UpdateToolOverwritesPrev) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    ToolInfo t1; t1.mass_kg = 0.5;
    ToolInfo t2; t2.mass_kg = 1.2;

    (void)arm->set_tool(t1);
    (void)arm->update_tool(t2);

    auto got = arm->get_tool();
    ASSERT_TRUE(got.has_value());
    EXPECT_NEAR(got->mass_kg, 1.2, 1e-9);
}

// ---------------------------------------------------------------------------
// Hybrid motion-force (sim stub: pose moves, wrench ignored)
// ---------------------------------------------------------------------------

TEST(SimArm, MoveHybridMotionForceIsNonBlocking) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();

    auto* fc = dynamic_cast<cynlr::arm::ForceControllable*>(arm.get());
    ASSERT_NE(fc, nullptr);

    CartesianTarget pose;
    pose.pose[0] = 0.4; pose.pose[3] = 1.0;
    std::array<double, 6> wrench = {0, 0, -5.0, 0, 0, 0};

    auto t0 = std::chrono::steady_clock::now();
    auto result = fc->move_hybrid_motion_force(pose, wrench, MotionParams{});
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);

    ASSERT_TRUE(result.has_value());
    EXPECT_LT(elapsed.count(), 30);
    EXPECT_TRUE(arm->nrt_active());

    // Poll to completion
    for (int i = 0; i < 100; ++i) {
        auto done = arm->is_motion_complete();
        if (done.has_value() && *done) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    EXPECT_FALSE(arm->nrt_active());
    auto state = arm->get_state();
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->tcp_pose[0], 0.4, 1e-6);
}

// ---------------------------------------------------------------------------
// intended_rt_mode via set_intended_rt_mode
// ---------------------------------------------------------------------------

TEST(SimArm, IntendedRtModeRecordedWithoutModeSwitch) {
    auto arm = create_arm(sim_config());
    (void)arm->connect(sim_config());
    (void)arm->enable();
    // Just record the mode — SimArm has no real mode switching
    arm->set_intended_rt_mode(StreamMode::CARTESIAN_MOTION_FORCE);
    // If we get here without crash the setter at least accepted the call
    SUCCEED();
}
