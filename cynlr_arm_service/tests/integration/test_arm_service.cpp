// Integration tests for cynlr_arm_service (supervision-only role) using SimArm backend.
// Requires a running arm_service_node launched with sim.yaml config.
// Run via: colcon test --packages-select cynlr_arm_service

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "cynlr_arm_interfaces/srv/connect.hpp"
#include "cynlr_arm_interfaces/srv/trigger.hpp"
#include "cynlr_arm_interfaces/srv/set_tool.hpp"
#include "cynlr_arm_interfaces/srv/get_digital_inputs.hpp"
#include "cynlr_arm_interfaces/msg/operational_status.hpp"

using namespace std::chrono_literals;

static const char* kNs = "/test_arm";

// ---------------------------------------------------------------------------
// Helper: synchronous service call with timeout
// ---------------------------------------------------------------------------
template<typename SrvT>
typename SrvT::Response::SharedPtr call_service(
    rclcpp::Node::SharedPtr node,
    const std::string& name,
    typename SrvT::Request::SharedPtr req,
    std::chrono::seconds timeout = 5s)
{
    auto client = node->create_client<SrvT>(name);
    if (!client->wait_for_service(timeout)) {
        return nullptr;
    }
    auto future = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, future, timeout)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        return nullptr;
    }
    return future.get();
}

// ---------------------------------------------------------------------------
// Fixture: spins up a client node for each test
// ---------------------------------------------------------------------------
class ArmServiceTest : public ::testing::Test {
protected:
    void SetUp() override {
        node_ = rclcpp::Node::make_shared("arm_service_test_client");
    }

    void TearDown() override {
        // Best-effort disconnect so state is clean for the next test.
        using Trigger = cynlr_arm_interfaces::srv::Trigger;
        auto req = std::make_shared<Trigger::Request>();
        call_service<Trigger>(node_, std::string(kNs) + "/disconnect", req, 2s);
    }

    // Convenience: connect → enable sequence
    bool connect_and_enable() {
        using Connect = cynlr_arm_interfaces::srv::Connect;
        using Trigger = cynlr_arm_interfaces::srv::Trigger;

        auto conn_req = std::make_shared<Connect::Request>();
        conn_req->vendor = "sim";
        conn_req->num_joints = 7;
        auto conn_res = call_service<Connect>(node_, std::string(kNs) + "/connect", conn_req);
        if (!conn_res || !conn_res->success) return false;

        auto en_req = std::make_shared<Trigger::Request>();
        auto en_res = call_service<Trigger>(node_, std::string(kNs) + "/enable", en_req);
        return en_res && en_res->success;
    }

    rclcpp::Node::SharedPtr node_;
};

// ---------------------------------------------------------------------------
// Test 1: Mode-gated transitions are rejected when preconditions not met
// ---------------------------------------------------------------------------
TEST_F(ArmServiceTest, InvalidTransitionsRejected) {
    using Trigger = cynlr_arm_interfaces::srv::Trigger;

    // Enable without connecting first — should fail (mode guard in handle_enable)
    auto en_req = std::make_shared<Trigger::Request>();
    auto en_res = call_service<Trigger>(node_, std::string(kNs) + "/enable", en_req);
    ASSERT_NE(en_res, nullptr) << "enable service not available";
    EXPECT_FALSE(en_res->success) << "Enable should fail when not connected";

    // clear_fault without being in FAULT mode — should fail (mode guard in handle_clear_fault)
    auto cf_req = std::make_shared<Trigger::Request>();
    auto cf_res = call_service<Trigger>(node_, std::string(kNs) + "/clear_fault", cf_req);
    ASSERT_NE(cf_res, nullptr) << "clear_fault service not available";
    EXPECT_FALSE(cf_res->success) << "clear_fault should fail when not in FAULT";
}

// ---------------------------------------------------------------------------
// Test 2: Connect and Disconnect cycle
// ---------------------------------------------------------------------------
TEST_F(ArmServiceTest, ConnectAndDisconnect) {
    using Connect = cynlr_arm_interfaces::srv::Connect;
    using Trigger = cynlr_arm_interfaces::srv::Trigger;

    auto conn_req = std::make_shared<Connect::Request>();
    conn_req->vendor = "sim";
    conn_req->num_joints = 7;
    auto conn_res = call_service<Connect>(node_, std::string(kNs) + "/connect", conn_req);
    ASSERT_NE(conn_res, nullptr) << "connect service not available";
    EXPECT_TRUE(conn_res->success) << conn_res->message;

    auto disc_req = std::make_shared<Trigger::Request>();
    auto disc_res = call_service<Trigger>(node_, std::string(kNs) + "/disconnect", disc_req);
    ASSERT_NE(disc_res, nullptr) << "disconnect service not available";
    EXPECT_TRUE(disc_res->success) << disc_res->message;

    // Enable after disconnect — should fail (back to IDLE)
    auto en_req = std::make_shared<Trigger::Request>();
    auto en_res = call_service<Trigger>(node_, std::string(kNs) + "/enable", en_req);
    ASSERT_NE(en_res, nullptr);
    EXPECT_FALSE(en_res->success) << "Enable should fail after disconnect";
}

// ---------------------------------------------------------------------------
// Test 3: Connect → Enable → Stop succeeds
// ---------------------------------------------------------------------------
TEST_F(ArmServiceTest, ConnectEnableAndStop) {
    ASSERT_TRUE(connect_and_enable()) << "connect+enable failed";

    using Trigger = cynlr_arm_interfaces::srv::Trigger;
    auto stop_req = std::make_shared<Trigger::Request>();
    auto stop_res = call_service<Trigger>(node_, std::string(kNs) + "/stop", stop_req);
    ASSERT_NE(stop_res, nullptr) << "stop service not available";
    EXPECT_TRUE(stop_res->success) << stop_res->message;
}

// ---------------------------------------------------------------------------
// Test 4: Supervision services respond correctly after connect+enable
// ---------------------------------------------------------------------------
TEST_F(ArmServiceTest, SupervisionServicesWork) {
    ASSERT_TRUE(connect_and_enable()) << "connect+enable failed";

    using SetTool = cynlr_arm_interfaces::srv::SetTool;
    using Trigger = cynlr_arm_interfaces::srv::Trigger;
    using GetDigIn = cynlr_arm_interfaces::srv::GetDigitalInputs;

    // set_tool — supply a valid zero-mass tool
    auto tool_req = std::make_shared<SetTool::Request>();
    tool_req->mass_kg = 0.0;
    tool_req->com = {0.0, 0.0, 0.0};
    tool_req->inertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    tool_req->tcp_pose = {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
    auto tool_res = call_service<SetTool>(node_, std::string(kNs) + "/set_tool", tool_req);
    ASSERT_NE(tool_res, nullptr) << "set_tool service not available";
    EXPECT_TRUE(tool_res->success) << tool_res->message;

    // zero_ft_sensor (service is named "zero_ft_sensor" in arm_service_node)
    auto zft_req = std::make_shared<Trigger::Request>();
    auto zft_res = call_service<Trigger>(node_, std::string(kNs) + "/zero_ft_sensor", zft_req);
    ASSERT_NE(zft_res, nullptr) << "zero_ft_sensor service not available";
    EXPECT_TRUE(zft_res->success) << zft_res->message;

    // get_digital_inputs — SimArm may not support DIO; check service is reachable
    auto gdi_req = std::make_shared<GetDigIn::Request>();
    auto gdi_res = call_service<GetDigIn>(node_, std::string(kNs) + "/get_digital_inputs",
                                          gdi_req);
    ASSERT_NE(gdi_res, nullptr) << "get_digital_inputs service not available";
}

// ---------------------------------------------------------------------------
// Test 5: OperationalStatus topic publishes ENABLED mode after connect+enable
// ---------------------------------------------------------------------------
TEST_F(ArmServiceTest, StatusTopicPublishes) {
    using OperationalStatus = cynlr_arm_interfaces::msg::OperationalStatus;
    bool received = false;
    uint8_t received_mode = 255;

    // Subscribe before connect+enable so we get the live ENABLED message,
    // not just the transient_local cache from a previous IDLE state.
    auto sub = node_->create_subscription<OperationalStatus>(
        std::string(kNs) + "/status",
        rclcpp::QoS(10).reliable().transient_local(),
        [&](const OperationalStatus::SharedPtr msg) {
            received_mode = msg->mode;
            if (msg->mode == OperationalStatus::MODE_ENABLED) {
                received = true;
            }
        });

    ASSERT_TRUE(connect_and_enable()) << "connect+enable failed";

    // Timer fires every 200ms; wait up to 3s for an ENABLED status message.
    auto deadline = std::chrono::steady_clock::now() + 3s;
    while (!received && std::chrono::steady_clock::now() < deadline) {
        rclcpp::spin_some(node_);
        std::this_thread::sleep_for(50ms);
    }

    EXPECT_TRUE(received) << "No ENABLED OperationalStatus message received within 3s";
    EXPECT_EQ(received_mode, OperationalStatus::MODE_ENABLED)
        << "Expected MODE_ENABLED (2), got: " << static_cast<int>(received_mode);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    ::testing::InitGoogleTest(&argc, argv);
    int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
