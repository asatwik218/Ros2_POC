#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/groot2_publisher.h>
#include <behaviortree_ros2/plugins.hpp>          // BT::RosNodeParams, registerRosNode

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_ros2/bt_action_node.hpp>

#include <fstream>
#include <thread>
#include <chrono>

#include "cynlr_bt_manager/bt_movej.hpp"   // your MoveJointsAction header

using namespace std::chrono_literals;

namespace BT {
template <>
inline std::array<double, 7> convertFromString(StringView str)
{
  std::array<double, 7> out{};
  std::string s(str);
  std::replace(s.begin(), s.end(), ',', ' ');
  std::istringstream ss(s);
  for (auto& v : out) {
    if (!(ss >> v))
      throw RuntimeError("MoveJoints: expected 7 doubles, got fewer");
  }
  return out;
}
} 

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    // ── 1. Create the ROS2 node that the BT action clients live on ──────────
    auto node = std::make_shared<rclcpp::Node>("bt_executor");

    // ── 2. Executor + spin thread ────────────────────────────────────────────
    //    Must be separate from the tick loop — same reason as controller_manager:
    //    tick() blocks waiting for callbacks that only fire when executor spins
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);

    std::thread spin_thread([&executor]() {
        executor->spin();
    });

    // ── 3. Build the BehaviorTree ────────────────────────────────────────────
    BT::BehaviorTreeFactory factory;

    // RosNodeParams bundles the node + action server name for RosActionNode
    BT::RosNodeParams params;
    params.nh = node;
    params.default_port_value = "move_joints";   // action server name

    // Register your node — RosActionNode needs the params overload
    factory.registerNodeType<MoveJointsAction>("MoveJoints", params);

    // Load XML
    std::string pkg_path =
        ament_index_cpp::get_package_share_directory("cynlr_bt_manager");
    factory.registerBehaviorTreeFromFile(pkg_path + "/tree.xml");
    auto tree = factory.createTree("Untitled");

    // ── 4. Groot2 monitor ────────────────────────────────────────────────────
    BT::Groot2Publisher publisher(tree, 5555);

    // ── 5. Tick loop — runs on main thread ───────────────────────────────────
    //    tickOnce() is non-blocking:
    //      - if action is in flight it returns RUNNING immediately
    //      - onFeedback/onResultReceived fire from the spin thread via callbacks
    //        and update the BT node state safely
    rclcpp::WallRate loop_rate(100ms);   // tick at 10 Hz — adjust to your needs

    while (rclcpp::ok())
    {
        BT::NodeStatus status = tree.tickOnce();

        if (status == BT::NodeStatus::SUCCESS)
        {
            RCLCPP_INFO(node->get_logger(), "Tree finished: SUCCESS");
            break;
        }
        if (status == BT::NodeStatus::FAILURE)
        {
            RCLCPP_ERROR(node->get_logger(), "Tree finished: FAILURE");
            break;
        }

        loop_rate.sleep();
    }

    // ── 6. Clean shutdown ────────────────────────────────────────────────────
    executor->cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}