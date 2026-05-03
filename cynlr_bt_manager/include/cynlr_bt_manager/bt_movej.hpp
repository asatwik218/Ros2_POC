#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <string>

#include <behaviortree_ros2/bt_action_node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cynlr_arm_interfaces/action/move_j.hpp>

using MoveJoints = cynlr_arm_interfaces::action::MoveJ;

// MoveJointsAction
//   Inherits BT::RosActionNode which handles:
//     - action client lifecycle
//     - goal sending / cancellation
//     - mapping rclcpp_action states to BT NodeStatus
class MoveJointsAction
    : public BT::RosActionNode<MoveJoints>
{
public:
  // ---- constructor (required signature for BT::RosActionNode) ----
  MoveJointsAction(const std::string& name,
                   const BT::NodeConfig& config,
                   const BT::RosNodeParams& params);

  // ---- port declaration ----
  static BT::PortsList providedPorts();

  // ---- called once when the node transitions to RUNNING ----
  //      Populate the goal from input ports.
  bool setGoal(Goal& goal) override;

  // ---- called on every feedback message while the action is running ----
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  // ---- called once when the action server sends a result ----
  BT::NodeStatus onResultReceived(const WrappedResult& wr) override;

  // ---- called if the action server becomes unreachable mid-flight ----
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};