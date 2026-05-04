#include "cynlr_bt_manager/bt_movej.hpp"

MoveJointsAction::MoveJointsAction(const std::string& name,
                                   const BT::NodeConfig& config,
                                   const BT::RosNodeParams& params)
    : BT::RosActionNode<MoveJoints>(name, config, params)
{}

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

BT::PortsList MoveJointsAction::providedPorts()
{
  return {
    // Inputs
    BT::InputPort<std::string>("robot_sn"),
    BT::InputPort<std::array<double,7>>("target_positions"),
    BT::InputPort<double>(
        "max_joint_vel",
        0.0,
        "Max joint velocity rad/s (0 = use server default)"),
    BT::InputPort<double>(
        "max_joint_acc",
        0.0,
        "Max joint acceleration rad/s² (0 = use server default)"),

    // Outputs (written on completion)
    BT::OutputPort<bool>(
        "success",
        "True if the action server reported success"),

    // Live feedback (updated on every feedback message)
    BT::OutputPort<std::array<double,7>>(
        "current_positions",
        "Live joint positions during motion"),
  };
}

bool MoveJointsAction::setGoal(Goal& goal)
{
  auto positions = getInput<std::array<double,7>>("target_positions");
  if (!positions)
  {
    RCLCPP_ERROR(logger(), "[MoveJointsAction] Missing required port 'target_positions': %s",
                 positions.error().c_str());
    return false;
  }

  // Copy array → repeated field
  for (size_t i = 0; i < 7; ++i)
    goal.target_positions[i] = positions.value()[i];

  goal.max_joint_vel = getInput<double>("max_joint_vel").value_or(0.0);
  goal.max_joint_acc = getInput<double>("max_joint_acc").value_or(0.0);

  RCLCPP_INFO(logger(), "[MoveJointsAction] Sending goal — "
              "vel_limit=%.3f  acc_limit=%.3f",
              goal.max_joint_vel, goal.max_joint_acc);
  return true;
}

BT::NodeStatus MoveJointsAction::onFeedback(const std::shared_ptr<const Feedback> feedback)
{
  std::array<double,7> cur{};
  for (size_t i = 0; i < 7; ++i)
  {
    cur[i] = feedback->current_positions[i];
  }
  setOutput("current_positions", cur);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveJointsAction::onResultReceived(const WrappedResult& wr)
{
  const auto& res = wr.result;

  setOutput("success", res->success);

  if (wr.code == rclcpp_action::ResultCode::SUCCEEDED && res->success)
  {
    RCLCPP_INFO(logger(), "[MoveJointsAction] SUCCESS — %s", res->message.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_WARN(logger(), "[MoveJointsAction] FAILURE — code=%d  msg=%s",
              static_cast<int>(wr.code), res->message.c_str());
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus MoveJointsAction::onFailure(BT::ActionNodeErrorCode error)
{
  RCLCPP_ERROR(logger(), "[MoveJointsAction] Action error: %s",
               BT::toStr(error));
  return BT::NodeStatus::FAILURE;
}