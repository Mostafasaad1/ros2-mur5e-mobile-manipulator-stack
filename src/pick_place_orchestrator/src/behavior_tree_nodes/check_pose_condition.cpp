#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

CheckPoseCondition::CheckPoseCondition(const std::string & name, const BT::NodeConfig & config)
: BT::ConditionNode(name, config)
{
}

BT::PortsList CheckPoseCondition::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "PoseStamped to validate")
  };
}

BT::NodeStatus CheckPoseCondition::tick()
{
  geometry_msgs::msg::PoseStamped pose;
  if (!getInput("pose", pose)) {
    RCLCPP_WARN(rclcpp::get_logger("BT_Node"),
        "CheckPoseCondition: Input port 'pose' not connected.");
    return BT::NodeStatus::FAILURE;
  }

  // If the frame ID is empty, we treat it as an unprovided/empty pose
  if (pose.header.frame_id.empty()) {
    RCLCPP_INFO(rclcpp::get_logger("BT_Node"), "CheckPoseCondition: Pose is empty/unprovided.");
    return BT::NodeStatus::FAILURE;
  }

  // If coordinates are all zero and frame ID is "none" or similar, we can also treat it as empty
  if (pose.header.frame_id == "none" || pose.header.frame_id == "empty") {
    RCLCPP_INFO(rclcpp::get_logger("BT_Node"),
        "CheckPoseCondition: Pose is explicitly set to empty.");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("BT_Node"),
      "CheckPoseCondition: Pose is valid and provided (frame: '%s').",
    pose.header.frame_id.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pick_place_orchestrator
