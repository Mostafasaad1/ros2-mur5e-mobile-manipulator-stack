#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

TargetAcquisition::TargetAcquisition(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("TargetAcquisition: Could not find 'node' on blackboard");
  }
}

BT::PortsList TargetAcquisition::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose", "Pose to acquire"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus TargetAcquisition::tick()
{
  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  int step_idx = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx);

  geometry_msgs::msg::PoseStamped pose;
  if (!getInput("pose", pose) || pose.header.frame_id.empty()) {
    reportFailure(config().blackboard, "Target pose not specified or invalid", step_idx);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
      "TargetAcquisition: Successfully acquired target pose in frame '%s' (x: %.2f, y: %.2f, z: %.2f)",
    pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pick_place_orchestrator
