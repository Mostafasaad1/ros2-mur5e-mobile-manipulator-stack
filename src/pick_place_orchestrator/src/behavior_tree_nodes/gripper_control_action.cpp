#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

GripperControlAction::GripperControlAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("GripperControlAction: Could not find 'node' on blackboard");
  }
  gripper_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "hand");
}

BT::PortsList GripperControlAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("command"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus GripperControlAction::onStart()
{
  started_ = false;

  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  step_idx_ = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx_);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx_);

  std::string command;
  if (!getInput("command", command) || command.empty()) {
    reportFailure(config().blackboard, "Missing input port 'command'", step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  // Normalize command to the MoveIt SRDF names
  std::string target_state = command;
  if (command == "close" || command == "closed") {
    target_state = "closed";
  } else if (command == "open") {
    target_state = "open";
  } else if (command == "pre_grasp") {
    target_state = "pre_grasp";
  }

  RCLCPP_INFO(node_->get_logger(), "GripperControlAction: Setting gripper target state to '%s'",
      target_state.c_str());

  if (!gripper_group_->setNamedTarget(target_state)) {
    reportFailure(config().blackboard,
        "Failed to set gripper target state to: '" + target_state + "'", step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  future_ = std::async(std::launch::async, [this]() {
        return gripper_group_->move();
  });
  started_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GripperControlAction::onRunning()
{
  if (started_) {
    if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      moveit::core::MoveItErrorCode result = future_.get();
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(),
            "GripperControlAction: Gripper motion completed successfully.");
        return BT::NodeStatus::SUCCESS;
      } else {
        reportFailure(config().blackboard,
            "Gripper motion failed with error code: " + std::to_string(result.val), step_idx_);
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  return BT::NodeStatus::RUNNING;
}

void GripperControlAction::onHalted()
{
  if (gripper_group_) {
    RCLCPP_INFO(node_->get_logger(), "GripperControlAction: Halted. Stopping gripper movement.");
    gripper_group_->stop();
  }
}

}  // namespace pick_place_orchestrator
