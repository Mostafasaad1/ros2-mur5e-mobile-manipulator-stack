#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

MoveArmAction::MoveArmAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("MoveArmAction: Could not find 'node' on blackboard");
  }
  // Initialize MoveGroupInterface
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,
      "robot_arm");
}

BT::PortsList MoveArmAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<std::string>("named_pose"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus MoveArmAction::onStart()
{
  started_ = false;

  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  step_idx_ = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx_);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx_);

  std::string named_pose;
  geometry_msgs::msg::PoseStamped target_pose;
  bool has_named = getInput("named_pose", named_pose).has_value() && !named_pose.empty();
  bool has_target = getInput("target_pose", target_pose).has_value();

  if (!has_named && !has_target) {
    reportFailure(config().blackboard, "Neither 'named_pose' nor 'target_pose' provided",
        step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  // Configure target
  if (has_named) {
    RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Planning to named pose: '%s'",
        named_pose.c_str());
    if (!move_group_->setNamedTarget(named_pose)) {
      reportFailure(config().blackboard, "Failed to set named target: '" + named_pose + "'",
          step_idx_);
      return BT::NodeStatus::FAILURE;
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Planning to target pose in frame: %s",
      target_pose.header.frame_id.c_str());
    if (!move_group_->setPoseTarget(target_pose)) {
      reportFailure(config().blackboard, "Failed to set pose target", step_idx_);
      return BT::NodeStatus::FAILURE;
    }
  }

  // Start execution in background thread to avoid blocking BT tick
  future_ = std::async(std::launch::async, [this]() {
        return move_group_->move();
  });
  started_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveArmAction::onRunning()
{
  if (started_) {
    if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      moveit::core::MoveItErrorCode result = future_.get();
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Motion completed successfully.");
        return BT::NodeStatus::SUCCESS;
      } else {
        reportFailure(config().blackboard,
            "Arm movement failed with error code: " + std::to_string(result.val), step_idx_);
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  return BT::NodeStatus::RUNNING;
}

void MoveArmAction::onHalted()
{
  if (move_group_) {
    RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Halted. Stopping active arm movement.");
    move_group_->stop();
  }
}

}  // namespace pick_place_orchestrator
