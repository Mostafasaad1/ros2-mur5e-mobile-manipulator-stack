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
    BT::InputPort<double>("z_offset"),
    BT::InputPort<double>("x_offset"),
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
    double z_offset = 0.0;
    getInput("z_offset", z_offset);
    double x_offset = 0.0;
    getInput("x_offset", x_offset);

    double yaw = 0.0;
    geometry_msgs::msg::PoseStamped base_pose;
    bool has_base_pose = false;
    if (phase == "PICK_PHASE") {
      has_base_pose = config().blackboard->get("optimized_pick_base_pose", base_pose);
    } else if (phase == "PLACE_PHASE") {
      has_base_pose = config().blackboard->get("optimized_place_base_pose", base_pose);
    }

    if (has_base_pose) {
      double base_x = base_pose.pose.position.x;
      double base_y = base_pose.pose.position.y;
      double dx = target_pose.pose.position.x - base_x;
      double dy = target_pose.pose.position.y - base_y;
      yaw = std::atan2(dy, dx);
      RCLCPP_INFO(node_->get_logger(),
          "MoveArmAction: Aligned grasp yaw to %.2f rad based on base pos (%.2f, %.2f) and target (%.2f, %.2f)",
        yaw, base_x, base_y, target_pose.pose.position.x, target_pose.pose.position.y);
    } else {
      RCLCPP_WARN(node_->get_logger(),
          "MoveArmAction: Could not find optimized base pose on blackboard for phase %s. Using default yaw = 0.0",
        phase.c_str());
    }

    // Set horizontal grasp orientation (90 deg pitch rotated by yaw around Z)
    double half_yaw = yaw * 0.5;
    target_pose.pose.orientation.x = -0.70710678 * std::sin(half_yaw);
    target_pose.pose.orientation.y = 0.70710678 * std::cos(half_yaw);
    target_pose.pose.orientation.z = 0.70710678 * std::sin(half_yaw);
    target_pose.pose.orientation.w = 0.70710678 * std::cos(half_yaw);

    if (x_offset != 0.0) {
      target_pose.pose.position.x -= x_offset * std::cos(yaw);
      target_pose.pose.position.y -= x_offset * std::sin(yaw);
    }
    if (z_offset != 0.0) {
      target_pose.pose.position.z += z_offset;
    }

    RCLCPP_INFO(node_->get_logger(),
        "MoveArmAction: Planning to target pose in frame: %s (x: %.3f, y: %.3f, z: %.3f, offset_x: %.3f, offset_z: %.3f)",
      target_pose.header.frame_id.c_str(), target_pose.pose.position.x, target_pose.pose.position.y,
        target_pose.pose.position.z, x_offset, z_offset);

    if (!move_group_->setPoseTarget(target_pose)) {
      reportFailure(config().blackboard, "Failed to set pose target", step_idx_);
      return BT::NodeStatus::FAILURE;
    }
  }

  // Start execution in background thread to avoid blocking BT tick
  future_ = std::async(std::launch::async, [this]() {
        move_group_->setMaxVelocityScalingFactor(0.8);
        move_group_->setMaxAccelerationScalingFactor(0.8);
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
  if (future_.valid()) {
    future_.wait();
  }
}

}  // namespace pick_place_orchestrator
