#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

NavigateToPoseAction::NavigateToPoseAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("NavigateToPoseAction: Could not find 'node' on blackboard");
  }
  client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList NavigateToPoseAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus NavigateToPoseAction::onStart()
{
  goal_sent_ = false;
  result_received_ = false;
  goal_handle_ = nullptr;
  result_ = nullptr;

  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  step_idx_ = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx_);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx_);

  geometry_msgs::msg::PoseStamped target_pose;
  if (!getInput("pose", target_pose)) {
    reportFailure(config().blackboard, "Missing input port 'pose'", step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(),
      "NavigateToPoseAction: Sending navigation goal to coordinates (%.2f, %.2f)",
    target_pose.pose.position.x, target_pose.pose.position.y);

  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    reportFailure(config().blackboard, "NavigateToPose action server not available", step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  auto goal = NavigateToPose::Goal();
  goal.pose = target_pose;

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNav::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseAction: Goal was rejected by server");
        this->result_received_ = true;
        this->result_ = nullptr;
      } else {
        RCLCPP_INFO(node_->get_logger(), "NavigateToPoseAction: Goal accepted by server");
        this->goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNav::WrappedResult & result) {
      this->result_received_ = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        this->result_ = result.result;
      } else {
        this->result_ = nullptr;
        RCLCPP_ERROR(node_->get_logger(), "NavigateToPoseAction: Navigation failed with code %d",
          static_cast<int>(result.code));
      }
    };

  client_->async_send_goal(goal, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus NavigateToPoseAction::onRunning()
{
  if (result_received_) {
    if (result_) {
      RCLCPP_INFO(node_->get_logger(), "NavigateToPoseAction: Goal reached.");
      return BT::NodeStatus::SUCCESS;
    } else {
      reportFailure(config().blackboard, "Navigation goal was aborted or rejected", step_idx_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void NavigateToPoseAction::onHalted()
{
  if (goal_handle_ && !result_received_) {
    RCLCPP_INFO(node_->get_logger(), "NavigateToPoseAction: Halted. Canceling active goal.");
    client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace pick_place_orchestrator
