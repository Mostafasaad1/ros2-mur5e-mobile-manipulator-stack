#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

OptimizePoseAction::OptimizePoseAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("OptimizePoseAction: Could not find 'node' on blackboard");
  }
  client_ = rclcpp_action::create_client<OptimizePlacement>(node_, "optimize_placement");
}

BT::PortsList OptimizePoseAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("pose"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("optimized_pose")
  };
}

BT::NodeStatus OptimizePoseAction::onStart()
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
      "OptimizePoseAction: Requesting optimization for target pose in frame %s",
    target_pose.header.frame_id.c_str());

  if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
    reportFailure(config().blackboard, "OptimizePlacement action server not available", step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  auto goal = OptimizePlacement::Goal();
  goal.target_pose = target_pose;
  goal_ = std::make_shared<const OptimizePlacement::Goal>(goal);

  auto send_goal_options = rclcpp_action::Client<OptimizePlacement>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleOptimize::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "OptimizePoseAction: Goal was rejected by server");
        this->result_received_ = true;
        this->result_ = nullptr;
      } else {
        RCLCPP_INFO(node_->get_logger(), "OptimizePoseAction: Goal accepted by server");
        this->goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleOptimize::WrappedResult & result) {
      this->result_received_ = true;
      this->result_ = result.result;
      if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_ERROR(node_->get_logger(), "OptimizePoseAction: Action failed with code %d",
          static_cast<int>(result.code));
      }
    };

  client_->async_send_goal(*goal_, send_goal_options);
  goal_sent_ = true;

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus OptimizePoseAction::onRunning()
{
  if (result_received_) {
    if (result_ && result_->success) {
      RCLCPP_INFO(node_->get_logger(), "OptimizePoseAction: Optimization succeeded.");
      setOutput("optimized_pose", result_->base_pose);
      return BT::NodeStatus::SUCCESS;
    } else {
      std::string reason = (result_ ? result_->error_reason : "Goal rejected or aborted");
      reportFailure(config().blackboard, "Optimization failed: " + reason, step_idx_);
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void OptimizePoseAction::onHalted()
{
  if (goal_handle_ && !result_received_) {
    RCLCPP_INFO(node_->get_logger(), "OptimizePoseAction: Halted. Canceling active goal.");
    client_->async_cancel_goal(goal_handle_);
  }
}

}  // namespace pick_place_orchestrator
