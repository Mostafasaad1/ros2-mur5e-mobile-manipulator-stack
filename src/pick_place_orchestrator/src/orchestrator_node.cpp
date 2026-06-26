#include "pick_place_orchestrator/orchestrator_node.hpp"
#include <thread>

namespace pick_place_orchestrator
{

OrchestratorNode::OrchestratorNode(const rclcpp::NodeOptions & options)
: Node("pick_place_orchestrator_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing OrchestratorNode");

  // Declare parameter for Behavior Tree XML path
  bt_xml_path_ = this->declare_parameter<std::string>("bt_xml_path", "");

  // Create action server
  using namespace std::placeholders;
  action_server_ = rclcpp_action::create_server<PickPlaceMission>(
    this,
    "pick_place_mission",
    std::bind(&OrchestratorNode::handle_goal, this, _1, _2),
    std::bind(&OrchestratorNode::handle_cancel, this, _1),
    std::bind(&OrchestratorNode::handle_accepted, this, _1)
  );
}

OrchestratorNode::~OrchestratorNode()
{
}

rclcpp_action::GoalResponse OrchestratorNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const PickPlaceMission::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received PickPlaceMission goal request");

  // T017: Validate pick_pose but allow empty place_pose
  if (goal->pick_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Rejecting goal request: 'pick_pose' has empty frame_id");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Accepting goal request. Pick frame: '%s', Place frame: '%s'",
    goal->pick_pose.header.frame_id.c_str(),
    goal->place_pose.header.frame_id.empty() ? "None (Pick-only mode)" :
      goal->place_pose.header.frame_id.c_str());

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrchestratorNode::handle_cancel(
  const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel active PickPlaceMission goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void OrchestratorNode::handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  // Spawn background thread to execute the mission asynchronously so ROS executor is not blocked
  std::thread([this, goal_handle]() {
      this->execute(goal_handle);
  }).detach();
}

void OrchestratorNode::execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
{
  std::lock_guard<std::mutex> lock(bt_mutex_);

  RCLCPP_INFO(this->get_logger(), "Executing PickPlaceMission goal");

  // Lazily construct bt_engine_ to safely pass shared_from_this()
  if (!bt_engine_) {
    bt_engine_ = std::make_unique<BehaviorTreeEngine>(shared_from_this());
  }

  // Get goal coordinates
  auto goal = goal_handle->get_goal();

  // Create feedback callback
  auto feedback_cb = [goal_handle](const std::string & phase, const std::string & step, int idx) {
      auto feedback = std::make_shared<PickPlaceMission::Feedback>();
      feedback->current_phase = phase;
      feedback->current_step = step;
      feedback->step_index = idx;
      goal_handle->publish_feedback(feedback);
    };

  // Resolve parameter if not set
  std::string xml_path = bt_xml_path_;
  if (xml_path.empty()) {
    // Attempt to read parameter again in case it was updated
    this->get_parameter("bt_xml_path", xml_path);
  }

  if (xml_path.empty()) {
    RCLCPP_ERROR(this->get_logger(), "bt_xml_path parameter is empty! Cannot run Behavior Tree.");
    auto result = std::make_shared<PickPlaceMission::Result>();
    result->success = false;
    result->failure_reason = "XML_PATH_EMPTY";
    result->failed_step_index = 0;
    goal_handle->abort(result);
    return;
  }

  // Start Behavior Tree execution
  if (!bt_engine_->run(xml_path, goal->pick_pose, goal->place_pose, feedback_cb)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to run Behavior Tree.");
    auto result = std::make_shared<PickPlaceMission::Result>();
    result->success = false;
    result->failure_reason = bt_engine_->getFailureReason();
    result->failed_step_index = bt_engine_->getFailedStepIndex();
    goal_handle->abort(result);
    return;
  }

  // Polling loop
  rclcpp::Rate rate(10);
  while (rclcpp::ok() && bt_engine_->isRunning()) {
    if (goal_handle->is_canceling()) {
      RCLCPP_INFO(this->get_logger(), "Mission goal is canceling. Halting BT...");
      bt_engine_->halt();
      auto result = std::make_shared<PickPlaceMission::Result>();
      result->success = false;
      result->failure_reason = "CANCELED";
      result->failed_step_index = bt_engine_->getFailedStepIndex();
      goal_handle->canceled(result);
      return;
    }
    rate.sleep();
  }

  // Handle final execution status
  auto status = bt_engine_->getStatus();
  auto result = std::make_shared<PickPlaceMission::Result>();

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(this->get_logger(), "Behavior Tree completed successfully.");
    result->success = true;
    result->failure_reason = "";
    result->failed_step_index = -1;
    goal_handle->succeed(result);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Behavior Tree execution failed.");
    result->success = false;
    result->failure_reason = bt_engine_->getFailureReason();
    result->failed_step_index = bt_engine_->getFailedStepIndex();
    goal_handle->abort(result);
  }
}

}  // namespace pick_place_orchestrator

// Register as a component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pick_place_orchestrator::OrchestratorNode)
