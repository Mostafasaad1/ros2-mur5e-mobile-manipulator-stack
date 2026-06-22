#include "pick_place_orchestrator/behavior_tree_engine.hpp"
#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

#include <chrono>

namespace pick_place_orchestrator
{

BehaviorTreeEngine::BehaviorTreeEngine(rclcpp::Node::SharedPtr node)
: node_(node)
{
  registerNodes();
}

BehaviorTreeEngine::~BehaviorTreeEngine()
{
  halt();
}

void BehaviorTreeEngine::registerNodes()
{
  // Note: Since BT_REGISTER_NODES macro registers them when plugins are loaded,
  // we can also register them programmatically here to ensure they are available
  // when compiled directly into the binary without dynamic library loading issues.
  factory_.registerNodeType<TargetAcquisition>("TargetAcquisition");
  factory_.registerNodeType<OptimizePoseAction>("OptimizePose");
  factory_.registerNodeType<NavigateToPoseAction>("NavigateToPose");
  factory_.registerNodeType<MoveArmAction>("MoveArm");
  factory_.registerNodeType<AttachPayloadAction>("AttachPayload");
  factory_.registerNodeType<DetachPayloadAction>("DetachPayload");
  factory_.registerNodeType<GripperControlAction>("GripperControl");
  factory_.registerNodeType<CheckPoseCondition>("CheckPoseCondition");
}

bool BehaviorTreeEngine::run(
  const std::string & xml_path,
  const geometry_msgs::msg::PoseStamped & pick_pose,
  const geometry_msgs::msg::PoseStamped & place_pose,
  FeedbackCallback feedback_cb)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (is_running_) {
    RCLCPP_ERROR(node_->get_logger(), "BehaviorTreeEngine: Already running a tree!");
    return false;
  }

  // Clear previous results
  failure_reason_ = "";
  failed_step_index_ = -1;
  status_ = BT::NodeStatus::IDLE;
  should_halt_ = false;

  // Create blackboard
  blackboard_ = BT::Blackboard::create();
  blackboard_->set("node", node_);
  blackboard_->set("pick_pose", pick_pose);
  blackboard_->set("place_pose", place_pose);

  blackboard_->set("current_phase", std::string("IDLE"));
  blackboard_->set("current_step", std::string("IDLE"));
  blackboard_->set("step_index", 0);
  blackboard_->set("failure_reason", std::string(""));
  blackboard_->set("failed_step_index", -1);

  // Load tree
  try {
    tree_ = factory_.createTreeFromFile(xml_path, blackboard_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "BehaviorTreeEngine: Failed to load BT XML: %s", e.what());
    failure_reason_ = std::string("XML_LOAD_FAILED: ") + e.what();
    failed_step_index_ = 0;
    return false;
  }

  is_running_ = true;
  thread_ = std::thread(&BehaviorTreeEngine::executeLoop, this, xml_path, feedback_cb);
  return true;
}

void BehaviorTreeEngine::halt()
{
  should_halt_ = true;
  if (thread_.joinable()) {
    thread_.join();
  }
  is_running_ = false;
}

bool BehaviorTreeEngine::isRunning() const
{
  return is_running_;
}

BT::NodeStatus BehaviorTreeEngine::getStatus() const
{
  return status_;
}

std::string BehaviorTreeEngine::getFailureReason() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return failure_reason_;
}

int BehaviorTreeEngine::getFailedStepIndex() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return failed_step_index_;
}

void BehaviorTreeEngine::executeLoop(const std::string & xml_path, FeedbackCallback feedback_cb)
{
  RCLCPP_INFO(node_->get_logger(), "BehaviorTreeEngine: Starting execution loop for tree: %s",
      xml_path.c_str());

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  auto rate = std::chrono::milliseconds(100);

  while (rclcpp::ok() && !should_halt_ && status == BT::NodeStatus::RUNNING) {
    try {
      status = tree_.tickExactlyOnce();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "BehaviorTreeEngine: Exception during BT tick: %s",
          e.what());
      status = BT::NodeStatus::FAILURE;

      std::lock_guard<std::mutex> lock(mutex_);
      failure_reason_ = std::string("EXCEPTION_DURING_TICK: ") + e.what();
      failed_step_index_ = blackboard_->get<int>("step_index");
    }

    status_ = status;

    // Propagate blackboard values to action feedback
    std::string phase = blackboard_->get<std::string>("current_phase");
    std::string step = blackboard_->get<std::string>("current_step");
    int idx = blackboard_->get<int>("step_index");
    if (feedback_cb) {
      feedback_cb(phase, step, idx);
    }

    std::this_thread::sleep_for(rate);
  }

  if (should_halt_) {
    RCLCPP_INFO(node_->get_logger(), "BehaviorTreeEngine: Halting active BT.");
    tree_.haltTree();
    status_ = BT::NodeStatus::FAILURE;

    std::lock_guard<std::mutex> lock(mutex_);
    failure_reason_ = "PREEMPTED";
    failed_step_index_ = blackboard_->get<int>("step_index");
  }

  if (status_ == BT::NodeStatus::FAILURE) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::string reason = blackboard_->get<std::string>("failure_reason");
    if (!reason.empty()) {
      failure_reason_ = reason;
    } else if (failure_reason_.empty()) {
      failure_reason_ = "BT_EXECUTION_FAILURE";
    }
    failed_step_index_ = blackboard_->get<int>("failed_step_index");
    if (failed_step_index_ == -1) {
      failed_step_index_ = blackboard_->get<int>("step_index");
    }
  }

  RCLCPP_INFO(node_->get_logger(), "BehaviorTreeEngine: Execution loop finished with status: %s",
    BT::toStr(status_.load()).c_str());

  is_running_ = false;
}

}  // namespace pick_place_orchestrator
