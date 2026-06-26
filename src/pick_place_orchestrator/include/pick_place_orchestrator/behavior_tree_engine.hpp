#ifndef PICK_PLACE_ORCHESTRATOR__BEHAVIOR_TREE_ENGINE_HPP_
#define PICK_PLACE_ORCHESTRATOR__BEHAVIOR_TREE_ENGINE_HPP_

#include <string>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pick_place_orchestrator
{

class BehaviorTreeEngine
{
public:
  using FeedbackCallback = std::function<void(const std::string & phase, const std::string & step,
      int step_idx)>;

  explicit BehaviorTreeEngine(rclcpp::Node::SharedPtr node);
  ~BehaviorTreeEngine();

  // Register all our custom BT nodes in the factory
  void registerNodes();

  // Initialize and run the BT
  bool run(
    const std::string & xml_path,
    const geometry_msgs::msg::PoseStamped & pick_pose,
    const geometry_msgs::msg::PoseStamped & place_pose,
    FeedbackCallback feedback_cb);

  // Stop/Halt the running tree
  void halt();

  // Check if BT is currently running
  bool isRunning() const;

  // Get the last execution status
  BT::NodeStatus getStatus() const;

  // Get the current failure reason if any
  std::string getFailureReason() const;

  // Get the failed step index if any
  int getFailedStepIndex() const;

private:
  void executeLoop(const std::string & xml_path, FeedbackCallback feedback_cb);

  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard_;

  std::thread thread_;
  std::atomic<bool> is_running_{false};
  std::atomic<bool> should_halt_{false};
  std::atomic<BT::NodeStatus> status_{BT::NodeStatus::IDLE};
  mutable std::mutex mutex_;

  std::string failure_reason_{""};
  int failed_step_index_{-1};
};

}  // namespace pick_place_orchestrator

#endif  // PICK_PLACE_ORCHESTRATOR__BEHAVIOR_TREE_ENGINE_HPP_
