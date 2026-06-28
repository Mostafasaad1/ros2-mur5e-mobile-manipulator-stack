#ifndef PICK_PLACE_ORCHESTRATOR__BEHAVIOR_TREE_NODES_HPP_
#define PICK_PLACE_ORCHESTRATOR__BEHAVIOR_TREE_NODES_HPP_

#include <string>
#include <memory>
#include <future>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "base_placement_optimizer/action/optimize_placement.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace pick_place_orchestrator
{

// Helper to set phase and step on blackboard for feedback
inline void updateBlackboardFeedback(
  const BT::Blackboard::Ptr & blackboard,
  const std::string & phase, const std::string & step, int step_idx)
{
  if (blackboard) {
    blackboard->set("current_phase", phase);
    blackboard->set("current_step", step);
    blackboard->set("step_index", step_idx);
  }
}

// Helper to log and set failure reason on blackboard
inline void reportFailure(
  const BT::Blackboard::Ptr & blackboard, const std::string & reason,
  int step_idx)
{
  if (blackboard) {
    blackboard->set("failure_reason", reason);
    blackboard->set("failed_step_index", step_idx);
  }
  RCLCPP_ERROR(rclcpp::get_logger("BT_Node"), "Step %d Failed: %s", step_idx, reason.c_str());
}

// 0. TargetAcquisition BT Node
class TargetAcquisition : public BT::SyncActionNode
{
public:
  TargetAcquisition(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
};

// 1. OptimizePoseAction BT Node
class OptimizePoseAction : public BT::StatefulActionNode
{
public:
  OptimizePoseAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using OptimizePlacement = base_placement_optimizer::action::OptimizePlacement;
  using GoalHandleOptimize = rclcpp_action::ClientGoalHandle<OptimizePlacement>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<OptimizePlacement>::SharedPtr client_;
  std::shared_ptr<const OptimizePlacement::Goal> goal_;
  std::shared_ptr<GoalHandleOptimize> goal_handle_;
  std::shared_ptr<OptimizePlacement::Result> result_;
  bool goal_sent_{false};
  bool result_received_{false};
  int step_idx_{0};
};

// 2. NavigateToPoseAction BT Node
class NavigateToPoseAction : public BT::StatefulActionNode
{
public:
  NavigateToPoseAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  std::shared_ptr<GoalHandleNav> goal_handle_;
  std::shared_ptr<NavigateToPose::Result> result_;
  bool goal_sent_{false};
  bool result_received_{false};
  int step_idx_{0};
};

// 3. MoveArmAction BT Node (asynchronous MoveGroup wrapper)
class MoveArmAction : public BT::StatefulActionNode
{
public:
  MoveArmAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::future<moveit::core::MoveItErrorCode> future_;
  bool started_{false};
  int step_idx_{0};
};

// 8. VisualServoAction BT Node
class VisualServoAction : public BT::StatefulActionNode
{
public:
  VisualServoAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  std::future<moveit::core::MoveItErrorCode> future_;
  bool started_{false};
  int step_idx_{0};
};

// 4. AttachPayloadAction BT Node
class AttachPayloadAction : public BT::SyncActionNode
{
public:
  AttachPayloadAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

// 5. DetachPayloadAction BT Node
class DetachPayloadAction : public BT::SyncActionNode
{
public:
  DetachPayloadAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

// 6. GripperControlAction BT Node
class GripperControlAction : public BT::StatefulActionNode
{
public:
  GripperControlAction(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> gripper_group_;
  std::future<moveit::core::MoveItErrorCode> future_;
  bool started_{false};
  int step_idx_{0};
};

// 7. CheckPoseCondition BT Node
class CheckPoseCondition : public BT::ConditionNode
{
public:
  CheckPoseCondition(const std::string & name, const BT::NodeConfig & config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace pick_place_orchestrator

#endif  // PICK_PLACE_ORCHESTRATOR__BEHAVIOR_TREE_NODES_HPP_
