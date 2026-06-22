#ifndef PICK_PLACE_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_
#define PICK_PLACE_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_

#include <memory>
#include <string>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pick_place_orchestrator/action/pick_place_mission.hpp"
#include "pick_place_orchestrator/behavior_tree_engine.hpp"

namespace pick_place_orchestrator
{

class OrchestratorNode : public rclcpp::Node
{
public:
  using PickPlaceMission = pick_place_orchestrator::action::PickPlaceMission;
  using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlaceMission>;

  explicit OrchestratorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~OrchestratorNode();

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PickPlaceMission::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle);

  rclcpp_action::Server<PickPlaceMission>::SharedPtr action_server_;
  std::unique_ptr<BehaviorTreeEngine> bt_engine_;
  std::string bt_xml_path_;
  std::mutex bt_mutex_;
};

}  // namespace pick_place_orchestrator

#endif  // PICK_PLACE_ORCHESTRATOR__ORCHESTRATOR_NODE_HPP_
