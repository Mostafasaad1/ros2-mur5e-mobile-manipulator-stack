// Copyright 2026 Mobile Manipulator Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BASE_PLACEMENT_OPTIMIZER__OPTIMIZER_NODE_HPP_
#define BASE_PLACEMENT_OPTIMIZER__OPTIMIZER_NODE_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "base_placement_optimizer/action/optimize_placement.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "moveit/robot_model_loader/robot_model_loader.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"

namespace base_placement_optimizer
{

class BasePlacementOptimizerNode : public rclcpp::Node
{
public:
  using OptimizePlacement = base_placement_optimizer::action::OptimizePlacement;
  using GoalHandleOptimizePlacement = rclcpp_action::ServerGoalHandle<OptimizePlacement>;

  explicit BasePlacementOptimizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~BasePlacementOptimizerNode();

private:
  // Action Server implementation
  rclcpp_action::Server<OptimizePlacement>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const OptimizePlacement::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleOptimizePlacement> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleOptimizePlacement> goal_handle);

  void execute(const std::shared_ptr<GoalHandleOptimizePlacement> goal_handle);

  // TF2 components
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // MoveIt components
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr robot_model_;

  // Costmap components
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool is_candidate_safe(
    double x, double y,
    const nav_msgs::msg::OccupancyGrid::SharedPtr & costmap) const;
  double compute_manipulability(
    const moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * jmg) const;

  // Parameters
  std::string map_frame_;
  std::string planning_group_;
  std::string costmap_topic_;
  double reach_radius_;
  int angular_samples_;
  double ik_timeout_;
  double alpha_;

};

}  // namespace base_placement_optimizer

#endif  // BASE_PLACEMENT_OPTIMIZER__OPTIMIZER_NODE_HPP_
