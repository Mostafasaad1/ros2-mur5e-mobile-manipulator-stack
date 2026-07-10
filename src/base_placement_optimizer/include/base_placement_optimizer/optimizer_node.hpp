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
#include "moveit/planning_scene_monitor/planning_scene_monitor.hpp"

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
  std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene_monitor_;
  moveit::core::RobotModelConstPtr robot_model_;

  // Costmap components
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
  std::mutex costmap_mutex_;

  void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  bool is_candidate_safe(
    double x, double y, double theta,
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
  // Maximum distance (m) from the robot's current pose a candidate may be.
  // Candidates beyond this threshold are pruned before scoring to prevent
  // the planner from receiving goals outside the navigable map region.
  double max_nav_distance_;
  std::string robot_base_frame_;
  // Minimum XY distance a candidate must have from the target object.
  // Enforces clearance from the table inflation zone even when the table
  // is not yet visible in the costmap.
  double target_clearance_radius_;
  // Grasp offsets — must match the MoveArm BT XML x_offset / z_offset values
  // so that IK validation targets the actual grasp pose, not the raw object pose.
  double grasp_x_offset_;
  double grasp_z_offset_;

};

}  // namespace base_placement_optimizer

#endif  // BASE_PLACEMENT_OPTIMIZER__OPTIMIZER_NODE_HPP_
