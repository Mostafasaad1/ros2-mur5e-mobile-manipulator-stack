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

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "base_placement_optimizer/optimizer_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace base_placement_optimizer
{

BasePlacementOptimizerNode::BasePlacementOptimizerNode(const rclcpp::NodeOptions & options)
: Node("base_placement_optimizer_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing BasePlacementOptimizerNode");

  // Declare parameters
  map_frame_ = this->declare_parameter("map_frame", "map");
  planning_group_ = this->declare_parameter("planning_group", "robot_arm");
  costmap_topic_ = this->declare_parameter("costmap_topic", "/global_costmap/costmap");
  // reach_radius: nominal distance from target object to robot base center.
  // The sweep now covers [reach_radius-0.20, reach_radius+0.60] so that
  // IK solutions at larger stand-offs (needed to clear table inflation) are
  // also considered.
  reach_radius_ = this->declare_parameter("reach_radius", 0.80);
  angular_samples_ = this->declare_parameter("angular_samples", 16);
  ik_timeout_ = this->declare_parameter("ik_timeout", 0.050);  // Raised from 0.010 to match kinematics.yaml budget
  alpha_ = this->declare_parameter("alpha", 0.5);
  // max_nav_distance: candidates farther than this from the robot's current
  // pose are discarded.  12 m is safely within any of our test maps while
  // still allowing cross-room navigation (~7 m pick-to-place distance).
  max_nav_distance_ = this->declare_parameter("max_nav_distance", 12.0);
  robot_base_frame_ = this->declare_parameter("robot_base_frame", "base_footprint");
  // target_clearance_radius: minimum XY distance a candidate base pose must
  // maintain from the TARGET OBJECT in the map frame.  This guards against
  // choosing a pose that will fall inside the costmap inflation zone once
  // the table becomes visible to the LIDAR (table is often beyond sensor
  // range during optimization).  Formula:
  //   table_half_width (≈0.25) + robot_front_length (0.495) + safety_margin (0.05) ≈ 0.80
  // Default 0.80 m ensures no physical collision while remaining within reach.
  target_clearance_radius_ = this->declare_parameter("target_clearance_radius", 0.80);
  // grasp_x_offset / grasp_z_offset: must match the values used in the BT XML
  // MoveArm nodes so the optimizer validates the actual grasp goal pose.
  grasp_x_offset_ = this->declare_parameter("grasp_x_offset", 0.20);
  grasp_z_offset_ = this->declare_parameter("grasp_z_offset", 0.20);

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize MoveIt (requires robot_description parameter to be available)
  rclcpp::NodeOptions loader_options;
  loader_options.automatically_declare_parameters_from_overrides(true);
  auto node_for_loader = std::make_shared<rclcpp::Node>("robot_model_loader_node", loader_options);

  // Note: For a real component, robot_description is usually passed via parameters to this node,
  // but robot_model_loader can also fetch it if standard topics/parameters are set.
  planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      node_for_loader, "robot_description");

  if (planning_scene_monitor_->getPlanningScene()) {
    planning_scene_monitor_->startSceneMonitor("/planning_scene");
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor("/joint_states", "/attached_collision_object");
    robot_model_ = planning_scene_monitor_->getRobotModel();
    RCLCPP_INFO(this->get_logger(), "Successfully loaded PlanningSceneMonitor and RobotModel.");
  } else {
    RCLCPP_ERROR(this->get_logger(),
        "Failed to load PlanningSceneMonitor! IK validation will fail.");
    robot_model_ = nullptr;
  }

  using namespace std::placeholders;

  // Costmap subscriber
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&BasePlacementOptimizerNode::costmap_callback, this, _1));

  action_server_ = rclcpp_action::create_server<OptimizePlacement>(
    this,
    "optimize_placement",
    std::bind(&BasePlacementOptimizerNode::handle_goal, this, _1, _2),
    std::bind(&BasePlacementOptimizerNode::handle_cancel, this, _1),
    std::bind(&BasePlacementOptimizerNode::handle_accepted, this, _1));
}

BasePlacementOptimizerNode::~BasePlacementOptimizerNode()
{
}

rclcpp_action::GoalResponse BasePlacementOptimizerNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const OptimizePlacement::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "Received goal request for target pose in frame: %s",
      goal->target_pose.header.frame_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BasePlacementOptimizerNode::handle_cancel(
  const std::shared_ptr<GoalHandleOptimizePlacement> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void BasePlacementOptimizerNode::handle_accepted(
  const std::shared_ptr<GoalHandleOptimizePlacement> goal_handle)
{
  // This needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&BasePlacementOptimizerNode::execute, this, std::placeholders::_1),
    goal_handle}.detach();
}

void BasePlacementOptimizerNode::execute(
  const std::shared_ptr<GoalHandleOptimizePlacement> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");

  auto feedback = std::make_shared<OptimizePlacement::Feedback>();
  auto result = std::make_shared<OptimizePlacement::Result>();

  // TBD: Actual logic goes here.

  auto goal = goal_handle->get_goal();

  feedback->current_phase = "TRANSFORMING_POSE";
  goal_handle->publish_feedback(feedback);

  // 1. TF Transform
  geometry_msgs::msg::PoseStamped map_target_pose;
  if (goal->target_pose.header.frame_id != map_frame_) {
    try {
      // Small timeout for fail-fast
      auto transform = tf_buffer_->lookupTransform(
        map_frame_, goal->target_pose.header.frame_id,
        tf2::TimePointZero, tf2::durationFromSec(0.05));
      tf2::doTransform(goal->target_pose, map_target_pose, transform);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "TF Transform failed: %s", ex.what());
      result->success = false;
      result->error_reason = "TRANSFORM_FAILED";
      goal_handle->abort(result);
      return;
    }
  } else {
    map_target_pose = goal->target_pose;
  }

  // 2. Validate RobotModel
  if (!robot_model_) {
    result->success = false;
    result->error_reason = "NO_ROBOT_MODEL";
    goal_handle->abort(result);
    return;
  }

  feedback->current_phase = "SAMPLING_CANDIDATES";
  goal_handle->publish_feedback(feedback);

  planning_scene_monitor::LockedPlanningSceneRO planning_scene(planning_scene_monitor_);
  moveit::core::RobotState robot_state = planning_scene->getCurrentState();
  const moveit::core::JointModelGroup * joint_model_group =
    robot_model_->getJointModelGroup(planning_group_);

  if (!joint_model_group) {
    result->success = false;
    result->error_reason = "INVALID_PLANNING_GROUP";
    goal_handle->abort(result);
    return;
  }

  // Target object pose in map frame
  Eigen::Isometry3d T_map_obj;
  tf2::fromMsg(map_target_pose.pose, T_map_obj);

  // Take single costmap snapshot
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_snapshot;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap_snapshot = latest_costmap_;
  }

  // 3. Radial Sampling
  struct Candidate
  {
    double x, y, theta;
    double score;
    double manipulability;
    double path_distance;
  };
  std::vector<Candidate> valid_candidates;

  for (int i = 0; i < angular_samples_; ++i) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->error_reason = "CANCELED";
      goal_handle->canceled(result);
      return;
    }

    double angle = (2.0 * M_PI * i) / angular_samples_;
    // The robot should face the object
    double ctheta = angle + M_PI; // point inwards

    // Sweep from target_clearance_radius_ up to reach_radius_ + 0.10.
    // The target_clearance_radius_ ensures we do not choose a pose that is
    // physically in collision with the table. The upper limit ensures we
    // stay within kinematic reach of the arm.
    double r_min = target_clearance_radius_;
    double r_max = std::max(r_min + 0.04, reach_radius_ + 0.10);
    for (double r = r_min; r <= r_max + 1e-4; r += 0.02) {
      double cx = map_target_pose.pose.position.x + r * std::cos(angle);
      double cy = map_target_pose.pose.position.y + r * std::sin(angle);

      // Geometric clearance guard: reject candidates too close to the target
      // object XY.  The table is often outside LIDAR range when the optimizer
      // runs, so the costmap cannot be relied upon for this check.
      double dist_to_target = std::hypot(
        cx - map_target_pose.pose.position.x,
        cy - map_target_pose.pose.position.y);
      if (dist_to_target < target_clearance_radius_) {
        continue;
      }

      // US2: Obstacle Check (Full footprint check against live costmap)
      if (!is_candidate_safe(cx, cy, ctheta, costmap_snapshot)) {
        continue;
      }

      // Create candidate transform (T_map_base)
      Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
      T_map_base.translation() = Eigen::Vector3d(cx, cy, 0.0);
      T_map_base.linear() = Eigen::AngleAxisd(ctheta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

      // Compute the actual grasp goal pose that MoveArm will target:
      // - Shift position by grasp_x_offset_ along the object→base direction
      //   (i.e. the direction from the object toward the robot base in XY).
      // - Add grasp_z_offset_ in Z.
      // - Apply horizontal side-grasp orientation (90° pitch, rotated by yaw).
      // This must match move_arm_action.cpp lines 100-113 exactly.
      Eigen::Vector3d obj_pos_map = T_map_obj.translation();
      Eigen::Vector3d base_pos_map(cx, cy, 0.0);
      Eigen::Vector3d obj_to_base_xy = (base_pos_map - obj_pos_map);
      obj_to_base_xy.z() = 0.0;
      double dist_xy = obj_to_base_xy.norm();
      Eigen::Vector3d obj_to_base_dir =
        dist_xy > 1e-3 ?
        Eigen::Vector3d(obj_to_base_xy / dist_xy) :
        Eigen::Vector3d::UnitX();
      double grasp_yaw = std::atan2(obj_to_base_dir.y(), obj_to_base_dir.x()) + M_PI;

      // Effective grasp position in map frame:
      // TCP is at a standoff on the BASE SIDE of the object (between base and object).
      // Matches move_arm_action.cpp which subtracts x_offset in the base→obj direction.
      Eigen::Vector3d grasp_pos_map =
        obj_pos_map +
        grasp_x_offset_ * Eigen::Vector3d(obj_to_base_dir.x(), obj_to_base_dir.y(), 0.0) +
        Eigen::Vector3d(0.0, 0.0, grasp_z_offset_);

      // Horizontal side-grasp quaternion (matches move_arm_action.cpp)
      double half_yaw = grasp_yaw * 0.5;
      Eigen::Quaterniond grasp_q(
        /* w */ 0.70710678 * std::cos(half_yaw),
        /* x */ -0.70710678 * std::sin(half_yaw),
        /* y */ 0.70710678 * std::cos(half_yaw),
        /* z */ 0.70710678 * std::sin(half_yaw));
      grasp_q.normalize();

      Eigen::Isometry3d T_map_grasp = Eigen::Isometry3d::Identity();
      T_map_grasp.translation() = grasp_pos_map;
      T_map_grasp.linear() = grasp_q.toRotationMatrix();

      // Convert grasp pose to base frame for IK
      Eigen::Isometry3d T_base_grasp = T_map_base.inverse() * T_map_grasp;

      // Run IK against the effective grasp pose (not the raw object pose)
      moveit::core::GroupStateValidityCallbackFn collision_callback =
        [&planning_scene](moveit::core::RobotState * rs, const moveit::core::JointModelGroup * jmg,
        const double * joint_group_variable_values) {
          rs->setJointGroupPositions(jmg, joint_group_variable_values);
          rs->update();
          return !planning_scene->isStateColliding(*rs, jmg->getName());
        };

      bool ik_valid = robot_state.setFromIK(joint_model_group, T_base_grasp, ik_timeout_,
          collision_callback);

      if (ik_valid) {
        // Guard: reject candidates that are farther from the robot's current
        // pose than max_nav_distance_.  This prevents NavFn from receiving a
        // goal that lies outside the navigable/mapped region of the costmap.
        double nav_dist_to_candidate = std::numeric_limits<double>::max();
        try {
          auto tf_stamped = tf_buffer_->lookupTransform(
            map_frame_, robot_base_frame_,
            tf2::TimePointZero, tf2::durationFromSec(0.1));
          double rx = tf_stamped.transform.translation.x;
          double ry = tf_stamped.transform.translation.y;
          nav_dist_to_candidate = std::hypot(cx - rx, cy - ry);
        } catch (const tf2::TransformException & ex) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "BasePlacementOptimizer: TF lookup for max_nav_distance guard failed: %s", ex.what());
          // If TF is unavailable, allow the candidate through rather than
          // discarding potentially good solutions.
        }

        if (nav_dist_to_candidate > max_nav_distance_) {
          RCLCPP_DEBUG(get_logger(),
            "Pruning candidate (%.2f, %.2f): %.2f m > max_nav_distance %.2f m",
            cx, cy, nav_dist_to_candidate, max_nav_distance_);
          continue;
        }

        double manipulability = compute_manipulability(robot_state, joint_model_group);
        double distance = std::hypot(cx - map_target_pose.pose.position.x,
            cy - map_target_pose.pose.position.y);
        double dist_score = distance > 0.01 ? (1.0 / distance) : 100.0;
        double score = (alpha_ * manipulability) + ((1.0 - alpha_) * dist_score);

        valid_candidates.push_back({cx, cy, ctheta, score, manipulability, distance});
      }
    }
  }

  if (!valid_candidates.empty()) {
    auto best = std::max_element(valid_candidates.begin(), valid_candidates.end(),
        [](const Candidate & a, const Candidate & b) {return a.score < b.score;});

    result->success = true;
    result->base_pose.header.frame_id = map_frame_;
    result->base_pose.header.stamp = this->now();
    result->base_pose.pose.position.x = best->x;
    result->base_pose.pose.position.y = best->y;
    result->base_pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, best->theta);
    result->base_pose.pose.orientation = tf2::toMsg(q);

    result->manipulability_score = best->manipulability;
    result->path_distance = best->path_distance;
    result->composite_score = best->score;

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Successfully found base placement. Score: %.3f", best->score);
  } else {
    result->success = false;
    result->error_reason = "NO_VALID_CANDIDATES";
    goal_handle->abort(result);
    RCLCPP_INFO(this->get_logger(), "Failed to find any kinematically valid base placement.");
  }
}

void BasePlacementOptimizerNode::costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

bool BasePlacementOptimizerNode::is_candidate_safe(
  double x, double y, double theta,
  const nav_msgs::msg::OccupancyGrid::SharedPtr & costmap) const
{
  if (!costmap) {
    // If no costmap is available, assume safe
    return true;
  }

  // Base footprint (from nav2_params.yaml)
  std::vector<std::pair<double, double>> footprint = {
    {0.495, 0.34}, {0.495, -0.34}, {-0.495, -0.34}, {-0.495, 0.34}
  };

  // Sample points along the footprint boundary
  std::vector<std::pair<double, double>> check_points;
  check_points.push_back({0.0, 0.0}); // Also check center

  int num_samples = 5;
  for (size_t i = 0; i < footprint.size(); ++i) {
    auto p1 = footprint[i];
    auto p2 = footprint[(i + 1) % footprint.size()];
    for (int j = 0; j < num_samples; ++j) {
      double t = static_cast<double>(j) / num_samples;
      check_points.push_back({
          p1.first + t * (p2.first - p1.first),
          p1.second + t * (p2.second - p1.second)
      });
    }
  }

  double cos_th = std::cos(theta);
  double sin_th = std::sin(theta);
  double origin_x = costmap->info.origin.position.x;
  double origin_y = costmap->info.origin.position.y;
  double resolution = costmap->info.resolution;

  for (const auto & pt : check_points) {
    // Transform footprint point to map frame
    double mx = x + pt.first * cos_th - pt.second * sin_th;
    double my = y + pt.first * sin_th + pt.second * cos_th;

    int cell_x = static_cast<int>((mx - origin_x) / resolution);
    int cell_y = static_cast<int>((my - origin_y) / resolution);

    if (cell_x < 0 || cell_x >= static_cast<int>(costmap->info.width) ||
      cell_y < 0 || cell_y >= static_cast<int>(costmap->info.height))
    {
      return false; // Out of bounds is not safe
    }

    int index = cell_y * costmap->info.width + cell_x;
    int8_t cost = costmap->data[index];

    bool is_center = (pt.first == 0.0 && pt.second == 0.0);
    if (is_center) {
      // Center safety: reject if inscribed-radius zone or lethal (cost >= 99) or unknown
      // This ensures the robot body center does not sit inside the inflation inscribed band
      if (cost >= 99 || cost < 0) {
        return false;
      }
    } else {
      // Footprint boundary safety: reject only if actual lethal (100) or unknown (-1)
      // Boundary points may sit in the inflated zone (cost 1..98) as long as they
      // don't intersect the obstacle itself
      if (cost == 100 || cost < 0) {
        return false;
      }
    }
  }

  return true;
}

double BasePlacementOptimizerNode::compute_manipulability(
  const moveit::core::RobotState & state,
  const moveit::core::JointModelGroup * jmg) const
{
  Eigen::MatrixXd jacobian;
  const moveit::core::LinkModel * link_model = state.getLinkModel(jmg->getLinkModelNames().back());
  if (!link_model) {return 0.0;}

  state.getJacobian(jmg, link_model, Eigen::Vector3d::Zero(), jacobian);

  // Yoshikawa's manipulability measure: sqrt(det(J * J^T))
  Eigen::MatrixXd j_jt = jacobian * jacobian.transpose();
  return std::sqrt(std::abs(j_jt.determinant()));
}

}  // namespace base_placement_optimizer

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(base_placement_optimizer::BasePlacementOptimizerNode)
