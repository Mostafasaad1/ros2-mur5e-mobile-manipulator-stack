#include "base_placement_optimizer/optimizer_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include <thread>
#include <functional>
#include <cmath>

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
  reach_radius_ = this->declare_parameter("reach_radius", 0.7);
  angular_samples_ = this->declare_parameter("angular_samples", 16);
  ik_timeout_ = this->declare_parameter("ik_timeout", 0.002);
  alpha_ = this->declare_parameter("alpha", 0.5);

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize MoveIt (requires robot_description parameter to be available)
  rclcpp::NodeOptions loader_options;
  loader_options.automatically_declare_parameters_from_overrides(true);
  auto node_for_loader = std::make_shared<rclcpp::Node>("robot_model_loader_node", loader_options);

  // Note: For a real component, robot_description is usually passed via parameters to this node,
  // but robot_model_loader can also fetch it if standard topics/parameters are set.
  robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_for_loader,
      "robot_description");
  robot_model_ = robot_model_loader_->getModel();

  if (!robot_model_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load RobotModel! IK validation will fail.");
  } else {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded RobotModel.");
  }

  // Costmap subscriber
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic_, rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&BasePlacementOptimizerNode::costmap_callback, this, _1));

  using namespace std::placeholders;
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

  moveit::core::RobotState robot_state(robot_model_);
  robot_state.setToDefaultValues();
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

    // Candidate position (radius distance from object)
    double cx = map_target_pose.pose.position.x + reach_radius_ * std::cos(angle);
    double cy = map_target_pose.pose.position.y + reach_radius_ * std::sin(angle);

    // US2: Obstacle Check
    if (!is_candidate_safe(cx, cy, costmap_snapshot)) {
      continue;
    }

    // The robot should face the object
    double ctheta = angle + M_PI; // point inwards

    // Create candidate transform (T_map_base)
    Eigen::Isometry3d T_map_base = Eigen::Isometry3d::Identity();
    T_map_base.translation() = Eigen::Vector3d(cx, cy, 0.0);
    T_map_base.linear() = Eigen::AngleAxisd(ctheta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // Convert object pose to base_link frame
    Eigen::Isometry3d T_base_obj = T_map_base.inverse() * T_map_obj;

    // Run IK
    bool ik_valid = robot_state.setFromIK(joint_model_group, T_base_obj, ik_timeout_);

    if (ik_valid) {
      double manipulability = compute_manipulability(robot_state, joint_model_group);
      double distance = std::hypot(cx - map_target_pose.pose.position.x,
          cy - map_target_pose.pose.position.y);
      double dist_score = distance > 0.01 ? (1.0 / distance) : 100.0;
      double score = (alpha_ * manipulability) + ((1.0 - alpha_) * dist_score);

      valid_candidates.push_back({cx, cy, ctheta, score, manipulability, distance});
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
  double x, double y,
  const nav_msgs::msg::OccupancyGrid::SharedPtr & costmap) const
{
  if (!costmap) {
    // If no costmap is available, assume safe or fail? The assumption is that Nav2 provides it.
    // For now, if no costmap, we skip safety check.
    return true;
  }

  // Convert world coordinates to map cell indices
  double origin_x = costmap->info.origin.pose.position.x;
  double origin_y = costmap->info.origin.pose.position.y;
  double resolution = costmap->info.resolution;

  int cell_x = static_cast<int>((x - origin_x) / resolution);
  int cell_y = static_cast<int>((y - origin_y) / resolution);

  // Check bounds
  if (cell_x < 0 || cell_x >= static_cast<int>(costmap->info.width) ||
    cell_y < 0 || cell_y >= static_cast<int>(costmap->info.height))
  {
    return false; // Out of bounds is not safe
  }

  int index = cell_y * costmap->info.width + cell_x;
  int8_t cost = costmap->data[index];

  // 254 is LETHAL, 253 is INSCRIBED_INFLATED
  if (cost == 254 || cost == 253) {
    return false;
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
