#include "pick_place_orchestrator/behavior_tree_nodes.hpp"
#include <cmath>
#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_state/conversions.hpp"

namespace pick_place_orchestrator
{

MoveArmAction::MoveArmAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("MoveArmAction: Could not find 'node' on blackboard");
  }
  // Initialize MoveGroupInterface
  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,
      "robot_arm");
}

BT::PortsList MoveArmAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<double>("z_offset"),
    BT::InputPort<double>("x_offset"),
    BT::InputPort<std::string>("named_pose"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index"),
    BT::InputPort<std::string>("planner_id"),
    BT::InputPort<double>("velocity_scaling"),
    BT::InputPort<double>("acceleration_scaling"),
    // Optional: SRDF named state to use as IK seed for PTP moves.
    // Seeding prevents Pilz/KDL from picking an elbow-flipped solution.
    BT::InputPort<std::string>("arm_config")
  };
}

BT::NodeStatus MoveArmAction::onStart()
{
  started_ = false;

  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  step_idx_ = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx_);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx_);

  std::string named_pose;
  geometry_msgs::msg::PoseStamped target_pose;
  bool has_named = getInput("named_pose", named_pose).has_value() && !named_pose.empty();
  bool has_target = getInput("target_pose", target_pose).has_value();

  if (!has_named && !has_target) {
    reportFailure(config().blackboard, "Neither 'named_pose' nor 'target_pose' provided",
        step_idx_);
    return BT::NodeStatus::FAILURE;
  }

  std::string planner_id_override = "";
  getInput("planner_id", planner_id_override);

  std::string arm_config_seed = "";
  getInput("arm_config", arm_config_seed);

  double velocity_scaling = 0.8;
  double acceleration_scaling = 0.8;
  bool has_vel_scale = getInput("velocity_scaling", velocity_scaling).has_value();
  bool has_acc_scale = getInput("acceleration_scaling", acceleration_scaling).has_value();

  // Compute final target pose with offsets and yaw alignment if has_target is true
  if (has_target) {
    double z_offset = 0.0;
    getInput("z_offset", z_offset);
    double x_offset = 0.0;
    getInput("x_offset", x_offset);

    double yaw = 0.0;
    geometry_msgs::msg::PoseStamped base_pose;
    bool has_base_pose = false;
    if (phase == "PICK_PHASE") {
      has_base_pose = config().blackboard->get("optimized_pick_base_pose", base_pose);
    } else if (phase == "PLACE_PHASE") {
      has_base_pose = config().blackboard->get("optimized_place_base_pose", base_pose);
    }

    if (has_base_pose) {
      double base_x = base_pose.pose.position.x;
      double base_y = base_pose.pose.position.y;
      double dx = target_pose.pose.position.x - base_x;
      double dy = target_pose.pose.position.y - base_y;
      yaw = std::atan2(dy, dx);
      RCLCPP_INFO(node_->get_logger(),
          "MoveArmAction: Aligned grasp yaw to %.2f rad based on base pos (%.2f, %.2f) and target (%.2f, %.2f)",
        yaw, base_x, base_y, target_pose.pose.position.x, target_pose.pose.position.y);
    } else {
      RCLCPP_WARN(node_->get_logger(),
          "MoveArmAction: Could not find optimized base pose on blackboard for phase %s. Using default yaw = 0.0",
        phase.c_str());
    }

    // Set horizontal grasp orientation (90 deg pitch rotated by yaw around Z)
    double half_yaw = yaw * 0.5;
    target_pose.pose.orientation.x = -0.70710678 * std::sin(half_yaw);
    target_pose.pose.orientation.y = 0.70710678 * std::cos(half_yaw);
    target_pose.pose.orientation.z = 0.70710678 * std::sin(half_yaw);
    target_pose.pose.orientation.w = 0.70710678 * std::cos(half_yaw);

    if (x_offset != 0.0) {
      // Negate: yaw points FROM base TOWARD object, so subtracting places
      // the TCP between the base and the object (standoff on the base side).
      target_pose.pose.position.x -= x_offset * std::cos(yaw);
      target_pose.pose.position.y -= x_offset * std::sin(yaw);
    }
    if (z_offset != 0.0) {
      target_pose.pose.position.z += z_offset;
    }

    RCLCPP_INFO(node_->get_logger(),
        "MoveArmAction: Planning to target pose in frame: %s (x: %.3f, y: %.3f, z: %.3f, offset_x: %.3f, offset_z: %.3f)",
      target_pose.header.frame_id.c_str(), target_pose.pose.position.x, target_pose.pose.position.y,
        target_pose.pose.position.z, x_offset, z_offset);
  }

  // Start execution in background thread to avoid blocking BT tick
  future_ = std::async(std::launch::async,
      [this, has_named, has_target, named_pose, target_pose, planner_id_override,
      velocity_scaling, acceleration_scaling, has_vel_scale, has_acc_scale, arm_config_seed]() {
        std::string pipeline = "pilz_industrial_motion_planner";
        std::string planner = "";
        bool fallback_allowed = false;

        if (!planner_id_override.empty()) {
          if (planner_id_override == "OMPL") {
            pipeline = "ompl";
            planner = "";
            fallback_allowed = false;
          } else if (planner_id_override == "PTP") {
            pipeline = "pilz_industrial_motion_planner";
            planner = "PTP";
            fallback_allowed = true;
          } else if (planner_id_override == "LIN") {
            pipeline = "pilz_industrial_motion_planner";
            planner = "LIN";
            fallback_allowed = false;
          }
        } else {
          if (has_named) {
            pipeline = "pilz_industrial_motion_planner";
            planner = "PTP";
            fallback_allowed = true;
          } else if (has_target) {
            pipeline = "pilz_industrial_motion_planner";
            planner = "LIN";
            fallback_allowed = false;
          }
        }

        // Apply scaling factors (LIN defaults to 0.05 for stability, PTP/others default to 0.8, user overrides preferred)
        double final_vel_scale = 0.8;
        double final_acc_scale = 0.8;
        if (planner == "LIN") {
          final_vel_scale = 0.05;
          final_acc_scale = 0.05;
        }
        if (has_vel_scale) {
          final_vel_scale = velocity_scaling;
        }
        if (has_acc_scale) {
          final_acc_scale = acceleration_scaling;
        }

        move_group_->setMaxVelocityScalingFactor(final_vel_scale);
        move_group_->setMaxAccelerationScalingFactor(final_acc_scale);
        RCLCPP_INFO(node_->get_logger(),
            "MoveArmAction: Set scaling factors - velocity: %.2f, acceleration: %.2f",
            final_vel_scale, final_acc_scale);

        // Apply path constraints only for LIN Cartesian moves
        if (pipeline == "pilz_industrial_motion_planner" && planner == "LIN" && has_target) {
          moveit_msgs::msg::Constraints path_constraints;
          moveit_msgs::msg::OrientationConstraint ocm;
          ocm.link_name = move_group_->getEndEffectorLink();
          ocm.header.frame_id =
          target_pose.header.frame_id.empty() ? move_group_->getPlanningFrame() :
          target_pose.header.frame_id;
          ocm.orientation = target_pose.pose.orientation;
          ocm.absolute_x_axis_tolerance = 0.02; // roll
          ocm.absolute_y_axis_tolerance = 0.02; // pitch
          ocm.absolute_z_axis_tolerance = 2 * M_PI; // yaw
          ocm.weight = 1.0;
          path_constraints.orientation_constraints.push_back(ocm);
          move_group_->setPathConstraints(path_constraints);
          RCLCPP_INFO(node_->get_logger(),
          "MoveArmAction: Set path orientation constraints for LIN Cartesian approach (roll/pitch tolerance: 0.02)");
        } else {
          move_group_->clearPathConstraints();
        }

        // Always plan from the actual current robot state
        move_group_->setStartStateToCurrentState();
        if (!arm_config_seed.empty() && planner == "PTP" && has_target) {
          RCLCPP_INFO(node_->get_logger(),
            "MoveArmAction: arm_config='%s' noted; KDL IK will seed from current state (arm "
            "should already be near this configuration)", arm_config_seed.c_str());
        }

        // Set target on move_group
        if (has_named) {
          if (!move_group_->setNamedTarget(named_pose)) {
            RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: Failed to set named target: '%s'",
            named_pose.c_str());
            return moveit::core::MoveItErrorCode(
            moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
          }
        } else if (has_target) {
          if (!move_group_->setPoseTarget(target_pose)) {
            RCLCPP_ERROR(node_->get_logger(), "MoveArmAction: Failed to set pose target");
            return moveit::core::MoveItErrorCode(
            moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
          }
        }

        // Set planning pipeline/planner
        move_group_->setPlanningPipelineId(pipeline);
        if (!planner.empty()) {
          move_group_->setPlannerId(planner);
        }

        RCLCPP_INFO(node_->get_logger(),
        "MoveArmAction: Planning attempt with pipeline: %s, planner: %s", pipeline.c_str(),
        planner.c_str());

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode plan_res = move_group_->plan(plan);

        if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
          RCLCPP_WARN(node_->get_logger(), "MoveArmAction: Primary planning failed with code: %d",
          plan_res.val);

          if (fallback_allowed) {
            RCLCPP_INFO(node_->get_logger(),
            "MoveArmAction: Fallback allowed. Retrying with OMPL...");
            move_group_->setPlanningPipelineId("ompl");
            move_group_->setPlannerId("");
            move_group_->clearPathConstraints();

            // Set target again
            if (has_named) {
              move_group_->setNamedTarget(named_pose);
            } else if (has_target) {
              move_group_->setPoseTarget(target_pose);
            }

            plan_res = move_group_->plan(plan);
            if (plan_res != moveit::core::MoveItErrorCode::SUCCESS) {
              RCLCPP_ERROR(node_->get_logger(),
              "MoveArmAction: Fallback planning also failed with code: %d", plan_res.val);
              return plan_res;
            }
            RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Fallback planning succeeded.");
          } else {
            RCLCPP_ERROR(node_->get_logger(),
            "MoveArmAction: Primary planning failed and fallback is NOT allowed. Aborting.");
            return plan_res;
          }
        } else {
          RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Primary planning succeeded.");
        }

        RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Executing planned trajectory...");
        moveit::core::MoveItErrorCode exec_res = move_group_->execute(plan);

        // Always clear path constraints after execution
        move_group_->clearPathConstraints();

        return exec_res;
  });

  started_ = true;
  return BT::NodeStatus::RUNNING;
}


BT::NodeStatus MoveArmAction::onRunning()
{
  if (started_) {
    if (future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
      moveit::core::MoveItErrorCode result = future_.get();
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Motion completed successfully.");
        return BT::NodeStatus::SUCCESS;
      } else {
        reportFailure(config().blackboard,
            "Arm movement failed with error code: " + std::to_string(result.val), step_idx_);
        return BT::NodeStatus::FAILURE;
      }
    }
  }
  return BT::NodeStatus::RUNNING;
}

void MoveArmAction::onHalted()
{
  if (move_group_) {
    RCLCPP_INFO(node_->get_logger(), "MoveArmAction: Halted. Stopping active arm movement.");
    move_group_->stop();
  }
  if (future_.valid()) {
    future_.wait();
  }
}

}  // namespace pick_place_orchestrator
