#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pick_place_orchestrator/action/pick_place_mission.hpp"

using namespace std::chrono_literals;
using PickPlaceMission = pick_place_orchestrator::action::PickPlaceMission;
using GoalHandlePickPlace = rclcpp_action::ClientGoalHandle<PickPlaceMission>;

class MissionTriggerNode : public rclcpp::Node
{
public:
  MissionTriggerNode()
  : Node("mission_trigger_node")
  {
    RCLCPP_INFO(this->get_logger(), "Mission Trigger Node starting...");

    // Declare parameters for pick and place poses
    // Object in world file at x=4.5, y=4.0, z=1.20 (pick table)
    // Approach at z=1.40m to avoid wrist collision with table during visual servo
    // Visual servo will detect actual object position
    this->declare_parameter("pick_x", 4.5);
    this->declare_parameter("pick_y", 4.0);
    this->declare_parameter("pick_z", 1.40);  // Raised approach height for wrist clearance
    this->declare_parameter("place_x", 4.5);
    this->declare_parameter("place_y", -4.0);
    this->declare_parameter("place_z", 1.30);

    // Get parameters
    double pick_x = this->get_parameter("pick_x").as_double();
    double pick_y = this->get_parameter("pick_y").as_double();
    double pick_z = this->get_parameter("pick_z").as_double();
    double place_x = this->get_parameter("place_x").as_double();
    double place_y = this->get_parameter("place_y").as_double();
    double place_z = this->get_parameter("place_z").as_double();

    RCLCPP_INFO(this->get_logger(),
      "Pick pose:  (%.2f, %.2f, %.2f)", pick_x, pick_y, pick_z);
    RCLCPP_INFO(this->get_logger(),
      "Place pose: (%.2f, %.2f, %.2f)", place_x, place_y, place_z);

    // Create action client
    action_client_ = rclcpp_action::create_client<PickPlaceMission>(
      this, "pick_place_mission");

    // Wait for action server
    RCLCPP_INFO(this->get_logger(),
      "Waiting for pick_place_mission action server...");

    if (!action_client_->wait_for_action_server(30s)) {
      RCLCPP_ERROR(this->get_logger(),
        "Action server not available after 30 seconds. Exiting.");
      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Action server is ready!");

    // Build goal message
    auto goal_msg = PickPlaceMission::Goal();

    // Pick pose
    goal_msg.pick_pose.header.frame_id = "map";
    goal_msg.pick_pose.header.stamp = this->now();
    goal_msg.pick_pose.pose.position.x = pick_x;
    goal_msg.pick_pose.pose.position.y = pick_y;
    goal_msg.pick_pose.pose.position.z = pick_z;
    goal_msg.pick_pose.pose.orientation.x = 0.0;  // Gripper pointing down, wrist up
    goal_msg.pick_pose.pose.orientation.y = 0.0;
    goal_msg.pick_pose.pose.orientation.z = 0.0;
    goal_msg.pick_pose.pose.orientation.w = 1.0;

    // Place pose
    goal_msg.place_pose.header.frame_id = "map";
    goal_msg.place_pose.header.stamp = this->now();
    goal_msg.place_pose.pose.position.x = place_x;
    goal_msg.place_pose.pose.position.y = place_y;
    goal_msg.place_pose.pose.position.z = place_z;
    goal_msg.place_pose.pose.orientation.x = 0.0;  // Gripper pointing down, wrist up
    goal_msg.place_pose.pose.orientation.y = 0.0;
    goal_msg.place_pose.pose.orientation.z = 0.0;
    goal_msg.place_pose.pose.orientation.w = 1.0;

    // Send goal
    RCLCPP_INFO(this->get_logger(),
      "Sending pick and place mission goal...");

    auto send_goal_options = rclcpp_action::Client<PickPlaceMission>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandlePickPlace> goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was REJECTED by server!");
        } else {
          RCLCPP_INFO(this->get_logger(), "Goal ACCEPTED by server, executing...");
        }
      };

    send_goal_options.feedback_callback =
      [this](
      std::shared_ptr<GoalHandlePickPlace>,
      const std::shared_ptr<const PickPlaceMission::Feedback> feedback) {
        RCLCPP_INFO(this->get_logger(),
          "Feedback - Phase: %s | Step: %s | Index: %d",
          feedback->current_phase.c_str(),
          feedback->current_step.c_str(),
          feedback->step_index);
      };

    send_goal_options.result_callback =
      [this](const GoalHandlePickPlace::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(),
              "✓ Mission SUCCEEDED! Object successfully moved from pick to place table.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(),
              "✗ Mission ABORTED. Reason: %s | Failed at step: %d",
              result.result->failure_reason.c_str(),
              result.result->failed_step_index);
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Mission was CANCELED.");
            break;
          default:
            RCLCPP_ERROR(this->get_logger(), "Mission completed with UNKNOWN result code.");
            break;
        }

        // Shutdown after result
        RCLCPP_INFO(this->get_logger(), "Mission trigger node shutting down.");
        rclcpp::shutdown();
      };

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<PickPlaceMission>::SharedPtr action_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MissionTriggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
