#include "pick_place_orchestrator/behavior_tree_nodes.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>

namespace pick_place_orchestrator
{

enum class VisualServoState {
  WAITING_FOR_IMAGES,
  MOVING
};

struct VisualServoPrivate {
  VisualServoState state{VisualServoState::WAITING_FOR_IMAGES};
  int current_iteration{0};
  int max_iterations{3};
  double servo_threshold{0.01};
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

  sensor_msgs::msg::Image::SharedPtr latest_color;
  sensor_msgs::msg::Image::SharedPtr latest_depth;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info;
};

// We will use a shared pointer to this private struct to store the state between ticks
static std::shared_ptr<VisualServoPrivate> d_;

VisualServoAction::VisualServoAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("VisualServoAction: Could not find 'node' on blackboard");
  }

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "robot_arm");
  
  if (!d_) {
    d_ = std::make_shared<VisualServoPrivate>();
    d_->tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    d_->tf_listener = std::make_shared<tf2_ros::TransformListener>(*(d_->tf_buffer));
  }
}

BT::PortsList VisualServoAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<double>("servo_threshold"),
    BT::InputPort<int>("max_iterations"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus VisualServoAction::onStart()
{
  started_ = false;
  d_->state = VisualServoState::WAITING_FOR_IMAGES;
  d_->current_iteration = 0;
  d_->latest_color = nullptr;
  d_->latest_depth = nullptr;
  d_->latest_info = nullptr;

  d_->servo_threshold = 0.01;
  d_->max_iterations = 3;
  getInput("servo_threshold", d_->servo_threshold);
  getInput("max_iterations", d_->max_iterations);

  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  step_idx_ = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx_);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx_);

  // Create subscriptions
  d_->image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
    "/wrist_camera/image", 10,
    [](const sensor_msgs::msg::Image::SharedPtr msg) {
      if (d_) d_->latest_color = msg;
    });

  d_->depth_sub = node_->create_subscription<sensor_msgs::msg::Image>(
    "/wrist_camera/depth_image", 10,
    [](const sensor_msgs::msg::Image::SharedPtr msg) {
      if (d_) d_->latest_depth = msg;
    });

  d_->info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/wrist_camera/camera_info", 10,
    [](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      if (d_) d_->latest_info = msg;
    });

  RCLCPP_INFO(node_->get_logger(), "VisualServo: Started. Max iterations: %d, threshold: %f m",
    d_->max_iterations, d_->servo_threshold);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus VisualServoAction::onRunning()
{
  if (d_->state == VisualServoState::MOVING) {
    if (!started_) {
      return BT::NodeStatus::RUNNING;
    }

    auto status = future_.wait_for(std::chrono::milliseconds(0));
    if (status == std::future_status::ready) {
      auto result = future_.get();
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node_->get_logger(), "VisualServo: Move completed successfully. Waiting for new images.");
        d_->latest_color = nullptr;
        d_->latest_depth = nullptr;
        d_->state = VisualServoState::WAITING_FOR_IMAGES;
        started_ = false;
      } else {
        reportFailure(config().blackboard, "VisualServo: MoveGroup failed with code: " + std::to_string(result.val), step_idx_);
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  // Waiting for images state
  if (!d_->latest_color || !d_->latest_depth || !d_->latest_info) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "VisualServo: Images received. Processing iteration %d/%d...",
    d_->current_iteration, d_->max_iterations);

  // Convert color image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(d_->latest_color, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(node_->get_logger(), "VisualServo: cv_bridge color exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  // HSV Mask for yellow cylinder
  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(15, 80, 50), cv::Scalar(40, 255, 255), mask);

  // Find contours
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    RCLCPP_WARN(node_->get_logger(), "VisualServo: No yellow contours found!");
    // In simulation sometimes it takes a frame to settle, we can continue waiting
    d_->latest_color = nullptr;
    return BT::NodeStatus::RUNNING;
  }

  // Get largest contour
  double max_area = 0.0;
  int largest_idx = -1;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > max_area) {
      max_area = area;
      largest_idx = i;
    }
  }

  if (largest_idx == -1 || max_area < 200.0) {
    RCLCPP_WARN(node_->get_logger(), "VisualServo: Largest contour area too small: %f", max_area);
    d_->latest_color = nullptr;
    return BT::NodeStatus::RUNNING;
  }

  // Calculate centroid
  cv::Moments m = cv::moments(contours[largest_idx]);
  if (m.m00 == 0) {
    RCLCPP_WARN(node_->get_logger(), "VisualServo: Zero moments");
    return BT::NodeStatus::FAILURE;
  }
  double cx = m.m10 / m.m00;
  double cy = m.m01 / m.m00;

  // Convert depth image
  cv_bridge::CvImagePtr depth_ptr;
  try {
    depth_ptr = cv_bridge::toCvCopy(d_->latest_depth, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    try {
      depth_ptr = cv_bridge::toCvCopy(d_->latest_depth, d_->latest_depth->encoding);
    } catch (cv_bridge::Exception& e2) {
      RCLCPP_ERROR(node_->get_logger(), "VisualServo: cv_bridge depth exception: %s", e2.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  cv::Mat depth_img = depth_ptr->image;
  float depth_val = 0.0;
  if (depth_img.type() == CV_32FC1) {
    depth_val = depth_img.at<float>(static_cast<int>(cy), static_cast<int>(cx));
  } else if (depth_img.type() == CV_16UC1) {
    depth_val = depth_img.at<uint16_t>(static_cast<int>(cy), static_cast<int>(cx)) * 0.001f;
  }

  // Neighborhood search fallback
  if (std::isnan(depth_val) || depth_val <= 0.1 || depth_val > 5.0) {
    float sum = 0.0;
    int count = 0;
    for (int dy = -3; dy <= 3; ++dy) {
      for (int dx = -3; dx <= 3; ++dx) {
        int nx = static_cast<int>(cx) + dx;
        int ny = static_cast<int>(cy) + dy;
        if (nx >= 0 && nx < depth_img.cols && ny >= 0 && ny < depth_img.rows) {
          float val = 0.0;
          if (depth_img.type() == CV_32FC1) {
            val = depth_img.at<float>(ny, nx);
          } else if (depth_img.type() == CV_16UC1) {
            val = depth_img.at<uint16_t>(ny, nx) * 0.001f;
          }
          if (!std::isnan(val) && val > 0.1 && val < 5.0) {
            sum += val;
            count++;
          }
        }
      }
    }
    if (count > 0) {
      depth_val = sum / count;
    } else {
      RCLCPP_WARN(node_->get_logger(), "VisualServo: Could not determine valid depth at centroid");
      d_->latest_color = nullptr;
      d_->latest_depth = nullptr;
      return BT::NodeStatus::RUNNING;
    }
  }

  // Intrinsics
  double fx = d_->latest_info->k[0];
  double fy = d_->latest_info->k[4];
  double cx_info = d_->latest_info->k[2];
  double cy_info = d_->latest_info->k[5];

  if (fx == 0.0 || fy == 0.0) {
    fx = fy = (depth_img.cols / 2.0) / std::tan(1.50098 / 2.0);
    cx_info = depth_img.cols / 2.0;
    cy_info = depth_img.rows / 2.0;
  }

  // Unproject to camera coordinates
  double x_cam = (cx - cx_info) * depth_val / fx;
  double y_cam = (cy - cy_info) * depth_val / fy;
  double z_cam = depth_val;

  // Transform to tool0
  geometry_msgs::msg::PointStamped point_cam;
  point_cam.header.frame_id = d_->latest_color->header.frame_id;
  point_cam.header.stamp = rclcpp::Time(0); // latest available
  point_cam.point.x = x_cam;
  point_cam.point.y = y_cam;
  point_cam.point.z = z_cam;

  geometry_msgs::msg::PointStamped point_tool;
  try {
    // Wait for transform availability
    if (d_->tf_buffer->canTransform("ur5e_tool0", point_cam.header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
      point_tool = d_->tf_buffer->transform(point_cam, "ur5e_tool0");
    } else {
      RCLCPP_WARN(node_->get_logger(), "VisualServo: TF transform from %s to ur5e_tool0 not available", point_cam.header.frame_id.c_str());
      return BT::NodeStatus::RUNNING;
    }
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "VisualServo TF Exception: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // We want to align tool0 center with the cylinder center.
  // The lateral alignment correction in tool0 coordinates:
  double dx = point_tool.point.x;
  double dy = point_tool.point.y;

  // Safety limits: max 15 cm shift per iteration
  if (std::abs(dx) > 0.15) dx = std::clamp(dx, -0.15, 0.15);
  if (std::abs(dy) > 0.15) dy = std::clamp(dy, -0.15, 0.15);

  double error = std::sqrt(dx*dx + dy*dy);
  RCLCPP_INFO(node_->get_logger(), "VisualServo: Centroid in tool0: x=%f, y=%f (error: %f m)", dx, dy, error);

  if (error < d_->servo_threshold) {
    RCLCPP_INFO(node_->get_logger(), "VisualServo: Aligned! error (%f) < threshold (%f)", error, d_->servo_threshold);
    return BT::NodeStatus::SUCCESS;
  }

  if (d_->current_iteration >= d_->max_iterations) {
    RCLCPP_WARN(node_->get_logger(), "VisualServo: Max iterations reached without target alignment. Continuing with current alignment.");
    return BT::NodeStatus::SUCCESS;
  }

  // Transform translation to planning frame
  geometry_msgs::msg::Vector3Stamped delta_tool;
  delta_tool.header.frame_id = "ur5e_tool0";
  delta_tool.header.stamp = rclcpp::Time(0);
  delta_tool.vector.x = dx;
  delta_tool.vector.y = dy;
  delta_tool.vector.z = 0.0;

  geometry_msgs::msg::Vector3Stamped delta_planning;
  try {
    delta_planning = d_->tf_buffer->transform(delta_tool, move_group_->getPlanningFrame());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "VisualServo TF delta exception: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Calculate new target pose
  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose("ur5e_tool0");
  geometry_msgs::msg::PoseStamped target_pose = current_pose;
  target_pose.pose.position.x += delta_planning.vector.x;
  target_pose.pose.position.y += delta_planning.vector.y;
  target_pose.pose.position.z += delta_planning.vector.z;

  RCLCPP_INFO(node_->get_logger(), "VisualServo: Moving arm to corrected pose: x=%f, y=%f, z=%f",
    target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

  move_group_->setPoseTarget(target_pose);

  future_ = std::async(std::launch::async, [this]() {
    move_group_->setMaxVelocityScalingFactor(0.2); // safe slow speed for alignment
    move_group_->setMaxAccelerationScalingFactor(0.2);
    return move_group_->move();
  });

  started_ = true;
  d_->state = VisualServoState::MOVING;
  d_->current_iteration++;

  return BT::NodeStatus::RUNNING;
}

void VisualServoAction::onHalted()
{
  move_group_->stop();
  started_ = false;
  
  // Cleanup subscriptions to prevent leak
  d_->image_sub.reset();
  d_->depth_sub.reset();
  d_->info_sub.reset();
}

}  // namespace pick_place_orchestrator
