#include "pick_place_orchestrator/behavior_tree_nodes.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <cmath>
#include <algorithm>

namespace pick_place_orchestrator
{

enum class VisualServoState
{
  WAITING_FOR_IMAGES,
  MOVING
};

struct VisualServoPrivate
{
  VisualServoState state{VisualServoState::WAITING_FOR_IMAGES};
  int current_iteration{0};
  int max_iterations{3};
  double servo_threshold{0.01};

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  // Fix 5: subscriptions live for the node's lifetime (created in constructor)
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub;

  sensor_msgs::msg::Image::SharedPtr latest_color;
  sensor_msgs::msg::Image::SharedPtr latest_depth;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info;
};

VisualServoAction::VisualServoAction(const std::string & name, const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("VisualServoAction: Could not find 'node' on blackboard");
  }

  move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_,
      "robot_arm");

  // Fix 5: d_ is now an instance member (not static). Subscriptions are created once here
  // in the constructor and persist for the lifetime of this BT node instance.
  d_ = std::make_shared<VisualServoPrivate>();
  d_->tf_buffer = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  d_->tf_listener = std::make_shared<tf2_ros::TransformListener>(*(d_->tf_buffer));

  d_->image_sub = node_->create_subscription<sensor_msgs::msg::Image>(
    "/wrist_camera/image", 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      d_->latest_color = msg;
    });

  d_->depth_sub = node_->create_subscription<sensor_msgs::msg::Image>(
    "/wrist_camera/depth_image", 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      d_->latest_depth = msg;
    });

  d_->info_sub = node_->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/wrist_camera/camera_info", 10,
    [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
      d_->latest_info = msg;
    });
}

BT::PortsList VisualServoAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>("target_pose"),
    BT::InputPort<double>("x_offset"),
    BT::InputPort<double>("servo_threshold"),
    BT::InputPort<int>("max_iterations"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index"),
    // Servo writes the corrected arm pose here so MoveArm can descend/advance from it
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("corrected_pose")
  };
}

BT::NodeStatus VisualServoAction::onStart()
{
  started_ = false;

  bool bypass = false;
  if (!node_->has_parameter("bypass_visual_servo")) {
    node_->declare_parameter<bool>("bypass_visual_servo", false);
  }
  node_->get_parameter("bypass_visual_servo", bypass);

  if (bypass) {
    RCLCPP_INFO(node_->get_logger(),
        "VisualServo: bypass_visual_servo is true. Skipping visual servoing.");
    geometry_msgs::msg::PoseStamped target_pose;
    if (getInput("target_pose", target_pose)) {
      setOutput("corrected_pose", target_pose);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Fix 5: reset state only — do NOT recreate subscriptions
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
        RCLCPP_INFO(node_->get_logger(), "VisualServo: Move completed. Waiting for new images.");
        d_->latest_color = nullptr;
        d_->latest_depth = nullptr;
        d_->state = VisualServoState::WAITING_FOR_IMAGES;
        started_ = false;
      } else {
        reportFailure(config().blackboard,
          "VisualServo: MoveGroup failed with code: " + std::to_string(result.val), step_idx_);
        return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  // ── WAITING_FOR_IMAGES ────────────────────────────────────────────────────
  if (!d_->latest_color || !d_->latest_depth || !d_->latest_info) {
    return BT::NodeStatus::RUNNING;
  }

  RCLCPP_INFO(node_->get_logger(), "VisualServo: Images received. Processing iteration %d/%d...",
    d_->current_iteration, d_->max_iterations);

  // ── Convert colour image ──────────────────────────────────────────────────
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(d_->latest_color, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "VisualServo: cv_bridge colour exception: %s", e.what());
    return BT::NodeStatus::FAILURE;
  }

  // ── HSV mask for yellow cylinder ─────────────────────────────────────────
  cv::Mat hsv;
  cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
  cv::Mat mask;
  cv::inRange(hsv, cv::Scalar(15, 80, 50), cv::Scalar(40, 255, 255), mask);

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  if (contours.empty()) {
    cv::imwrite(
        "/home/mox/.gemini/antigravity/brain/93e3932c-2e82-4a2d-931d-ed97b8590283/wrist_camera_view.png",
        cv_ptr->image);
    RCLCPP_WARN(node_->get_logger(),
        "VisualServo: No yellow contours found! Saved image to wrist_camera_view.png");
    d_->latest_color = nullptr;
    return BT::NodeStatus::RUNNING;
  }

  // Largest contour
  double max_area = 0.0;
  int largest_idx = -1;
  for (size_t i = 0; i < contours.size(); ++i) {
    double area = cv::contourArea(contours[i]);
    if (area > max_area) {max_area = area; largest_idx = static_cast<int>(i);}
  }

  if (largest_idx == -1 || max_area < 200.0) {
    RCLCPP_WARN(node_->get_logger(), "VisualServo: Largest contour area too small: %f", max_area);
    d_->latest_color = nullptr;
    return BT::NodeStatus::RUNNING;
  }

  // Centroid
  cv::Moments m = cv::moments(contours[largest_idx]);
  if (m.m00 == 0) {
    RCLCPP_WARN(node_->get_logger(), "VisualServo: Zero moments");
    return BT::NodeStatus::FAILURE;
  }
  double cx = m.m10 / m.m00;
  double cy = m.m01 / m.m00;

  // ── Convert depth image ───────────────────────────────────────────────────
  cv_bridge::CvImagePtr depth_ptr;
  try {
    depth_ptr = cv_bridge::toCvCopy(d_->latest_depth, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception & e) {
    try {
      depth_ptr = cv_bridge::toCvCopy(d_->latest_depth, d_->latest_depth->encoding);
    } catch (cv_bridge::Exception & e2) {
      RCLCPP_ERROR(node_->get_logger(), "VisualServo: cv_bridge depth exception: %s", e2.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  cv::Mat depth_img = depth_ptr->image;
  float depth_val = 0.0f;
  if (depth_img.type() == CV_32FC1) {
    depth_val = depth_img.at<float>(static_cast<int>(cy), static_cast<int>(cx));
  } else if (depth_img.type() == CV_16UC1) {
    depth_val = depth_img.at<uint16_t>(static_cast<int>(cy), static_cast<int>(cx)) * 0.001f;
  }

  // Neighbourhood search fallback
  if (std::isnan(depth_val) || depth_val <= 0.1f || depth_val > 5.0f) {
    float sum = 0.0f;
    int count = 0;
    for (int dy = -3; dy <= 3; ++dy) {
      for (int dx = -3; dx <= 3; ++dx) {
        int nx = static_cast<int>(cx) + dx;
        int ny = static_cast<int>(cy) + dy;
        if (nx >= 0 && nx < depth_img.cols && ny >= 0 && ny < depth_img.rows) {
          float val = 0.0f;
          if (depth_img.type() == CV_32FC1) {
            val = depth_img.at<float>(ny, nx);
          } else if (depth_img.type() == CV_16UC1) {
            val = depth_img.at<uint16_t>(ny, nx) * 0.001f;
          }
          if (!std::isnan(val) && val > 0.1f && val < 5.0f) {
            sum += val; count++;
          }
        }
      }
    }
    if (count > 0) {
      depth_val = sum / count;
    } else {
      std::stringstream ss;
      ss << "[";
      for (int dy = -2; dy <= 2; ++dy) {
        ss << "\n  ";
        for (int dx = -2; dx <= 2; ++dx) {
          int nx = static_cast<int>(cx) + dx;
          int ny = static_cast<int>(cy) + dy;
          if (nx >= 0 && nx < depth_img.cols && ny >= 0 && ny < depth_img.rows) {
            float val = (depth_img.type() == CV_32FC1) ? depth_img.at<float>(ny,
  nx) : depth_img.at<uint16_t>(ny, nx) * 0.001f;
            ss << val << " ";
          } else {
            ss << "OOB ";
          }
        }
      }
      ss << "\n]";
      RCLCPP_WARN(node_->get_logger(),
        "VisualServo: Could not determine valid depth at centroid (cx=%f, cy=%f, type=%d, size=%dx%d). Neighborhood grid:%s",
        cx, cy, depth_img.type(), depth_img.cols, depth_img.rows, ss.str().c_str());
      d_->latest_color = nullptr;
      d_->latest_depth = nullptr;
      return BT::NodeStatus::RUNNING;
    }
  }

  // ── Intrinsics ────────────────────────────────────────────────────────────
  double fx = d_->latest_info->k[0];
  double fy = d_->latest_info->k[4];
  double cx_info = d_->latest_info->k[2];
  double cy_info = d_->latest_info->k[5];

  if (fx == 0.0 || fy == 0.0) {
    fx = fy = (depth_img.cols / 2.0) / std::tan(1.50098 / 2.0);
    cx_info = depth_img.cols / 2.0;
    cy_info = depth_img.rows / 2.0;
  }

  // Fix 4: read frame_id from camera_info
  std::string camera_frame = d_->latest_info->header.frame_id;
  if (camera_frame.empty()) {
    camera_frame = "wrist_camera_link";
  }

  // Construct the optical frame name dynamically (pixel u,v and intrinsics are defined in the optical frame)
  std::string optical_frame = camera_frame;
  if (optical_frame.size() > 5 && optical_frame.substr(optical_frame.size() - 5) == "_link") {
    optical_frame = optical_frame.substr(0, optical_frame.size() - 5) + "_depth_optical_frame";
  } else if (optical_frame.find("optical") == std::string::npos) {
    optical_frame = optical_frame + "_depth_optical_frame";
  }


  // Calculate the 3D position of the object in the camera optical frame.
  // Add 0.03 m (cylinder radius) along the optical Z axis (look direction) to target the center.
  geometry_msgs::msg::PointStamped obj_cam;
  obj_cam.header.frame_id = optical_frame;
  obj_cam.header.stamp = rclcpp::Time(0);
  obj_cam.point.x = (cx - cx_info) * depth_val / fx;
  obj_cam.point.y = (cy - cy_info) * depth_val / fy;
  obj_cam.point.z = depth_val + 0.03;

  // Transform object 3D position into the planning frame
  geometry_msgs::msg::PointStamped obj_planning;
  try {
    obj_planning = d_->tf_buffer->transform(obj_cam, move_group_->getPlanningFrame());
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(node_->get_logger(), "VisualServo TF object transform exception: %s", ex.what());
    return BT::NodeStatus::FAILURE;
  }

  // Read approach x_offset from input port
  double x_offset = 0.0;
  getInput("x_offset", x_offset);

  // Get current pose of the gripper flange (ur5e_tool0)
  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose("ur5e_tool0");
  geometry_msgs::msg::Pose target_pose = current_pose.pose;

  // Calculate approach direction directly from the current gripper orientation (in planning frame)
  // to avoid gimbal lock singularities associated with RPY yaw at 90 deg pitch.
  tf2::Quaternion q;
  tf2::fromMsg(current_pose.pose.orientation, q);
  tf2::Vector3 approach_dir = tf2::quatRotate(q, tf2::Vector3(0, 0, 1));
  double yaw = std::atan2(approach_dir.y(), approach_dir.x());

  RCLCPP_WARN(node_->get_logger(),
    "VisualServo DEBUG: approach_dir = [%.3f, %.3f, %.3f] | Gripper pointing sideways (Y-axis)",
    approach_dir.x(), approach_dir.y(), approach_dir.z());

  // Calculate the target standoff pose: align XY with object, but MAINTAIN current Z height
  // approach_dir.z ≈ 0 (gripper horizontal), so using it causes upward drift as camera sees object higher in frame
  target_pose.position.x = obj_planning.point.x - x_offset * approach_dir.x();
  target_pose.position.y = obj_planning.point.y - x_offset * approach_dir.y();
  target_pose.position.z = current_pose.pose.position.z;  // KEEP CURRENT Z - no vertical motion!

  // Calculate alignment error relative to the standoff pose in 3D
  double error = std::sqrt(
    std::pow(current_pose.pose.position.x - target_pose.position.x, 2) +
    std::pow(current_pose.pose.position.y - target_pose.position.y, 2) +
    std::pow(current_pose.pose.position.z - target_pose.position.z, 2));

  RCLCPP_INFO(node_->get_logger(),
    "VisualServo: Target object 3D pos=[%f, %f, %f] (yaw: %f) → Standoff target=[%f, %f, %f] → current gripper=[%f, %f, %f]",
    obj_planning.point.x, obj_planning.point.y, obj_planning.point.z, yaw,
    target_pose.position.x, target_pose.position.y, target_pose.position.z,
    current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  RCLCPP_INFO(node_->get_logger(),
    "VisualServo: 3D alignment error = %f m (frame: %s)",
    error, move_group_->getPlanningFrame().c_str());

  if (error < d_->servo_threshold) {
    RCLCPP_INFO(node_->get_logger(),
      "VisualServo: Aligned! error (%f) < threshold (%f)", error, d_->servo_threshold);

    // Output corrected pose: the detected OBJECT position (final MoveArm will advance to this)
    geometry_msgs::msg::PoseStamped corrected_base;
    corrected_base.header.frame_id = move_group_->getPlanningFrame();
    corrected_base.header.stamp = node_->now();
    corrected_base.pose.position.x = obj_planning.point.x;
    corrected_base.pose.position.y = obj_planning.point.y;
    corrected_base.pose.position.z = obj_planning.point.z;
    corrected_base.pose.orientation = current_pose.pose.orientation;

    geometry_msgs::msg::PoseStamped corrected_map;
    try {
      d_->tf_buffer->transform(corrected_base, corrected_map, "map", tf2::durationFromSec(0.5));
      RCLCPP_INFO(node_->get_logger(),
        "VisualServo: Outputting corrected_pose in map frame: x=%.3f, y=%.3f, z=%.3f (detected object position)",
        corrected_map.pose.position.x, corrected_map.pose.position.y,
          corrected_map.pose.position.z);
      setOutput("corrected_pose", corrected_map);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
          "VisualServo: TF transform failed: %s. Using base_footprint frame.", ex.what());
      setOutput("corrected_pose", corrected_base);
    }
    return BT::NodeStatus::SUCCESS;
  }

  if (d_->current_iteration >= d_->max_iterations) {
    RCLCPP_WARN(node_->get_logger(),
      "VisualServo: Max iterations reached. Writing best-effort corrected pose.");

    // Output corrected pose: object XY position but CURRENT gripper Z
    geometry_msgs::msg::PoseStamped corrected_base;
    corrected_base.header.frame_id = move_group_->getPlanningFrame();
    corrected_base.header.stamp = node_->now();
    corrected_base.pose.position.x = obj_planning.point.x;
    corrected_base.pose.position.y = obj_planning.point.y;
    corrected_base.pose.position.z = current_pose.pose.position.z;  // Keep current Z!
    corrected_base.pose.orientation = current_pose.pose.orientation;

    geometry_msgs::msg::PoseStamped corrected_map;
    try {
      d_->tf_buffer->transform(corrected_base, corrected_map, "map", tf2::durationFromSec(0.5));
      RCLCPP_INFO(node_->get_logger(),
        "VisualServo: Outputting best-effort corrected_pose in map frame: x=%.3f, y=%.3f, z=%.3f (maintained current Z)",
        corrected_map.pose.position.x, corrected_map.pose.position.y,
          corrected_map.pose.position.z);
      setOutput("corrected_pose", corrected_map);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(),
          "VisualServo: TF transform failed: %s. Using base_footprint frame.", ex.what());
      setOutput("corrected_pose", corrected_base);
    }
    return BT::NodeStatus::SUCCESS;
  }

  // Clamp the correction step magnitude to max 15 cm to prevent large unexpected motions
  double dx = target_pose.position.x - current_pose.pose.position.x;
  double dy = target_pose.position.y - current_pose.pose.position.y;
  double dz = target_pose.position.z - current_pose.pose.position.z;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (dist > 0.15) {
    dx = (dx / dist) * 0.15;
    dy = (dy / dist) * 0.15;
    dz = (dz / dist) * 0.15;
    target_pose.position.x = current_pose.pose.position.x + dx;
    target_pose.position.y = current_pose.pose.position.y + dy;
    target_pose.position.z = current_pose.pose.position.z + dz;
  }

  RCLCPP_INFO(node_->get_logger(),
    "VisualServo: Planning Cartesian correction step to target=[%.4f, %.4f, %.4f] (offset: %.4f, %.4f, %.4f)",
    target_pose.position.x, target_pose.position.y, target_pose.position.z, dx, dy, dz);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double eef_step = 0.005;   // 5 mm interpolation step
  const double jump_thresh = 1.0;  // Check for joint jumps to avoid flips
  double fraction = move_group_->computeCartesianPath(
    waypoints, eef_step, jump_thresh, trajectory);

  if (fraction < 0.8) {
    // Less than 80% of the path is reachable — skip this iteration
    RCLCPP_WARN(node_->get_logger(),
      "VisualServo: Cartesian path only %.1f%% complete (collision near target). "
      "Skipping this correction (iteration %d/%d). "
      "This can indicate wrong camera orientation or gripper too far from object.",
      fraction * 100.0, d_->current_iteration, d_->max_iterations);
    d_->latest_color = nullptr;
    d_->latest_depth = nullptr;
    d_->current_iteration++;
    return BT::NodeStatus::RUNNING;
  }

  // Execute the Cartesian plan asynchronously
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory;

  future_ = std::async(std::launch::async, [this, plan]() mutable {
        move_group_->setMaxVelocityScalingFactor(0.15);
        move_group_->setMaxAccelerationScalingFactor(0.15);
        return move_group_->execute(plan);
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
  // Fix 5: subscriptions are NOT destroyed here; they live for the node's lifetime.
  // Only reset per-tick data.
  d_->latest_color = nullptr;
  d_->latest_depth = nullptr;
  d_->latest_info = nullptr;
}

}  // namespace pick_place_orchestrator
