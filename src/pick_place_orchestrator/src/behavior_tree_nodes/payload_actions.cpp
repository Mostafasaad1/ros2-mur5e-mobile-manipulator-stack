#include "pick_place_orchestrator/behavior_tree_nodes.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace pick_place_orchestrator
{

// --- AttachPayloadAction ---

AttachPayloadAction::AttachPayloadAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("AttachPayloadAction: Could not find 'node' on blackboard");
  }
}

BT::PortsList AttachPayloadAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_id", std::string("target_payload"),
        "ID of the payload object"),
    BT::InputPort<std::string>("attached_link", std::string("ur5e_tool0"),
        "Robot link to attach payload to"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus AttachPayloadAction::tick()
{
  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  int step_idx = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx);

  std::string object_id;
  std::string attached_link;
  getInput("object_id", object_id);
  getInput("attached_link", attached_link);

  RCLCPP_INFO(node_->get_logger(), "AttachPayloadAction: Attaching '%s' to link '%s'",
    object_id.c_str(), attached_link.c_str());

  // 1. Create CollisionObject representing the payload
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = attached_link;
  collision_object.id = object_id;

  // Define actual cylinder workpiece size: radius = 0.03m, length = 0.60m
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 0.60;
  primitive.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.03;

  // Center the cylinder at fingertips (Z axis offset of 0.03m)
  // Rotate by 90 degrees around Y axis to align the cylinder's height (Z axis)
  // with tool0's X axis (which points vertically in world coordinates when grasping).
  geometry_msgs::msg::Pose cylinder_pose;
  cylinder_pose.position.x = 0.0;
  cylinder_pose.position.y = 0.0;
  cylinder_pose.position.z = 0.03;
  cylinder_pose.orientation.x = 0.0;
  cylinder_pose.orientation.y = 0.70710678;
  cylinder_pose.orientation.z = 0.0;
  cylinder_pose.orientation.w = 0.70710678;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(cylinder_pose);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

  // 2. Wrap it inside an AttachedCollisionObject message
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = attached_link;
  attached_object.object = collision_object;
  // Touch links allow fingers/wrist to touch the object without triggering collision faults
  attached_object.touch_links = {"finger_left", "finger_right", attached_link, "ur5e_flange",
    "ur5e_wrist_3_link"};

  // 3. Apply changes asynchronously to the PlanningScene
  if (!planning_scene_interface_.applyAttachedCollisionObject(attached_object)) {
    reportFailure(config().blackboard,
        "Failed to apply attached collision object to planning scene", step_idx);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(node_->get_logger(), "AttachPayloadAction: Successfully attached payload.");
  return BT::NodeStatus::SUCCESS;
}

// --- DetachPayloadAction ---

DetachPayloadAction::DetachPayloadAction(const std::string & name, const BT::NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
  if (!node_) {
    throw std::runtime_error("DetachPayloadAction: Could not find 'node' on blackboard");
  }
}

BT::PortsList DetachPayloadAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("object_id", std::string("target_payload"),
        "ID of the payload object"),
    BT::InputPort<std::string>("attached_link", std::string("ur5e_tool0"),
        "Robot link the payload is attached to"),
    BT::InputPort<std::string>("phase"),
    BT::InputPort<std::string>("step"),
    BT::InputPort<int>("step_index")
  };
}

BT::NodeStatus DetachPayloadAction::tick()
{
  std::string phase = "UNKNOWN_PHASE";
  std::string step = "UNKNOWN_STEP";
  int step_idx = 0;
  getInput("phase", phase);
  getInput("step", step);
  getInput("step_index", step_idx);

  updateBlackboardFeedback(config().blackboard, phase, step, step_idx);

  std::string object_id;
  std::string attached_link;
  getInput("object_id", object_id);
  getInput("attached_link", attached_link);

  RCLCPP_INFO(node_->get_logger(),
      "DetachPayloadAction: Detaching and removing '%s' from link '%s'",
    object_id.c_str(), attached_link.c_str());

  // 1. Create message to detach the object
  moveit_msgs::msg::AttachedCollisionObject detach_object;
  detach_object.link_name = attached_link;
  detach_object.object.id = object_id;
  detach_object.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  if (!planning_scene_interface_.applyAttachedCollisionObject(detach_object)) {
    RCLCPP_WARN(node_->get_logger(),
        "DetachPayloadAction: Detach operation warning or no object was attached.");
  }

  // 2. Create message to remove object completely from world planning scene
  moveit_msgs::msg::CollisionObject remove_object;
  remove_object.id = object_id;
  remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;

  if (!planning_scene_interface_.applyCollisionObject(remove_object)) {
    RCLCPP_WARN(node_->get_logger(),
        "DetachPayloadAction: Remove operation warning or no object was present in scene.");
  }

  RCLCPP_INFO(node_->get_logger(),
      "DetachPayloadAction: Successfully detached and removed payload.");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pick_place_orchestrator
