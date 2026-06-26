#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"

#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

// Helper to resolve xacro and read files
inline std::string getResolvedUrdf()
{
  std::string config_dir =
    ament_index_cpp::get_package_share_directory("mobile_manipulator_moveit_config");
  std::string xacro_path = config_dir + "/config/mobile_manipulator.urdf.xacro";
  std::string urdf_out = "/tmp/mobile_manipulator_test_payload.urdf";
  std::string cmd = "xacro " + xacro_path + " ros2_control_hardware_type:=gz > " + urdf_out;
  int ret = std::system(cmd.c_str());
  if (ret != 0) {
    throw std::runtime_error("Failed to run xacro");
  }
  std::ifstream ifs(urdf_out);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

inline std::string getSrdf()
{
  std::string config_dir =
    ament_index_cpp::get_package_share_directory("mobile_manipulator_moveit_config");
  std::string srdf_path = config_dir + "/config/mobile_manipulator.srdf";
  std::ifstream ifs(srdf_path);
  std::stringstream ss;
  ss << ifs.rdbuf();
  return ss.str();
}

class PayloadAttachmentTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"robot_description", getResolvedUrdf()},
        {"robot_description_semantic", getSrdf()}
    });

    node_ = std::make_shared<rclcpp::Node>("payload_test_node", options);
    mock_node_ = std::make_shared<rclcpp::Node>("mock_planning_scene_node");

    // Spawn mock /apply_planning_scene service
    mock_service_ = mock_node_->create_service<moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene",
      [this](const std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request> request,
      std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Response> response) {
        this->requests_.push_back(request);
        response->success = true;
      }
    );

    mock_get_scene_service_ = mock_node_->create_service<moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene",
      [](const std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Request> request,
      std::shared_ptr<moveit_msgs::srv::GetPlanningScene::Response> response) {
        (void)request;
        response->scene.is_diff = true;
      }
    );

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(mock_node_);
    executor_thread_ = std::thread([this]() {executor_->spin();});
  }

  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> mock_node_;
  rclcpp::Service<moveit_msgs::srv::ApplyPlanningScene>::SharedPtr mock_service_;
  rclcpp::Service<moveit_msgs::srv::GetPlanningScene>::SharedPtr mock_get_scene_service_;
  std::vector<std::shared_ptr<moveit_msgs::srv::ApplyPlanningScene::Request>> requests_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

TEST_F(PayloadAttachmentTest, AttachesAndDetachesPayload)
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<AttachPayloadAction>("AttachPayload");
  factory.registerNodeType<DetachPayloadAction>("DetachPayload");

  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node_);
  blackboard->set("current_phase", std::string("IDLE"));
  blackboard->set("current_step", std::string("IDLE"));
  blackboard->set("step_index", 0);

  std::string xml =
    R"(<root BTCPP_format="4">
         <BehaviorTree ID="TestTree">
           <Sequence>
             <AttachPayload object_id="test_payload" attached_link="ur5e_tool0" phase="PICK" step="ATTACH" step_index="5"/>
             <DetachPayload object_id="test_payload" attached_link="ur5e_tool0" phase="PLACE" step="DETACH" step_index="11"/>
           </Sequence>
         </BehaviorTree>
       </root>)";

  auto tree = factory.createTreeFromText(xml, blackboard);

  // Tick the tree
  BT::NodeStatus status = tree.tickExactlyOnce();
  ASSERT_EQ(status, BT::NodeStatus::SUCCESS);

  // Verify that the mock planning scene service was called with correct data
  ASSERT_EQ(requests_.size(), 3u);

  // Verify AttachPayload (ADD)
  auto & scene_attach = requests_[0]->scene;
  ASSERT_EQ(scene_attach.robot_state.attached_collision_objects.size(), 1u);
  auto & attached_obj = scene_attach.robot_state.attached_collision_objects[0];
  EXPECT_EQ(attached_obj.link_name, "ur5e_tool0");
  EXPECT_EQ(attached_obj.object.id, "test_payload");
  EXPECT_EQ(attached_obj.object.operation, moveit_msgs::msg::CollisionObject::ADD);

  // Verify touch links
  bool has_left = false;
  bool has_right = false;
  for (const auto & link : attached_obj.touch_links) {
    if (link == "finger_left") {has_left = true;}
    if (link == "finger_right") {has_right = true;}
  }
  EXPECT_TRUE(has_left);
  EXPECT_TRUE(has_right);

  // Verify DetachPayload (REMOVE from robot)
  auto & scene_detach = requests_[1]->scene;
  ASSERT_EQ(scene_detach.robot_state.attached_collision_objects.size(), 1u);
  auto & detached_obj = scene_detach.robot_state.attached_collision_objects[0];
  EXPECT_EQ(detached_obj.object.id, "test_payload");
  EXPECT_EQ(detached_obj.object.operation, moveit_msgs::msg::CollisionObject::REMOVE);

  // Verify DetachPayload (REMOVE from world)
  auto & scene_remove = requests_[2]->scene;
  ASSERT_EQ(scene_remove.world.collision_objects.size(), 1u);
  auto & removed_obj = scene_remove.world.collision_objects[0];
  EXPECT_EQ(removed_obj.id, "test_payload");
  EXPECT_EQ(removed_obj.operation, moveit_msgs::msg::CollisionObject::REMOVE);
}

}  // namespace pick_place_orchestrator

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
