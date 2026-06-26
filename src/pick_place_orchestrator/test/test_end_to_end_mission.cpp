#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstdlib>

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "pick_place_orchestrator/orchestrator_node.hpp"
#include "base_placement_optimizer/action/optimize_placement.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/query_planner_interfaces.hpp"
#include "moveit_msgs/srv/get_planner_params.hpp"
#include "moveit_msgs/srv/set_planner_params.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"

namespace pick_place_orchestrator
{

// Helper to resolve xacro and read files
inline std::string getResolvedUrdf()
{
  std::string config_dir =
    ament_index_cpp::get_package_share_directory("mobile_manipulator_moveit_config");
  std::string xacro_path = config_dir + "/config/mobile_manipulator.urdf.xacro";
  std::string urdf_out = "/tmp/mobile_manipulator_test.urdf";
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

// Mock Action Server Base Class
template<typename ActionT>
class MockActionServer
{
public:
  MockActionServer(
    rclcpp::Node::SharedPtr node, const std::string & action_name,
    bool succeed = true)
  : node_(node), name_(action_name), succeed_(succeed)
  {
    using namespace std::placeholders;
    server_ = rclcpp_action::create_server<ActionT>(
      node_,
      name_,
      std::bind(&MockActionServer::handle_goal, this, _1, _2),
      std::bind(&MockActionServer::handle_cancel, this, _1),
      std::bind(&MockActionServer::handle_accepted, this, _1)
    );
  }

protected:
  virtual void setResult(std::shared_ptr<typename ActionT::Result> result)
  {
    (void)result;
  }

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename ActionT::Goal> goal)
  {
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
  {
    std::thread([this, goal_handle]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        auto result = std::make_shared<typename ActionT::Result>();
        setResult(result);
        if (succeed_) {
          goal_handle->succeed(result);
        } else {
          goal_handle->abort(result);
        }
    }).detach();
  }

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  bool succeed_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
};

// Specialized Mock Servers
class MockOptimizeServer : public MockActionServer<base_placement_optimizer::action::OptimizePlacement>
{
public:
  MockOptimizeServer(rclcpp::Node::SharedPtr node, bool succeed = true)
  : MockActionServer(node, "optimize_placement", succeed) {}

protected:
  void setResult(
    std::shared_ptr<base_placement_optimizer::action::OptimizePlacement::Result> result) override
  {
    result->success = true;
    result->base_pose.header.frame_id = "map";
    result->base_pose.pose.position.x = 2.0;
    result->base_pose.pose.position.y = 1.0;
    result->base_pose.pose.orientation.w = 1.0;
  }
};

class MockMoveGroupServer : public MockActionServer<::moveit_msgs::action::MoveGroup>
{
public:
  MockMoveGroupServer(
    rclcpp::Node::SharedPtr node, const std::string & name = "move_action",
    bool succeed = true)
  : MockActionServer(node, name, succeed) {}

protected:
  void setResult(std::shared_ptr<::moveit_msgs::action::MoveGroup::Result> result) override
  {
    result->error_code.val = ::moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
};

class MockExecuteTrajectoryServer : public MockActionServer<::moveit_msgs::action::ExecuteTrajectory>
{
public:
  MockExecuteTrajectoryServer(rclcpp::Node::SharedPtr node, bool succeed = true)
  : MockActionServer(node, "execute_trajectory", succeed) {}

protected:
  void setResult(std::shared_ptr<::moveit_msgs::action::ExecuteTrajectory::Result> result) override
  {
    result->error_code.val = ::moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
};

class MockNavServer : public MockActionServer<nav2_msgs::action::NavigateToPose>
{
public:
  MockNavServer(rclcpp::Node::SharedPtr node, bool succeed = true)
  : MockActionServer(node, "navigate_to_pose", succeed) {}
};

class EndToEndTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);

    // Build options with MoveIt parameters
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"robot_description", getResolvedUrdf()},
        {"robot_description_semantic", getSrdf()},
        {"bt_xml_path",
          ament_index_cpp::get_package_share_directory("pick_place_orchestrator") +
          "/behavior_trees/pick_place_mission.xml"}
    });

    orchestrator_node_ = std::make_shared<OrchestratorNode>(options);
    mock_node_ = rclcpp::Node::make_shared("mock_servers_node");

    // Initialize Mock servers
    mock_optimize_ = std::make_unique<MockOptimizeServer>(mock_node_);
    mock_nav_ = std::make_unique<MockNavServer>(mock_node_);
    mock_move_group_ = std::make_unique<MockMoveGroupServer>(mock_node_, "move_action");
    mock_move_group_2_ = std::make_unique<MockMoveGroupServer>(mock_node_, "move_group");
    mock_execute_trajectory_ = std::make_unique<MockExecuteTrajectoryServer>(mock_node_);

    mock_get_scene_service_ = mock_node_->create_service<::moveit_msgs::srv::GetPlanningScene>(
      "/get_planning_scene",
      [](const std::shared_ptr<::moveit_msgs::srv::GetPlanningScene::Request> request,
      std::shared_ptr<::moveit_msgs::srv::GetPlanningScene::Response> response) {
        (void)request;
        response->scene.is_diff = true;
        response->scene.robot_state.is_diff = true;
      }
    );

    mock_apply_scene_service_ = mock_node_->create_service<::moveit_msgs::srv::ApplyPlanningScene>(
      "/apply_planning_scene",
      [](const std::shared_ptr<::moveit_msgs::srv::ApplyPlanningScene::Request> request,
      std::shared_ptr<::moveit_msgs::srv::ApplyPlanningScene::Response> response) {
        (void)request;
        response->success = true;
      }
    );

    mock_query_planner_service_ = mock_node_->create_service<::moveit_msgs::srv::QueryPlannerInterfaces>(
      "/query_planner_interface",
      [](const std::shared_ptr<::moveit_msgs::srv::QueryPlannerInterfaces::Request> request,
      std::shared_ptr<::moveit_msgs::srv::QueryPlannerInterfaces::Response> response) {
        (void)request;
        (void)response;
      }
    );

    mock_get_planner_params_service_ = mock_node_->create_service<::moveit_msgs::srv::GetPlannerParams>(
      "/get_planner_params",
      [](const std::shared_ptr<::moveit_msgs::srv::GetPlannerParams::Request> request,
      std::shared_ptr<::moveit_msgs::srv::GetPlannerParams::Response> response) {
        (void)request;
        (void)response;
      }
    );

    mock_set_planner_params_service_ = mock_node_->create_service<::moveit_msgs::srv::SetPlannerParams>(
      "/set_planner_params",
      [](const std::shared_ptr<::moveit_msgs::srv::SetPlannerParams::Request> request,
      std::shared_ptr<::moveit_msgs::srv::SetPlannerParams::Response> response) {
        (void)request;
        (void)response;
      }
    );

    mock_compute_ik_service_ = mock_node_->create_service<::moveit_msgs::srv::GetPositionIK>(
      "/compute_ik",
      [](const std::shared_ptr<::moveit_msgs::srv::GetPositionIK::Request> request,
      std::shared_ptr<::moveit_msgs::srv::GetPositionIK::Response> response) {
        (void)request;
        response->error_code.val = ::moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      }
    );

    mock_compute_fk_service_ = mock_node_->create_service<::moveit_msgs::srv::GetPositionFK>(
      "/compute_fk",
      [](const std::shared_ptr<::moveit_msgs::srv::GetPositionFK::Request> request,
      std::shared_ptr<::moveit_msgs::srv::GetPositionFK::Response> response) {
        (void)request;
        response->error_code.val = ::moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
      }
    );

    mock_state_validity_service_ = mock_node_->create_service<::moveit_msgs::srv::GetStateValidity>(
      "/check_state_validity",
      [](const std::shared_ptr<::moveit_msgs::srv::GetStateValidity::Request> request,
      std::shared_ptr<::moveit_msgs::srv::GetStateValidity::Response> response) {
        (void)request;
        response->valid = true;
      }
    );

    // Spin executors in threads
    orchestrator_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    orchestrator_executor_->add_node(orchestrator_node_);
    orchestrator_thread_ = std::thread([this]() {orchestrator_executor_->spin();});

    mock_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    mock_executor_->add_node(mock_node_);
    mock_thread_ = std::thread([this]() {mock_executor_->spin();});
  }

  void TearDown() override
  {
    orchestrator_executor_->cancel();
    if (orchestrator_thread_.joinable()) {
      orchestrator_thread_.join();
    }

    mock_executor_->cancel();
    if (mock_thread_.joinable()) {
      mock_thread_.join();
    }

    rclcpp::shutdown();
  }

  std::shared_ptr<OrchestratorNode> orchestrator_node_;
  std::shared_ptr<rclcpp::Node> mock_node_;
  std::unique_ptr<MockOptimizeServer> mock_optimize_;
  std::unique_ptr<MockNavServer> mock_nav_;
  std::unique_ptr<MockMoveGroupServer> mock_move_group_;
  std::unique_ptr<MockMoveGroupServer> mock_move_group_2_;
  std::unique_ptr<MockExecuteTrajectoryServer> mock_execute_trajectory_;
  rclcpp::Service<::moveit_msgs::srv::GetPlanningScene>::SharedPtr mock_get_scene_service_;
  rclcpp::Service<::moveit_msgs::srv::ApplyPlanningScene>::SharedPtr mock_apply_scene_service_;
  rclcpp::Service<::moveit_msgs::srv::QueryPlannerInterfaces>::SharedPtr mock_query_planner_service_;
  rclcpp::Service<::moveit_msgs::srv::GetPlannerParams>::SharedPtr mock_get_planner_params_service_;
  rclcpp::Service<::moveit_msgs::srv::SetPlannerParams>::SharedPtr mock_set_planner_params_service_;
  rclcpp::Service<::moveit_msgs::srv::GetPositionIK>::SharedPtr mock_compute_ik_service_;
  rclcpp::Service<::moveit_msgs::srv::GetPositionFK>::SharedPtr mock_compute_fk_service_;
  rclcpp::Service<::moveit_msgs::srv::GetStateValidity>::SharedPtr mock_state_validity_service_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> orchestrator_executor_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> mock_executor_;
  std::thread orchestrator_thread_;
  std::thread mock_thread_;
};

TEST_F(EndToEndTest, ExecutesFullMission)
{
  auto client_node = rclcpp::Node::make_shared("test_client");
  auto action_client = rclcpp_action::create_client<pick_place_orchestrator::action::PickPlaceMission>(
    client_node, "pick_place_mission");

  ASSERT_TRUE(action_client->wait_for_action_server(std::chrono::seconds(5)));

  auto goal = pick_place_orchestrator::action::PickPlaceMission::Goal();
  goal.pick_pose.header.frame_id = "map";
  goal.pick_pose.pose.position.x = 1.0;
  goal.pick_pose.pose.position.y = 2.0;
  goal.pick_pose.pose.orientation.w = 1.0;

  goal.place_pose.header.frame_id = "map";
  goal.place_pose.pose.position.x = 4.0;
  goal.place_pose.pose.position.y = -1.0;
  goal.place_pose.pose.orientation.w = 1.0;

  auto goal_handle_future = action_client->async_send_goal(goal);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, goal_handle_future, std::chrono::seconds(5)),
    rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);

  auto result_future = action_client->async_get_result(goal_handle);
  ASSERT_EQ(
    rclcpp::spin_until_future_complete(client_node, result_future, std::chrono::seconds(15)),
    rclcpp::FutureReturnCode::SUCCESS);

  auto wrapped_result = result_future.get();
  EXPECT_EQ(wrapped_result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(wrapped_result.result->success);
}

}  // namespace pick_place_orchestrator

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
