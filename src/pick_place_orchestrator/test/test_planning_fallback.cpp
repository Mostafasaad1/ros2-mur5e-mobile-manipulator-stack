#include <thread>
#include <chrono>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <mutex>
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "moveit_msgs/action/move_group.hpp"
#include "moveit_msgs/action/execute_trajectory.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "moveit_msgs/srv/apply_planning_scene.hpp"
#include "moveit_msgs/srv/query_planner_interfaces.hpp"
#include "moveit_msgs/srv/get_planner_params.hpp"
#include "moveit_msgs/srv/set_planner_params.hpp"
#include "moveit_msgs/srv/get_position_ik.hpp"
#include "moveit_msgs/srv/get_position_fk.hpp"
#include "moveit_msgs/srv/get_state_validity.hpp"

#include "pick_place_orchestrator/behavior_tree_nodes.hpp"

namespace pick_place_orchestrator
{

inline std::string getResolvedUrdf()
{
  std::string config_dir =
    ament_index_cpp::get_package_share_directory("mobile_manipulator_moveit_config");
  std::string xacro_path = config_dir + "/config/mobile_manipulator.urdf.xacro";
  std::string urdf_out = "/tmp/mobile_manipulator_test_fallback.urdf";
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

template<typename ActionT>
class MockActionServer
{
public:
  MockActionServer(rclcpp::Node::SharedPtr node, const std::string & action_name)
  : node_(node), name_(action_name)
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

  std::shared_ptr<const typename ActionT::Goal> get_last_goal() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_goal_;
  }

  void clear_last_goal()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_goal_ = nullptr;
  }

protected:
  virtual void setResult(std::shared_ptr<typename ActionT::Result> result) = 0;

private:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const typename ActionT::Goal> goal)
  {
    (void)uuid;
    std::lock_guard<std::mutex> lock(mutex_);
    last_goal_ = goal;
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
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        auto result = std::make_shared<typename ActionT::Result>();
        setResult(result);
        goal_handle->succeed(result);
    }).detach();
  }

  rclcpp::Node::SharedPtr node_;
  std::string name_;
  typename rclcpp_action::Server<ActionT>::SharedPtr server_;
  mutable std::mutex mutex_;
  std::shared_ptr<const typename ActionT::Goal> last_goal_;
};

class MockMoveGroupFallbackServer : public MockActionServer<moveit_msgs::action::MoveGroup>
{
public:
  MockMoveGroupFallbackServer(rclcpp::Node::SharedPtr node, const std::string & name)
  : MockActionServer(node, name), fail_pilz_(false), pilz_attempts_(0), ompl_attempts_(0) {}

  void set_fail_pilz(bool fail)
  {
    fail_pilz_ = fail;
  }

  int get_pilz_attempts() const
  {
    return pilz_attempts_;
  }

  int get_ompl_attempts() const
  {
    return ompl_attempts_;
  }

  void reset_counters()
  {
    pilz_attempts_ = 0;
    ompl_attempts_ = 0;
  }

protected:
  void setResult(std::shared_ptr<moveit_msgs::action::MoveGroup::Result> result) override
  {
    auto goal = get_last_goal();
    if (goal) {
      if (goal->request.pipeline_id == "pilz_industrial_motion_planner") {
        pilz_attempts_++;
        if (fail_pilz_) {
          result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
          return;
        }
      } else if (goal->request.pipeline_id == "ompl") {
        ompl_attempts_++;
      }
    }
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }

private:
  bool fail_pilz_;
  int pilz_attempts_;
  int ompl_attempts_;
};

class MockExecuteTrajectoryServer : public MockActionServer<moveit_msgs::action::ExecuteTrajectory>
{
public:
  MockExecuteTrajectoryServer(rclcpp::Node::SharedPtr node, const std::string & name)
  : MockActionServer(node, name) {}

protected:
  void setResult(std::shared_ptr<moveit_msgs::action::ExecuteTrajectory::Result> result) override
  {
    result->error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
};

class PlanningFallbackTest : public ::testing::Test
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
    test_node_ = rclcpp::Node::make_shared("test_planning_fallback_node", options);
    mock_node_ = rclcpp::Node::make_shared("mock_action_node");

    // Start mock action servers
    mock_move_group_ = std::make_unique<MockMoveGroupFallbackServer>(mock_node_, "move_action");
    mock_move_group_2_ = std::make_unique<MockMoveGroupFallbackServer>(mock_node_, "move_group");
    mock_execute_trajectory_ = std::make_unique<MockExecuteTrajectoryServer>(mock_node_,
        "execute_trajectory");

    // Start mock services
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

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
    executor_->add_node(mock_node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});

    // Setup BT factory
    BT::BehaviorTreeFactory factory;
    factory.registerBuilder<MoveArmAction>("MoveArm",
      [this](const std::string & name, const BT::NodeConfig & config) {
        return std::make_unique<MoveArmAction>(name, config);
      });

    blackboard_ = BT::Blackboard::create();
    blackboard_->set<rclcpp::Node::SharedPtr>("node", test_node_);

    std::string xml_txt =
      R"(
      <root BTCPP_format="4">
        <BehaviorTree ID="MainTree">
          <MoveArm name="move_arm"
                   named_pose="{named_pose}"
                   target_pose="{target_pose}"
                   z_offset="{z_offset}"
                   x_offset="{x_offset}"
                   phase="{phase}"
                   step="{step}"
                   step_index="{step_index}"/>
        </BehaviorTree>
      </root>
    )";

    tree_ = factory.createTreeFromText(xml_txt, blackboard_);
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    rclcpp::shutdown();
  }

  rclcpp::Node::SharedPtr test_node_;
  rclcpp::Node::SharedPtr mock_node_;
  std::unique_ptr<MockMoveGroupFallbackServer> mock_move_group_;
  std::unique_ptr<MockMoveGroupFallbackServer> mock_move_group_2_;
  std::unique_ptr<MockExecuteTrajectoryServer> mock_execute_trajectory_;

  rclcpp::Service<::moveit_msgs::srv::GetPlanningScene>::SharedPtr mock_get_scene_service_;
  rclcpp::Service<::moveit_msgs::srv::ApplyPlanningScene>::SharedPtr mock_apply_scene_service_;
  rclcpp::Service<::moveit_msgs::srv::QueryPlannerInterfaces>::SharedPtr mock_query_planner_service_;
  rclcpp::Service<::moveit_msgs::srv::GetPlannerParams>::SharedPtr mock_get_planner_params_service_;
  rclcpp::Service<::moveit_msgs::srv::SetPlannerParams>::SharedPtr mock_set_planner_params_service_;
  rclcpp::Service<::moveit_msgs::srv::GetPositionIK>::SharedPtr mock_compute_ik_service_;
  rclcpp::Service<::moveit_msgs::srv::GetPositionFK>::SharedPtr mock_compute_fk_service_;
  rclcpp::Service<::moveit_msgs::srv::GetStateValidity>::SharedPtr mock_state_validity_service_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  BT::Blackboard::Ptr blackboard_;
  BT::Tree tree_;
};

TEST_F(PlanningFallbackTest, NamedPoseFallsBackToOmplOnPilzFailure)
{
  blackboard_->set("named_pose", std::string("home"));
  blackboard_->set("phase", std::string("PICK_PHASE"));
  blackboard_->set("step", std::string("HOME_ARM"));
  blackboard_->set("step_index", 1);

  // Instruct mock server to fail Pilz planning requests
  mock_move_group_->set_fail_pilz(true);
  mock_move_group_->reset_counters();
  mock_move_group_2_->set_fail_pilz(true);
  mock_move_group_2_->reset_counters();

  // Tick Behavior Tree
  BT::NodeStatus status = tree_.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // Wait for the action to complete
  for (int i = 0; i < 50; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    status = tree_.tickOnce();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }

  // Fallback to OMPL should succeed
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);

  // Retrieve total attempts
  int total_pilz = mock_move_group_->get_pilz_attempts() + mock_move_group_2_->get_pilz_attempts();
  int total_ompl = mock_move_group_->get_ompl_attempts() + mock_move_group_2_->get_ompl_attempts();

  EXPECT_GT(total_pilz, 0);
  EXPECT_GT(total_ompl, 0);
}

TEST_F(PlanningFallbackTest, CartesianTargetAbortsImmediatelyOnPilzFailure)
{
  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.header.frame_id = "ur5e_base_link";
  target_pose.pose.position.x = 0.5;
  target_pose.pose.position.y = 0.0;
  target_pose.pose.position.z = 0.3;
  target_pose.pose.orientation.w = 1.0;

  blackboard_->set("target_pose", target_pose);
  blackboard_->set("z_offset", 0.0);
  blackboard_->set("x_offset", 0.0);
  blackboard_->set("phase", std::string("PICK_PHASE"));
  blackboard_->set("step", std::string("EXECUTE_PICK"));
  blackboard_->set("step_index", 4);

  // Set optimized base pose to satisfy base-to-target yaw calculations
  geometry_msgs::msg::PoseStamped base_pose;
  base_pose.header.frame_id = "map";
  base_pose.pose.position.x = 0.0;
  base_pose.pose.position.y = 0.0;
  base_pose.pose.orientation.w = 1.0;
  blackboard_->set("optimized_pick_base_pose", base_pose);

  // Instruct mock server to fail Pilz planning requests
  mock_move_group_->set_fail_pilz(true);
  mock_move_group_->reset_counters();
  mock_move_group_2_->set_fail_pilz(true);
  mock_move_group_2_->reset_counters();

  // Tick Behavior Tree
  BT::NodeStatus status = tree_.tickOnce();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  // Wait for the action to complete
  for (int i = 0; i < 50; ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    status = tree_.tickOnce();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
  }

  // Should fail immediately without falling back to OMPL
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);

  int total_pilz = mock_move_group_->get_pilz_attempts() + mock_move_group_2_->get_pilz_attempts();
  int total_ompl = mock_move_group_->get_ompl_attempts() + mock_move_group_2_->get_ompl_attempts();

  EXPECT_GT(total_pilz, 0);
  EXPECT_EQ(total_ompl, 0); // NO OMPL ATTEMPTS
}

} // namespace pick_place_orchestrator
