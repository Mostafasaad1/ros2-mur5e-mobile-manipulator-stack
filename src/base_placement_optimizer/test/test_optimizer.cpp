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

#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "base_placement_optimizer/optimizer_node.hpp"

class OptimizerNodeTest : public ::testing::Test {
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<base_placement_optimizer::BasePlacementOptimizerNode>();

    // We need an executor to spin the node
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

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

  std::shared_ptr<base_placement_optimizer::BasePlacementOptimizerNode> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
};

TEST_F(OptimizerNodeTest, ActionServerAcceptsGoal) {
  auto client_node = rclcpp::Node::make_shared("test_action_client");
  auto action_client = rclcpp_action::create_client<base_placement_optimizer::action::OptimizePlacement>(
    client_node, "optimize_placement");

  bool server_exists = action_client->wait_for_action_server(std::chrono::seconds(2));
  ASSERT_TRUE(server_exists);

  auto goal_msg = base_placement_optimizer::action::OptimizePlacement::Goal();
  goal_msg.target_pose.header.frame_id = "map";
  goal_msg.target_pose.pose.position.x = 1.0;

  auto send_goal_options = rclcpp_action::Client<base_placement_optimizer::action::OptimizePlacement>::SendGoalOptions();

  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);

  ASSERT_EQ(rclcpp::spin_until_future_complete(client_node, goal_handle_future,
    std::chrono::seconds(2)),
            rclcpp::FutureReturnCode::SUCCESS);

  auto goal_handle = goal_handle_future.get();
  ASSERT_NE(goal_handle, nullptr);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
