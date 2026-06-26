#include <string>
#include <memory>
#include <functional>
#include <thread>
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mobile_manipulator_nav
{

class IsArmStowed : public BT::ConditionNode
{
public:
  IsArmStowed(const std::string & name, const BT::NodeConfig & config)
  : BT::ConditionNode(name, config),
    is_stowed_(false),
    received_state_(false)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    if (!node_) {
      throw std::runtime_error("IsArmStowed: Could not find 'node' on blackboard");
    }

    // Create a unique sub-node name to avoid collisions
    std::string node_name = "is_arm_stowed_sub_node_" + name;
    // Replace any slashes or invalid characters in node name
    for (char & c : node_name) {
      if (c == '/' || c == ' ' || c == '.' || c == '-') {
        c = '_';
      }
    }

    sub_node_ = std::make_shared<rclcpp::Node>(node_name);
    sub_ = sub_node_->create_subscription<std_msgs::msg::String>(
      "/arm_state",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IsArmStowed::callback, this, std::placeholders::_1)
    );

    // Run the sub-node in a dedicated executor and thread to guarantee callbacks are executed
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(sub_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });
  }

  ~IsArmStowed() override
  {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    if (!received_state_) {
      RCLCPP_WARN_ONCE(node_->get_logger(),
          "IsArmStowed: No arm state received yet, defaulting to RUNNING (paused)");
      return BT::NodeStatus::RUNNING;
    }

    if (is_stowed_) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        2000,
        "IsArmStowed: Arm is ACTIVE or NOT stowed. Navigation paused."
      );
      return BT::NodeStatus::RUNNING;
    }
  }

private:
  void callback(const std_msgs::msg::String::SharedPtr msg)
  {
    received_state_ = true;
    if (msg->data == "STOWED") {
      is_stowed_ = true;
    } else {
      is_stowed_ = false;
    }
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr sub_node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  bool is_stowed_;
  bool received_state_;
};

} // namespace mobile_manipulator_nav

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<mobile_manipulator_nav::IsArmStowed>("IsArmStowed");
}
