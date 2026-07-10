#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointMonitor(Node):
    def __init__(self):
        super().__init__('joint_monitor')
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )
        self.get_logger().info('Joint monitor started.')

    def callback(self, msg):
        # Look for finger_left_joint and finger_right_joint
        out = {}
        for name, pos in zip(msg.name, msg.position):
            if 'finger' in name or 'ur5e' in name:
                out[name] = pos
        # Print only if finger joints are present
        if 'finger_left_joint' in out:
            print(f"Joints: left={out.get('finger_left_joint', 0.0):.6f}, right={out.get('finger_right_joint', 0.0):.6f}")

def main():
    rclpy.init()
    node = JointMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
