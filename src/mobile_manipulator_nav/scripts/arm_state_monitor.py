#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ArmStateMonitor(Node):
    def __init__(self):
        super().__init__('arm_state_monitor')
        self.pub = self.create_publisher(String, '/arm_state', 10)
        self.sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Stowed configuration joint positions
        self.stowed_config = {
            'ur5e_shoulder_pan_joint': 0.0,
            'ur5e_shoulder_lift_joint': -1.5708,
            'ur5e_elbow_joint': 1.5708,
            'ur5e_wrist_1_joint': 0.0,
            'ur5e_wrist_2_joint': 0.0,
            'ur5e_wrist_3_joint': 0.0
        }
        self.current_positions = {}
        self.threshold = 0.25  # radians
        self.get_logger().info('Arm State Monitor started. Monitoring UR5e joint states...')

    def joint_state_callback(self, msg):
        # Update our cached positions with the joints in this message
        for i, name in enumerate(msg.name):
            if name in self.stowed_config:
                self.current_positions[name] = msg.position[i]

        # If we have not received positions for all 6 arm joints, default to ACTIVE
        if len(self.current_positions) < len(self.stowed_config):
            state_msg = String()
            state_msg.data = 'ACTIVE'
            self.pub.publish(state_msg)
            return

        is_stowed = True
        for name, target_pos in self.stowed_config.items():
            current_pos = self.current_positions.get(name, 0.0)
            if abs(current_pos - target_pos) > self.threshold:
                is_stowed = False
                break

        state_msg = String()
        state_msg.data = 'STOWED' if is_stowed else 'ACTIVE'
        self.pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
