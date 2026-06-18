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
        self.threshold = 0.05  # radians
        self.get_logger().info('Arm State Monitor started. Monitoring UR5e joint states...')

    def joint_state_callback(self, msg):
        is_stowed = True
        found_joints = 0
        for name, target_pos in self.stowed_config.items():
            if name in msg.name:
                idx = msg.name.index(name)
                current_pos = msg.position[idx]
                if abs(current_pos - target_pos) > self.threshold:
                    is_stowed = False
                found_joints += 1

        state_msg = String()
        if found_joints == len(self.stowed_config):
            state_msg.data = 'STOWED' if is_stowed else 'ACTIVE'
        else:
            state_msg.data = 'ACTIVE'

        self.pub.publish(state_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmStateMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
