#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class TestGripperNode(Node):
    def __init__(self, command):
        super().__init__('test_gripper')
        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )
        
        # Determine positions based on command (0.0 = closed, 0.04 = open)
        if command == 'open':
            pos = [0.04, 0.04]
        else:
            pos = [0.00, 0.00]
            
        self.get_logger().info(f'Commanding gripper to: {command} (positions: {pos})')
        
        # Periodically publish command to ensure the controller manager processes it
        self.timer = self.create_timer(0.5, lambda: self.send_command(pos))
        self.command_sent = False

    def send_command(self, pos):
        if self.command_sent:
            return
        
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = ['finger_left_joint', 'finger_right_joint']
        
        point = JointTrajectoryPoint()
        point.positions = pos
        point.time_from_start = Duration(sec=1, nanosec=0)
        msg.points.append(point)
        
        self.gripper_pub.publish(msg)
        self.get_logger().info('Published gripper command.')
        self.command_sent = True
        
        # Exit after a brief delay to allow message to go out
        self.create_timer(1.0, lambda: sys.exit(0))

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in ['open', 'close']:
        print("Usage: test_gripper.py [open|close]")
        sys.exit(1)
        
    command = sys.argv[1]
    rclpy.init()
    node = TestGripperNode(command)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
