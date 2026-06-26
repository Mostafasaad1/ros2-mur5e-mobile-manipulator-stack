#!/usr/bin/env python3

import sys

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class TestMovementNode(Node):

    def __init__(self):
        super().__init__('test_movement')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # Timers and counters
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.start_time = None
        self.duration_seconds = 5.0
        self.arm_command_sent = False

        self.get_logger().info(
            'Test movement node initialized. Starting concurrent base and arm test.'
        )

    def timer_callback(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return

        if self.start_time is None:
            self.start_time = now
            self.get_logger().info(
                f'Start time initialized to simulation time: {now.nanoseconds / 1e9}s'
            )
            return

        elapsed = (now - self.start_time).nanoseconds / 1e9

        if elapsed < self.duration_seconds:
            # 1. Drive the base concurrently (move forward in a curve)
            twist = TwistStamped()
            twist.header.stamp = now.to_msg()
            twist.header.frame_id = 'base_footprint'
            twist.twist.linear.x = 0.2
            twist.twist.angular.z = 0.2
            self.cmd_vel_pub.publish(twist)

            # 2. Publish arm command once
            if not self.arm_command_sent:
                self.send_arm_trajectory()
                self.arm_command_sent = True
        else:
            # Test duration finished: stop the base and shutdown
            self.get_logger().info('Test duration completed. Stopping base.')
            stop_twist = TwistStamped()
            stop_twist.header.stamp = now.to_msg()
            stop_twist.header.frame_id = 'base_footprint'
            self.cmd_vel_pub.publish(stop_twist)
            self.timer.destroy()
            sys.exit(0)

    def send_arm_trajectory(self):
        self.get_logger().info('Sending joint trajectory command to UR5e arm...')
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [
            'ur5e_shoulder_pan_joint',
            'ur5e_shoulder_lift_joint',
            'ur5e_elbow_joint',
            'ur5e_wrist_1_joint',
            'ur5e_wrist_2_joint',
            'ur5e_wrist_3_joint'
        ]

        point = JointTrajectoryPoint()
        # Move the arm to a safe upright pose over 3 seconds
        point.positions = [0.0, -1.57, 1.57, 0.0, 0.0, 0.0]
        point.time_from_start = Duration(sec=3, nanosec=0)
        msg.points.append(point)

        self.arm_pub.publish(msg)
        self.get_logger().info('Joint trajectory published.')


def main(args=None):
    rclpy.init(args=args)
    node = TestMovementNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure the robot base is stopped if interrupted
        stop_node = rclpy.create_node('stop_base_node')
        stop_pub = stop_node.create_publisher(TwistStamped, '/cmd_vel', 10)
        stop_twist = TwistStamped()
        stop_twist.header.stamp = stop_node.get_clock().now().to_msg()
        stop_twist.header.frame_id = 'base_footprint'
        stop_pub.publish(stop_twist)
        stop_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
