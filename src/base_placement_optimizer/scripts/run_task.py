#!/usr/bin/env python3
# Copyright 2026 Mobile Manipulator Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
run_task.py — End-to-end demonstration of the Base Placement Optimizer.

Execution sequence
──────────────────
STEP 1  /optimize_placement   (base_placement_optimizer)
        Radial sampling + Nav2 costmap obstacle filter + MoveIt IK validation
        + Yoshikawa manipulability scoring → returns best (base_pose, heading).

STEP 2  /navigate_to_pose     (Nav2 bt_navigator)
        Drives the mobile base to the XY position only.
        yaw_goal_tolerance is set to 6.28 rad (full circle) so Nav2 stops
        as soon as XY is reached without any in-place heading rotation.

STEP 2.5  Heading alignment   (run_task.py, P-controller over /cmd_vel)
        The optimizer computes a heading that points the robot toward the
        target for maximum arm manipulability. Nav2 cannot smooth-align a
        ~200° heading difference without oscillating. Instead we publish
        directly to /cmd_vel with a proportional controller: angular velocity
        scales down as the error shrinks, giving jerk-free convergence.

STEP 3  /move_action          (MoveIt2 move_group)
        Plans a collision-free trajectory for the UR5e so that the
        end-effector (ur5e_tool0) reaches the original target_pose.
        Target is first transformed from 'map' → 'base_link' via TF2.
"""

import math
import time

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseStamped, Twist
from shape_msgs.msg import SolidPrimitive

from base_placement_optimizer.action import OptimizePlacement
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    PositionConstraint,
)
from nav2_msgs.action import NavigateToPose


class RunTaskNode(Node):
    """Orchestrates: stow arm → optimize → navigate (XY) → align heading → MoveIt arm reach."""

    # P-controller gains for heading alignment
    HEADING_KP = 0.8          # rad/s per rad of error
    HEADING_MAX_VEL = 0.40    # rad/s — caps max angular speed
    HEADING_MIN_VEL = 0.05    # rad/s — minimum to overcome static friction
    HEADING_TOLERANCE = 0.05  # rad (~2.8°) — done when error < this
    HEADING_TIMEOUT = 20.0    # seconds — safety abort

    def __init__(self):
        super().__init__('run_movement_task_client')

        self.optimize_client = ActionClient(
            self, OptimizePlacement, 'optimize_placement')
        self.navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')
        self.move_group_client = ActionClient(
            self, MoveGroup, 'move_action')

        # Direct cmd_vel_nav publisher — used ONLY after Nav2 goal is complete
        # We publish to /cmd_vel_nav so the velocity_smoother processes it
        # and doesn't override us with its own deadband zeros on /cmd_vel.
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('RunTaskNode initialised.')

    # ── Private helpers ───────────────────────────────────────────────────────

    def _call_action(self, client, goal, label):
        """Send goal, block until done, return wrapped result. Raises on rejection."""
        self.get_logger().info(f'[{label}] Sending goal …')
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            raise RuntimeError(f'[{label}] Goal REJECTED by server.')
        self.get_logger().info(f'[{label}] Accepted. Waiting for result …')
        res_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result()

    @staticmethod
    def _yaw_from_quaternion(q) -> float:
        """Extract yaw (rad) from a geometry_msgs Quaternion."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-π, π]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _get_current_yaw(self) -> float | None:
        """Return current robot yaw from TF (map→base_footprint), or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
            return self._yaw_from_quaternion(t.transform.rotation)
        except Exception:
            return None

    def _align_heading(self, target_yaw: float):
        """
        P-controller in-place rotation to target_yaw.

        Publishes to /cmd_vel directly. Safe to call only after Nav2 has
        reported its goal as complete (no active Nav2 goal → no cmd_vel
        conflict). Uses a proportional + minimum-velocity law:

            ω = clip(Kp * error, ±MIN_VEL, ±MAX_VEL)

        This naturally decelerates as the error shrinks → no overshoot.
        """
        self.get_logger().info(
            f'STEP 2.5 — Aligning heading to {math.degrees(target_yaw):.1f}°'
            f' (P-controller, max={self.HEADING_MAX_VEL} rad/s) …')

        deadline = time.monotonic() + self.HEADING_TIMEOUT
        twist = Twist()

        while time.monotonic() < deadline:
            current_yaw = self._get_current_yaw()
            if current_yaw is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                continue

            error = self._wrap_angle(target_yaw - current_yaw)

            if abs(error) < self.HEADING_TOLERANCE:
                break

            # Proportional angular velocity, clamped and with minimum kick
            raw = self.HEADING_KP * error
            if abs(raw) < self.HEADING_MIN_VEL:
                raw = math.copysign(self.HEADING_MIN_VEL, error)
            omega = max(-self.HEADING_MAX_VEL, min(self.HEADING_MAX_VEL, raw))

            twist.angular.z = omega
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.05)

        # Send explicit stop
        self.cmd_vel_pub.publish(Twist())
        rclpy.spin_once(self, timeout_sec=0.1)

        final_yaw = self._get_current_yaw()
        if final_yaw is not None:
            residual = abs(self._wrap_angle(target_yaw - final_yaw))
            self.get_logger().info(
                f'STEP 2.5 ✓  Heading aligned. '
                f'Residual error: {math.degrees(residual):.1f}°')
        else:
            self.get_logger().warn('STEP 2.5 — Could not verify final heading via TF.')

    # ── Main task ─────────────────────────────────────────────────────────────

    def run_task(self, target_x: float, target_y: float, target_z: float):
        # ── Wait for servers ──────────────────────────────────────────────────
        self.get_logger().info('Waiting for action servers …')
        self.optimize_client.wait_for_server()
        self.navigate_client.wait_for_server()
        self.move_group_client.wait_for_server()
        self.get_logger().info('All servers ready.')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 0 — Stow the Arm
        # Nav2's Behavior Tree requires the arm to be stowed before it will
        # navigate. We command the stowed joint configuration via MoveIt2.
        # ══════════════════════════════════════════════════════════════════════
        self.get_logger().info('STEP 0 — Stowing arm for safe navigation …')
        stow_goal = MoveGroup.Goal()
        stow_req = MotionPlanRequest()
        stow_req.group_name = 'robot_arm'
        stow_req.num_planning_attempts = 3
        stow_req.allowed_planning_time = 5.0
        stow_req.max_velocity_scaling_factor = 0.5
        stow_req.max_acceleration_scaling_factor = 0.5

        # We can use the named target 'stowed' which is defined in our SRDF
        stow_goal.request = stow_req
        stow_goal.request.workspace_parameters.header.frame_id = 'base_footprint'
        
        # In MoveIt2 action, setting the target state via a named pose requires
        # setting it in the goal request constraints, but MoveGroup action allows
        # leaving constraints empty and using group_name if we provide a joint 
        # constraint. We'll specify the exact joint values to be safe.
        from moveit_msgs.msg import JointConstraint
        stowed_joints = {
            'ur5e_shoulder_pan_joint': 0.0,
            'ur5e_shoulder_lift_joint': -1.5708,
            'ur5e_elbow_joint': 1.5708,
            'ur5e_wrist_1_joint': 0.0,
            'ur5e_wrist_2_joint': 0.0,
            'ur5e_wrist_3_joint': 0.0
        }
        goal_constraints = Constraints()
        goal_constraints.name = 'stowed'
        for j_name, j_val in stowed_joints.items():
            jc = JointConstraint()
            jc.joint_name = j_name
            jc.position = j_val
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)

        stow_req.goal_constraints = [goal_constraints]
        stow_goal.request = stow_req
        stow_goal.planning_options.plan_only = False
        stow_goal.planning_options.replan = True
        stow_goal.planning_options.replan_attempts = 3

        stow_wrapped = self._call_action(self.move_group_client, stow_goal, 'stow_arm')
        stow_res = stow_wrapped.result
        if stow_res.error_code.val != 1:
            self.get_logger().warn(f'MoveIt2 stowing reported error_code={stow_res.error_code.val}. Continuing anyway...')
        self.get_logger().info('STEP 0 ✓  Arm stowed.')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 1 — Base Placement Optimizer
        # ══════════════════════════════════════════════════════════════════════
        opt_goal = OptimizePlacement.Goal()
        opt_goal.target_pose.header.frame_id = 'map'
        opt_goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        opt_goal.target_pose.pose.position.x = target_x
        opt_goal.target_pose.pose.position.y = target_y
        opt_goal.target_pose.pose.position.z = target_z
        opt_goal.target_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'STEP 1 — Optimizing for target ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})')

        wrapped = self._call_action(self.optimize_client, opt_goal, 'optimize_placement')
        result = wrapped.result
        if not result.success:
            raise RuntimeError(f'Optimizer failed: {result.error_reason}')

        base_pose: PoseStamped = result.base_pose
        target_yaw = self._yaw_from_quaternion(base_pose.pose.orientation)

        self.get_logger().info(
            f'STEP 1 ✓  base_pose=({base_pose.pose.position.x:.3f}, '
            f'{base_pose.pose.position.y:.3f}), '
            f'heading={math.degrees(target_yaw):.1f}° | '
            f'score={result.composite_score:.3f} '
            f'(manip={result.manipulability_score:.4f}, '
            f'dist={result.path_distance:.2f} m)')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 2 — Navigate to XY position only
        # Nav2 yaw_goal_tolerance=6.28 rad → success on XY arrival alone.
        # No in-place spinning → fast, direct path to the optimal stance.
        # ══════════════════════════════════════════════════════════════════════
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = base_pose  # position used; heading handled in step 2.5

        self.get_logger().info(
            f'STEP 2 — Navigating to XY '
            f'({base_pose.pose.position.x:.3f}, {base_pose.pose.position.y:.3f})'
            f' [heading alignment deferred to step 2.5]')

        self._call_action(self.navigate_client, nav_goal, 'navigate_to_pose')
        
        # Actually verify Nav2 succeeded. If the arm wasn't stowed or path failed,
        # we must abort instead of executing arm reach from the wrong location.
        # Wait, the action client wrapper doesn't currently return the status code.
        # Let's ensure the robot is actually somewhat close to the goal.
        final_x = 0.0
        final_y = 0.0
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            final_x = t.transform.translation.x
            final_y = t.transform.translation.y
            dist = math.hypot(final_x - base_pose.pose.position.x, final_y - base_pose.pose.position.y)
            if dist > 0.5:
                raise RuntimeError(f'Nav2 failed to reach goal! Distance to goal: {dist:.2f}m')
        except Exception as e:
            self.get_logger().warn(f'Could not verify Nav2 arrival via TF: {e}')

        self.get_logger().info('STEP 2 ✓  Base arrived at optimal XY position.')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 2.5 — Heading alignment (P-controller, /cmd_vel)
        # Now that Nav2 has no active goal it stops publishing cmd_vel.
        # Our P-controller rotates the robot smoothly to the target heading
        # without oscillation: ω ∝ error, naturally decelerating to zero.
        # ══════════════════════════════════════════════════════════════════════
        self._align_heading(target_yaw)

        # ══════════════════════════════════════════════════════════════════════
        # STEP 3 — MoveIt2: plan + execute arm reach to target
        # Transform target from map → base_link (now that robot is in position
        # and correctly oriented), then issue a MotionPlanRequest with a
        # PositionConstraint on ur5e_tool0.
        # ══════════════════════════════════════════════════════════════════════
        self.get_logger().info(
            'STEP 3 — Transforming target from map → base_link …')

        target_in_map = PoseStamped()
        target_in_map.header.frame_id = 'map'
        target_in_map.header.stamp = self.get_clock().now().to_msg()
        target_in_map.pose.position.x = target_x
        target_in_map.pose.position.y = target_y
        target_in_map.pose.position.z = target_z
        target_in_map.pose.orientation.w = 1.0

        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'map',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0))
            target_in_base: PoseStamped = \
                tf2_geometry_msgs.do_transform_pose_stamped(target_in_map, transform)
        except Exception as ex:
            raise RuntimeError(f'TF lookup map→base_link failed: {ex}')

        self.get_logger().info(
            f'Target in base_link: '
            f'x={target_in_base.pose.position.x:.3f}, '
            f'y={target_in_base.pose.position.y:.3f}, '
            f'z={target_in_base.pose.position.z:.3f}')

        # Build MoveGroup.Goal
        move_goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'robot_arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]

        region = BoundingVolume()
        region.primitives = [sphere]
        target_local = Pose()
        target_local.position.x = target_in_base.pose.position.x
        target_local.position.y = target_in_base.pose.position.y
        target_local.position.z = target_in_base.pose.position.z
        target_local.orientation.w = 1.0
        region.primitive_poses = [target_local]

        pos_c = PositionConstraint()
        pos_c.header.frame_id = 'base_link'
        pos_c.link_name = 'ur5e_tool0'
        pos_c.constraint_region = region
        pos_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.name = 'reach_target'
        goal_constraints.position_constraints = [pos_c]

        req.goal_constraints = [goal_constraints]
        move_goal.request = req
        move_goal.planning_options.plan_only = False
        move_goal.planning_options.replan = True
        move_goal.planning_options.replan_attempts = 3

        self.get_logger().info('STEP 3 — MoveIt2 planning + executing via /move_action …')

        mv_wrapped = self._call_action(self.move_group_client, move_goal, 'move_action')
        mv_result = mv_wrapped.result

        if mv_result.error_code.val != 1:
            raise RuntimeError(
                f'MoveIt2 failed — error_code={mv_result.error_code.val}')

        self.get_logger().info('STEP 3 ✓  Arm reached target via MoveIt2. Full task complete!')


def main(args=None):
    rclpy.init(args=args)
    node = RunTaskNode()
    try:
        node.run_task(target_x=-2.0, target_y=0.0, target_z=0.5)
    except RuntimeError as exc:
        node.get_logger().error(f'Task FAILED: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
