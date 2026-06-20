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
        + Yoshikawa manipulability scoring → returns best base_pose.

STEP 2  /navigate_to_pose     (Nav2 bt_navigator)
        Drives the mobile base to the optimizer-selected pose.

STEP 3  /move_action          (MoveIt2 move_group)
        Plans a collision-free trajectory for the UR5e so that the
        end-effector (ur5e_tool0) reaches the original target_pose.
        Target is first transformed from 'map' → 'base_link' via TF2
        so MoveIt's planner works in the robot's own frame.
"""

import rclpy
import rclpy.duration
import rclpy.time
from rclpy.action import ActionClient
from rclpy.node import Node

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import Pose, PoseStamped
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
    """Orchestrates: optimize → navigate → MoveIt arm reach."""

    def __init__(self):
        super().__init__('run_movement_task_client')

        # ── Action clients ────────────────────────────────────────────────────
        # Step 1: the base placement optimizer action server
        self.optimize_client = ActionClient(
            self, OptimizePlacement, 'optimize_placement')

        # Step 2: Nav2 navigation-to-pose action server
        self.navigate_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        # Step 3: MoveIt2 MoveGroup action server (plan + execute via controllers)
        self.move_group_client = ActionClient(
            self, MoveGroup, 'move_action')

        # TF2 buffer used to transform the target from map → base_link
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info('RunTaskNode initialised.')

    # ── Private helpers ───────────────────────────────────────────────────────

    def _call_action(self, client, goal, label):
        """Send a goal, block until done, return the wrapped result. Raises on rejection."""
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

    # ── Main task ─────────────────────────────────────────────────────────────

    def run_task(self, target_x: float, target_y: float, target_z: float):
        # ── Wait for all three action servers to come up ──────────────────────
        self.get_logger().info('Waiting for action servers …')
        self.optimize_client.wait_for_server()
        self.navigate_client.wait_for_server()
        self.move_group_client.wait_for_server()
        self.get_logger().info('All servers ready.')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 1 — Base Placement Optimizer
        # The node internally:
        #   • samples 16 radial candidates around (target_x, target_y)
        #   • filters each against the Nav2 global costmap (LETHAL / INSCRIBED)
        #   • calls robot_state.setFromIK() for each surviving candidate
        #   • scores survivors via Yoshikawa manipulability + inverse distance
        #   • returns the highest-scoring, collision-free, IK-valid base_pose
        # ══════════════════════════════════════════════════════════════════════
        opt_goal = OptimizePlacement.Goal()
        opt_goal.target_pose.header.frame_id = 'map'
        opt_goal.target_pose.header.stamp = self.get_clock().now().to_msg()
        opt_goal.target_pose.pose.position.x = target_x
        opt_goal.target_pose.pose.position.y = target_y
        opt_goal.target_pose.pose.position.z = target_z
        opt_goal.target_pose.pose.orientation.w = 1.0

        self.get_logger().info(
            f'STEP 1 — Optimizing base placement for target '
            f'({target_x:.2f}, {target_y:.2f}, {target_z:.2f}) in map frame')

        wrapped = self._call_action(
            self.optimize_client, opt_goal, 'optimize_placement')
        result = wrapped.result

        if not result.success:
            raise RuntimeError(f'Optimizer failed: {result.error_reason}')

        base_pose: PoseStamped = result.base_pose
        self.get_logger().info(
            f'STEP 1 ✓  Optimal base_pose: '
            f'x={base_pose.pose.position.x:.3f}, '
            f'y={base_pose.pose.position.y:.3f} | '
            f'composite_score={result.composite_score:.3f} '
            f'(manipulability={result.manipulability_score:.4f}, '
            f'reach_distance={result.path_distance:.2f} m)')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 2 — Navigate mobile base to the optimizer-selected pose
        # Nav2 BehaviorTree planner computes a global path, the controller
        # tracks it, and bt_navigator reports success once within goal tolerance.
        # ══════════════════════════════════════════════════════════════════════
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = base_pose  # the optimizer's output drives navigation

        self.get_logger().info(
            f'STEP 2 — Navigating to '
            f'({base_pose.pose.position.x:.3f}, {base_pose.pose.position.y:.3f})')

        self._call_action(self.navigate_client, nav_goal, 'navigate_to_pose')
        self.get_logger().info('STEP 2 ✓  Mobile base arrived at optimal pose.')

        # ══════════════════════════════════════════════════════════════════════
        # STEP 3 — MoveIt2: plan + execute arm reach to the target
        # The base is now at the IK-validated pose.  We ask MoveIt's
        # move_group server to plan a collision-aware trajectory that moves
        # ur5e_tool0 to the target object location.
        #
        # The target must be expressed in 'base_link' (the URDF root /
        # planning frame).  We use TF2 to look up map → base_link *after*
        # navigation has completed, so the transform reflects the robot's
        # actual pose.
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
                'base_link',          # target frame
                'map',                # source frame
                rclpy.time.Time(),    # latest available
                timeout=rclpy.duration.Duration(seconds=5.0))
            target_in_base: PoseStamped = \
                tf2_geometry_msgs.do_transform_pose_stamped(
                    target_in_map, transform)
        except Exception as ex:
            raise RuntimeError(f'TF lookup map→base_link failed: {ex}')

        self.get_logger().info(
            f'Target in base_link: '
            f'x={target_in_base.pose.position.x:.3f}, '
            f'y={target_in_base.pose.position.y:.3f}, '
            f'z={target_in_base.pose.position.z:.3f}')

        # ── Build MoveGroup.Goal → MotionPlanRequest ─────────────────────────
        # group   : robot_arm   (ur5e_base_link → ur5e_tool0 as per SRDF)
        # tip link: ur5e_tool0  (declared as end-effector in SRDF)
        # constraint: position-only sphere (5 cm tolerance) at target
        # plan_only = False → MoveIt plans AND executes via joint_trajectory_controller
        move_goal = MoveGroup.Goal()

        req = MotionPlanRequest()
        req.group_name = 'robot_arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # Position constraint: ur5e_tool0 inside a 5 cm sphere at target
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # 5 cm tolerance

        region = BoundingVolume()
        region.primitives = [sphere]
        target_local = Pose()
        target_local.position.x = target_in_base.pose.position.x
        target_local.position.y = target_in_base.pose.position.y
        target_local.position.z = target_in_base.pose.position.z
        target_local.orientation.w = 1.0
        region.primitive_poses = [target_local]

        pos_c = PositionConstraint()
        pos_c.header.frame_id = 'base_link'   # same as planning frame
        pos_c.link_name = 'ur5e_tool0'        # end-effector link (SRDF)
        pos_c.constraint_region = region
        pos_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.name = 'reach_target'
        goal_constraints.position_constraints = [pos_c]

        req.goal_constraints = [goal_constraints]

        move_goal.request = req
        move_goal.planning_options.plan_only = False   # plan + execute
        move_goal.planning_options.replan = True
        move_goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            'STEP 3 — MoveIt2: requesting plan + execution '
            'via /move_action …')

        mv_wrapped = self._call_action(
            self.move_group_client, move_goal, 'move_action')
        mv_result = mv_wrapped.result

        # MoveIt error_code.val: 1 = SUCCESS (moveit_msgs/MoveItErrorCodes)
        if mv_result.error_code.val != 1:
            raise RuntimeError(
                f'MoveIt2 failed — error_code={mv_result.error_code.val} '
                f'(1=SUCCESS, -1=FAILURE, see MoveItErrorCodes.msg)')

        self.get_logger().info(
            'STEP 3 ✓  Arm reached the target via MoveIt2. '
            'Full task complete!')


def main(args=None):
    rclpy.init(args=args)
    node = RunTaskNode()
    try:
        # Object placed at (x=-2.0, y=0.0, z=0.5) in map frame.
        # The optimizer will find the best base stance; Nav2 will drive there;
        # MoveIt2 will plan and execute the reach.
        node.run_task(target_x=-2.0, target_y=0.0, target_z=0.5)
    except RuntimeError as exc:
        node.get_logger().error(f'Task FAILED: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
