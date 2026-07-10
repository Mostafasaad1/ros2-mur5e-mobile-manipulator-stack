# Base Placement Optimizer

This package provides a ROS 2 Action Server for computing optimal base placement poses for a mobile manipulator. It takes a target object pose in the world and returns a kinematically reachable, collision-safe base pose.

For a detailed explanation of the system architecture, algorithm flowcharts, and how to run the full end-to-end example, please read **[ARCHITECTURE.md](ARCHITECTURE.md)**.

## Action Server

- **Action Name:** `optimize_placement`
- **Type:** `base_placement_optimizer/action/OptimizePlacement`

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `map_frame` | string | `map` | The global reference frame. |
| `planning_group` | string | `robot_arm` | The MoveIt planning group for the manipulator. |
| `costmap_topic` | string | `/global_costmap/costmap` | Topic name for the Nav2 OccupancyGrid costmap. |
| `reach_radius` | double | `0.80` | The nominal radial distance from the target object to sample candidate base poses. |
| `angular_samples` | int | `16` | Number of radial samples around the target object to evaluate. |
| `ik_timeout` | double | `0.050` | Timeout in seconds for each inverse kinematics evaluation. |
| `alpha` | double | `0.5` | Weight factor `[0.0, 1.0]` balancing manipulability and path distance in the scoring function. |
| `max_nav_distance` | double | `12.0` | Discards candidates farther than this from the robot's current pose to avoid navigation planning outside the costmap. |
| `robot_base_frame` | string | `base_footprint` | The frame of the mobile robot base. |
| `target_clearance_radius` | double | `0.80` | Minimum radial distance candidates must maintain from the target object to avoid physical collision with tables. |
| `grasp_x_offset` | double | `0.20` | The tool X offset from the object center utilized during grasp validation. |
| `grasp_z_offset` | double | `0.20` | The tool Z offset from the object center utilized during grasp validation. |

## Quickstart

Launch the optimizer node:
```bash
ros2 launch base_placement_optimizer optimizer.launch.py
```

Send a test goal:
```bash
ros2 action send_goal /optimize_placement base_placement_optimizer/action/OptimizePlacement "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"
```
