# Base Placement Optimizer

This package provides a ROS 2 Action Server for computing optimal base placement poses for a mobile manipulator. It takes a target object pose in the world and returns a kinematically reachable, collision-safe base pose.

## Action Server

- **Action Name:** `optimize_placement`
- **Type:** `base_placement_optimizer/action/OptimizePlacement`

## Parameters

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `map_frame` | string | `map` | The global reference frame. |
| `planning_group` | string | `robot_arm` | The MoveIt planning group for the manipulator. |
| `costmap_topic` | string | `/global_costmap/costmap` | Topic name for the Nav2 OccupancyGrid costmap. |
| `reach_radius` | double | `0.7` | The radial distance from the target object to sample candidate base poses. |
| `angular_samples` | int | `16` | Number of radial samples around the target object to evaluate. |
| `ik_timeout` | double | `0.002` | Timeout in seconds for each inverse kinematics evaluation. |
| `alpha` | double | `0.5` | Weight factor `[0.0, 1.0]` balancing manipulability and path distance in the scoring function. |

## Quickstart

Launch the optimizer node:
```bash
ros2 launch base_placement_optimizer optimizer.launch.py
```

Send a test goal:
```bash
ros2 action send_goal /optimize_placement base_placement_optimizer/action/OptimizePlacement "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"
```
