# Launch Command Reference

Complete listing of every `ros2 launch` command available in this workspace, grouped by package.
Build the workspace first and source the install overlay before running any command.

```bash
colcon build --symlink-install
source install/setup.bash
```

> **Tip** — arguments shown as `arg:=value` can be omitted when you want the default.

---

## Table of Contents

- [pick\_place\_orchestrator](#pick_place_orchestrator) — Mission executive (start here)
- [base\_placement\_optimizer](#base_placement_optimizer) — IK-aware base pose server
- [mobile\_manipulator\_gazebo](#mobile_manipulator_gazebo) — Simulation world
- [mobile\_manipulator\_description](#mobile_manipulator_description) — Robot URDF tools
- [mobile\_manipulator\_moveit\_config](#mobile_manipulator_moveit_config) — MoveIt 2
- [mobile\_manipulator\_nav](#mobile_manipulator_nav) — Nav2 navigation
- [mobile\_manipulator\_slam](#mobile_manipulator_slam) — SLAM mapping
- [mir\_description](#mir_description) — MiR 250 base visualiser
- [ur\_description](#ur_description) — UR5e arm tools

---

## `pick_place_orchestrator`

> **Master mission executive.** Hosts the `PickPlaceMission` action server and drives the 12-step pick-and-place BehaviorTree. Depends on the full simulation + MoveIt + Nav2 + optimizer stack being up.

### `validation.launch.py` ⭐ Unified entry point

Brings up the **entire system** in one command: Gazebo simulation → Nav2 → MoveIt 2 → base placement optimizer → orchestrator (with a 10 s startup delay).

```bash
# Default: headless Gazebo + RViz
ros2 launch pick_place_orchestrator validation.launch.py

# With Gazebo GUI
ros2 launch pick_place_orchestrator validation.launch.py headless:=false

# Headless, no RViz (CI / server use)
ros2 launch pick_place_orchestrator validation.launch.py headless:=true use_rviz:=false
```

| Argument | Default | Description |
|---|---|---|
| `headless` | `true` | Run Gazebo server-only (no GUI) |
| `use_rviz` | `true` | Launch RViz2 |

---

### `orchestrator.launch.py`

Starts **only** the orchestrator node. Assumes everything else (sim, MoveIt, Nav2, optimizer) is already running.

```bash
ros2 launch pick_place_orchestrator orchestrator.launch.py
```

No configurable arguments. Loads `behavior_trees/pick_place_mission.xml` automatically.

---

## `base_placement_optimizer`

> **IK-aware base pose action server.** Samples candidate poses around a target, scores them by IK manipulability and path cost, and returns the best reachable base pose.

### `optimizer_demo.launch.py`

Full demo stack: Gazebo simulation + Nav2 + MoveIt 2 + optimizer node. The recommended way to test the optimizer in isolation.

```bash
# Default: headless + RViz
ros2 launch base_placement_optimizer optimizer_demo.launch.py

# Show Gazebo GUI
ros2 launch base_placement_optimizer optimizer_demo.launch.py headless:=false

# No RViz
ros2 launch base_placement_optimizer optimizer_demo.launch.py use_rviz:=false
```

| Argument | Default | Description |
|---|---|---|
| `headless` | `true` | Gazebo headless mode |
| `use_rviz` | `true` | Launch RViz2 |

**Test with:**

```bash
ros2 action send_goal /optimize_placement \
  base_placement_optimizer/action/OptimizePlacement \
  "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}}}}"
```

---

### `optimizer.launch.py`

Starts the optimizer node alone (no simulation). Requires an already-running `/global_costmap/costmap` and MoveIt move_group.

```bash
ros2 launch base_placement_optimizer optimizer.launch.py
```

No configurable arguments. Runs with `use_sim_time:=true`.

---

## `mobile_manipulator_gazebo`

> **Simulation world and hardware bridges.** Spawns the robot into Gazebo Harmonic, starts all `ros_gz_bridge` instances (cmd_vel, odom, tf, joint_states, camera/points), and sequences the ros2_control controller spawners.

### `simulation.launch.py`

Core Gazebo launch. Spawns the robot and brings up all bridges and controllers. Included by higher-level launchers.

```bash
# Default world, with GUI
ros2 launch mobile_manipulator_gazebo simulation.launch.py

# Choose a different world
ros2 launch mobile_manipulator_gazebo simulation.launch.py world:=simple_maze.sdf

# Headless
ros2 launch mobile_manipulator_gazebo simulation.launch.py headless:=true

# Custom spawn position
ros2 launch mobile_manipulator_gazebo simulation.launch.py x_pose:=-2.0 y_pose:=1.5 z_pose:=0.15
```

| Argument | Default | Description |
|---|---|---|
| `world` | `nav_workspace.sdf` | SDF world file (relative to package `worlds/`) |
| `headless` | `false` | Gazebo server-only mode |
| `use_sim_time` | `true` | Use simulation clock |
| `x_pose` | `-4.0` | Robot spawn X (m) |
| `y_pose` | `0.0` | Robot spawn Y (m) |
| `z_pose` | `0.15` | Robot spawn Z (m) |

---

## `mobile_manipulator_description`

> **Robot URDF / xacro tooling.** Provides the combined MiR 250 + UR5e + gripper description and helpers for visualisation and quick Gazebo tests.

### `display.launch.py`

Load the combined robot URDF in RViz with an interactive joint-state slider GUI. Useful for URDF authoring and TF validation.

```bash
ros2 launch mobile_manipulator_description display.launch.py

# Change arm type or prefix
ros2 launch mobile_manipulator_description display.launch.py ur_type:=ur5e tf_prefix:=ur5e_
```

| Argument | Default | Description |
|---|---|---|
| `ur_type` | `ur5e` | UR arm variant |
| `tf_prefix` | `ur5e_` | TF prefix for arm links |
| `use_sim_time` | `false` | Use simulation clock |

---

### `gazebo.launch.py`

Spawn the robot into a bare Gazebo world (default: `empty.sdf`). Lighter than the full `simulation.launch.py`; useful for hardware-level testing without the nav workspace.

```bash
ros2 launch mobile_manipulator_description gazebo.launch.py

# Headless with a specific world
ros2 launch mobile_manipulator_description gazebo.launch.py headless:=true world:=nav_workspace.sdf
```

| Argument | Default | Description |
|---|---|---|
| `world` | `empty.sdf` | Gazebo world file |
| `headless` | `false` | Gazebo server-only mode |
| `use_sim_time` | `true` | Use simulation clock |

---

## `mobile_manipulator_moveit_config`

> **MoveIt 2 planning configuration.** Contains the SRDF, kinematics config, and 3-D sensor pipeline for the `robot_arm` planning group (UR5e joints). Defines the `stowed` named state for safe transit.

### `demo.launch.py`

Full MoveIt 2 demo: Gazebo simulation + MoveIt move_group + optional RViz with the MotionPlanning panel.

```bash
ros2 launch mobile_manipulator_moveit_config demo.launch.py

# Headless, no RViz
ros2 launch mobile_manipulator_moveit_config demo.launch.py headless:=true use_rviz:=false
```

| Argument | Default | Description |
|---|---|---|
| `headless` | `false` | Gazebo server-only mode |
| `use_rviz` | `true` | Launch RViz2 with MotionPlanning panel |

---

### `move_group.launch.py`

Starts only the MoveIt 2 `move_group` node. Assumes simulation (or hardware) is already running.

```bash
# Against running Gazebo simulation
ros2 launch mobile_manipulator_moveit_config move_group.launch.py \
  use_sim_time:=true ros2_control_hardware_type:=gz

# Against mock hardware (offline planning)
ros2 launch mobile_manipulator_moveit_config move_group.launch.py \
  use_sim_time:=false ros2_control_hardware_type:=mock
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `ros2_control_hardware_type` | `mock` | `gz` for Gazebo, `mock` for offline |

---

### `rviz.launch.py`

Open RViz2 pre-configured with the MoveIt MotionPlanning panel.

```bash
# Standalone (offline / mock hardware)
ros2 launch mobile_manipulator_moveit_config rviz.launch.py

# Against running simulation
ros2 launch mobile_manipulator_moveit_config rviz.launch.py \
  use_sim_time:=true ros2_control_hardware_type:=gz
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |
| `ros2_control_hardware_type` | `mock` | Hardware interface type |

---

### `spawn_controllers.launch.py`

Spawns ros2_control controllers without restarting the move_group.

```bash
ros2 launch mobile_manipulator_moveit_config spawn_controllers.launch.py
```

---

## `mobile_manipulator_nav`

> **Nav2 autonomous navigation stack.** Provides AMCL localisation, map server, path planning, and a custom `is_arm_stowed` condition node that blocks navigation when the arm is out of its safe travel pose.

### `nav_bringup.launch.py`

Full navigation bringup: Gazebo simulation **+** Nav2 stack (map server, AMCL, planners, controllers, sensor bridge, RViz).

```bash
ros2 launch mobile_manipulator_nav nav_bringup.launch.py

# Headless, custom map
ros2 launch mobile_manipulator_nav nav_bringup.launch.py \
  headless:=true \
  map:=/path/to/your_map.yaml

# Custom Nav2 parameters
ros2 launch mobile_manipulator_nav nav_bringup.launch.py \
  params_file:=/path/to/nav2_params.yaml
```

| Argument | Default | Description |
|---|---|---|
| `headless` | `false` | Gazebo server-only mode |
| `use_rviz` | `true` | Launch RViz2 |
| `world` | `nav_workspace.sdf` | Gazebo world |
| `map` | `slam/maps/nav_workspace.yaml` | Pre-built map YAML |
| `params_file` | `nav/config/nav2_params.yaml` | Nav2 parameters |
| `x_pose` | `-4.0` | Robot spawn X |
| `y_pose` | `0.0` | Robot spawn Y |
| `z_pose` | `0.15` | Robot spawn Z |

---

### `navigation.launch.py`

Nav2 stack only (no Gazebo). Assumes the simulation or hardware is already publishing `/scan`, `/odom`, `/tf`.

```bash
ros2 launch mobile_manipulator_nav navigation.launch.py

# Custom map
ros2 launch mobile_manipulator_nav navigation.launch.py map:=/path/to/map.yaml

# No RViz
ros2 launch mobile_manipulator_nav navigation.launch.py use_rviz:=false
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `true` | Use simulation clock |
| `use_rviz` | `true` | Launch RViz2 |
| `map` | `slam/maps/nav_workspace.yaml` | Map YAML path |
| `params_file` | `nav/config/nav2_params.yaml` | Nav2 params |

---

### `nav2_no_collision_monitor.launch.py`

Minimal Nav2 bringup without the collision monitor. Called internally by `navigation.launch.py`.

```bash
ros2 launch mobile_manipulator_nav nav2_no_collision_monitor.launch.py \
  use_sim_time:=true params_file:=/path/to/nav2_params.yaml autostart:=true
```

---

## `mobile_manipulator_slam`

> **Online SLAM with slam_toolbox.** Builds a 2-D occupancy map in real time from LiDAR and IMU data. Used to generate the `nav_workspace.yaml` map that Nav2 loads for autonomous navigation.

### `slam_mapping.launch.py`

Gazebo simulation (simple maze world) + sensor bridge + slam_toolbox lifecycle node + RViz.

```bash
# Default (with Gazebo GUI)
ros2 launch mobile_manipulator_slam slam_mapping.launch.py

# Headless
ros2 launch mobile_manipulator_slam slam_mapping.launch.py headless:=true

# Custom spawn position
ros2 launch mobile_manipulator_slam slam_mapping.launch.py x_pose:=-2.0 y_pose:=0.0
```

| Argument | Default | Description |
|---|---|---|
| `headless` | `false` | Gazebo server-only mode |
| `use_sim_time` | `true` | Use simulation clock |
| `x_pose` | `-2.0` | Robot spawn X |
| `y_pose` | `0.0` | Robot spawn Y |

**Save the finished map:**

```bash
ros2 run nav2_map_server map_saver_cli -f src/mobile_manipulator_slam/maps/nav_workspace
```

---

## `mir_description`

> **MiR 250 base description.** Standalone URDF and RViz viewer for the MiR 250 omnidirectional mobile base.

### `display.launch.py`

```bash
ros2 launch mir_description display.launch.py
```

| Argument | Default | Description |
|---|---|---|
| `use_sim_time` | `false` | Use simulation clock |

---

## `ur_description`

> **UR5e arm description and standalone demos.** Upstream Universal Robots URDF macros and example pick-and-place scripts bundled for development and testing.

### `view_ur.launch.py`

Visualise the UR5e arm in RViz with joint sliders.

```bash
ros2 launch ur_description view_ur.launch.py ur_type:=ur5e
```

---

### `pick_place_demo.launch.py`

Standalone UR5e pick-and-place demo (arm only, no mobile base).

```bash
ros2 launch ur_description pick_place_demo.launch.py
```

---

### `pick_place_demo_interactive.launch.py`

Interactive version of the arm-only pick-and-place demo.

```bash
ros2 launch ur_description pick_place_demo_interactive.launch.py
```
