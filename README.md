# MUR5e (UR5e on MiR100)

<p align="center">
  <img src="https://github.com/user-attachments/assets/3a18e1ab-bb63-461e-99d1-662592d28470" width="220" alt="MUR5e Robot Icon" />
</p>

<p align="center">
  <img src="https://img.shields.io/badge/ROS-2_Jazzy-blue?style=flat-square&logo=ros&logoColor=white" alt="ROS 2 Jazzy" />
  <img src="https://img.shields.io/badge/Gazebo-Harmonic-orange?style=flat-square&logo=gazebo&logoColor=white" alt="Gazebo Harmonic" />
  <img src="https://img.shields.io/badge/MoveIt-2-blue?style=flat-square" alt="MoveIt 2" />
  <img src="https://img.shields.io/badge/Nav2-Jazzy-red?style=flat-square" alt="Nav2" />
  <a href="LICENSE"><img src="https://img.shields.io/badge/License-Apache_2.0-green.svg?style=flat-square" alt="License" /></a>
</p>

An autonomous mobile manipulation system built on **ROS 2 Jazzy** and **Gazebo Harmonic**. The robot — a **MiR 100** differential drive base carrying a **UR5e** 6-DOF arm with a parallel-jaw gripper — can locate a target object, navigate to an optimal pick pose, grasp the object, carry it to a drop location, and place it, all under the supervision of a **BehaviorTree.CPP** mission executive.

---

## Demo / Showcasing

Watch the full, autonomous 12-step pick-and-place mission in action:

<p align="center">
  <video width="640" height="360" src="https://github.com/user-attachments/assets/ae783f16-9604-42a9-8498-a8ca07e86a3c" controls alt="Full Working Pick and Place Example"></video>
</p>

---

## Table of Contents

- [Demo / Showcasing](#demo--showcasing)
- [System Overview](#system-overview)
- [Hardware & Simulation Stack](#hardware--simulation-stack)
- [Repository Layout](#repository-layout)
- [Package Reference](#package-reference)
- [Quick Start](#quick-start)
- [Launch Reference](#launch-reference)
- [Architecture](#architecture)
- [Development Workflow](#development-workflow)
- [Prerequisites](#prerequisites)

> 📄 **All launch commands with full argument tables → [LAUNCH.md](LAUNCH.md)**

---

## System Overview

```
┌──────────────────────────────────────────────────────────┐
│                    pick_place_orchestrator                 │
│   BehaviorTree.CPP  ──►  12-step pick-and-place mission   │
│    TargetAcquisition │ OptimizePose │ NavigateToPose       │
│    MoveArm │ GripperControl │ Attach/DetachPayload         │
└────────┬─────────┬──────────────┬────────────┬────────────┘
         │         │              │            │
         ▼         ▼              ▼            ▼
  [Nav2 stack] [MoveIt 2]  [Base Placement] [Gazebo Bridge]
  autonomous   motion       Optimizer       ros_gz_bridge
  navigation   planning     (IK + costmap   topic bridging
  (Nav2 + SLAM  (MoveIt 2   scoring)
   Toolbox)     + ros2_control)
```

The 12-step mission (6 pick + 6 place) is encoded in a single BehaviorTree XML file. A fallback recovery branch detaches any held payload and stows the arm if any step fails.

---

## Hardware & Simulation Stack

| Layer | Technology |
|---|---|
| Mobile Base | MiR 100 (differential drive) |
| Manipulator | Universal Robots UR5e (6-DOF) |
| Gripper | Parallel-jaw (prismatic fingers) |
| Simulator | Gazebo Harmonic (`ros_gz_sim`) |
| Motion Planning | MoveIt 2 |
| Navigation | Nav2 |
| SLAM | slam_toolbox (online async) |
| Mission Logic | BehaviorTree.CPP 4 |
| Build System | ROS 2 Jazzy · `ament_cmake` |

---

## Repository Layout

```
mobile_manipulator/
├── src/
│   ├── mir_description/               # MiR 100 URDF / meshes
│   ├── ur_description/                # UR5e URDF macros (upstream)
│   ├── mobile_manipulator_description/# Combined robot xacro + gripper
│   ├── mobile_manipulator_gazebo/     # Worlds, simulation launch
│   ├── mobile_manipulator_slam/       # slam_toolbox bringup + maps
│   ├── mobile_manipulator_nav/        # Nav2 bringup + arm-stow condition
│   ├── mobile_manipulator_moveit_config/ # MoveIt 2 SRDF + config
│   ├── base_placement_optimizer/      # IK-aware base pose action server
│   └── pick_place_orchestrator/       # BT engine + mission action server
├── specs/                             # Feature specs & implementation plans
│   ├── 001-mir-ur5e-description/
│   ├── …
│   └── 012-unified-launch/
└── .agents/                           # Spec Kit agent skills
```

---

## Package Reference

### `mobile_manipulator_description`
Provides the unified robot URDF/xacro.

- **`mobile_manipulator.urdf.xacro`** — Assembles the MiR 100 base, UR5e arm (mounted on the `surface` link), safety collision volume, parallel-jaw gripper, and an **Intel RealSense D435 camera** mounted on the end-effector (`ur5e_tool0`) for visual servoing alignment.
- **`mobile_manipulator.gazebo.xacro`** — Adds Gazebo plugins (ros2_control, differential drive, wrist depth camera, IMU, etc.).

<br/>
<img width="480" height="270" alt="Robot Description Visualisation" src="https://github.com/user-attachments/assets/cc760f68-91d3-4b93-b585-bd319e97ae51" />

### `mobile_manipulator_gazebo`
Simulation world and launch infrastructure.

- **World**: `worlds/nav_workspace.sdf` — Table, shelving, and target workpiece in a realistic warehouse scene. Includes 2 static monitoring cameras and a top-down overview camera to visualize navigation and manipulation.
- **Gazebo GUI**: Configured via `config/gazebo_gui.config` to dock the 3D View and display live monitoring feeds of the workspace and robot.
- **`simulation.launch.py`** — Brings up Gazebo Harmonic, spawns the robot, starts all `ros_gz_bridge` topics (cmd_vel, odom, tf, joint_states, camera/points), and sequences controller spawners via event handlers.

| Launch arg | Default | Description |
|---|---|---|
| `world` | `nav_workspace.sdf` | SDF world file name |
| `headless` | `false` | Run Gazebo without GUI |
| `x_pose` | `-4.0` | Robot spawn X |
| `y_pose` | `0.0` | Robot spawn Y |
| `z_pose` | `0.15` | Robot spawn Z |

### `mobile_manipulator_slam`
Online SLAM using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).

- Configured for async online mapping (`mapper_params_online_async.yaml`).
- RViz preset included for live map visualization.

<br/>
<img width="480" height="270" alt="SLAM Demo" src="https://github.com/user-attachments/assets/59093d08-08a7-45df-b1e2-0987b3b21738" />

### `mobile_manipulator_nav`
Nav2 bringup and a custom lifecycle condition node.

- **`is_arm_stowed_condition`** — Monitors the arm joint state and prevents navigation when the arm is not in its safe travel pose.
- Launch variants: full Nav2, nav2 without collision monitor, and a minimal nav_bringup.

<br/>
<img width="480" height="270" alt="Navigation Demo" src="https://github.com/user-attachments/assets/7bd079b4-632d-4106-a076-16ac18fc476c" />

### `mobile_manipulator_moveit_config`
Auto-generated (and hand-tuned) MoveIt 2 configuration.

- Planning group: `robot_arm` (UR5e joints).
- SRDF defines `stowed` named state for safe travel.
- **Deterministic Motion Planning**: Configured with the **Pilz Industrial Motion Planner** pipeline (`CommandPlanner`) for deterministic named (`PTP`) and straight-line Cartesian (`LIN`) trajectories. Enforces strict end-effector roll/pitch locking ($\pm 0.02$ rad) during Cartesian approaches/retreats, with automatic fallback to **OMPL** only for free-space/named transitions. Aborts immediately on linear approach planning failure to ensure safety.

<br/>
<img width="480" height="270" alt="MoveIt 2 Joint Trajectory Control" src="https://github.com/user-attachments/assets/d9dbdd4d-0688-4eb1-a44a-e8ef7e992d2f" />
<img width="480" height="270" alt="OctoMap 3D Occupancy Grid Representation" src="https://github.com/user-attachments/assets/978cc2d2-1db7-4118-a7a6-9f1264b3bc37" />


### `base_placement_optimizer`
ROS 2 Action Server that computes a reachable, collision-safe base pose around a target object.

**Action**: `optimize_placement` (`OptimizePlacement.action`)

**Algorithm**:
1. Sample `angular_samples` candidate poses at `reach_radius` around the target.
2. Filter by Nav2 OccupancyGrid (obstacle check).
3. Score each pose via IK feasibility + manipulability.
4. Score = `alpha × manipulability + (1 − alpha) × (1 / path_distance)`.

See [`src/base_placement_optimizer/README.md`](src/base_placement_optimizer/README.md) for full parameter reference, and **[`src/base_placement_optimizer/ARCHITECTURE.md`](src/base_placement_optimizer/ARCHITECTURE.md)** for a deep dive into the system architecture, algorithm flowcharts, and how to run the end-to-end task example.

<br/>
<img width="480" height="270" alt="Base Placement Optimizer Demo" src="https://github.com/user-attachments/assets/eba0b9b1-af66-48ee-abc9-22e476d65e41" />

### `pick_place_orchestrator`
The mission executive. Exposes a `PickPlaceMission` action server and drives execution through a BehaviorTree.CPP tree.

**Action**: `pick_place_mission` (`PickPlaceMission.action`)

**BT Nodes**:

| Node | Type | Description |
|---|---|---|
| `TargetAcquisition` | Action | Resolve pick/place pose from parameter server |
| `OptimizePose` | Action | Call `optimize_placement` action |
| `NavigateToPose` | Action | Send Nav2 NavigateToPose goal |
| `MoveArm` | Action | Execute MoveIt 2 Cartesian or named-pose goal |
| `VisualServo` | Action | Refine end-effector alignment using wrist depth camera feedback |
| `GripperControl` | Action | Open / close gripper via `gripper_controller` |
| `AttachPayload` | Action | Attach collision object to `ur5e_tool0` in MoveIt |
| `DetachPayload` | Action | Detach collision object from `ur5e_tool0` |
| `CheckPoseCondition` | Condition | Guard: skip place phase if no place pose set |

**Mission flow** (`behavior_trees/pick_place_mission.xml`):
```
Steps 1-6  (PICK_PHASE):
  1. TargetAcquisition(pick)
  2. OptimizePose(pick)  →  optimized_pick_base_pose
  3. NavigateToPose(optimized_pick_base_pose)
  4. Execute Pick:
     - GripperOpen
     - MoveArm(grasp_ready) [PTP]
     - MoveArm(pick_pose + 20cm standoff) [PTP]
     - VisualServo (align wrist camera to cylinder side contours)
     - MoveArm(pick_pose + 3cm advance) [PTP]
     - GripperClose
  5. AttachPayload
     - MoveArm(pick_pose + 15cm lift) [PTP]
  6. MoveArm(stowed) [PTP]

Steps 7-12 (PLACE_PHASE, optional):
  7. TargetAcquisition(place)
  8. OptimizePose(place)  →  optimized_place_base_pose
  9. NavigateToPose(optimized_place_base_pose)
 10. Execute Place:
     - MoveArm(place_pose + 15cm approach) [PTP]
     - MoveArm(place_pose) [LIN]
     - GripperOpen
 11. DetachPayload
     - MoveArm(place_pose + 15cm retract) [PTP]
 12. MoveArm(stowed) [PTP]

Recovery (on any failure):
  DetachPayload → MoveArm(stowed) → AlwaysFailure
```

> 📺 **See the [Demo / Showcasing](#demo--showcasing) section at the top of the page for a video of the full mission in action.**

---

## Quick Start

### 1. Prerequisites

```bash
# ROS 2 Jazzy + Gazebo Harmonic
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz*

# MoveIt 2
sudo apt install ros-jazzy-moveit

# Nav2
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# BehaviorTree.CPP 4
sudo apt install ros-jazzy-behaviortree-cpp

# slam_toolbox
sudo apt install ros-jazzy-slam-toolbox

# ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
```

### 2. Clone & Build

```bash
git clone <repo-url> mobile_manipulator
cd mobile_manipulator
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 3. Run the Full Pick-and-Place Mission (Auto-Triggered)

You can launch the entire stack (Gazebo simulation, Nav2, MoveIt 2, Base Placement Optimizer, BT Orchestrator) and automatically trigger the 12-step pick-and-place mission:

- **Via the Quickstart Script**:
  ```bash
  ./run_pick_place_mission.sh
  ```

- **Via ROS 2 Launch**:
  ```bash
  ros2 launch pick_place_orchestrator pick_place_mission.launch.py
  ```

**Options**:
```bash
# Run Gazebo in headless mode (no GUI)
ros2 launch pick_place_orchestrator pick_place_mission.launch.py headless:=true

# Launch without auto-starting the mission (manual trigger)
ros2 launch pick_place_orchestrator pick_place_mission.launch.py auto_start:=false

# Pass custom pick and place locations
ros2 launch pick_place_orchestrator pick_place_mission.launch.py \
  pick_x:=4.5 pick_y:=4.0 pick_z:=0.80 \
  place_x:=4.5 place_y:=-4.0 place_z:=0.75
```

The mission trigger node will send the action goal after an 80-second delay, allowing all system layers (localization, costmaps, planning scenes) to fully initialize.

### 4. Run the Validation Stack (Manual/Custom Goals)

Alternatively, bring up the entire stack *without* auto-starting the mission immediately:

```bash
ros2 launch pick_place_orchestrator validation.launch.py
```

Then, send a mission goal manually in another terminal:

```bash
ros2 action send_goal /pick_place_mission \
  pick_place_orchestrator/action/PickPlaceMission \
  "{pick_pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.5, y: 4.0, z: 0.80}, orientation: {w: 1.0}}}, \
    place_pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.5, y: -4.0, z: 0.75}, orientation: {w: 1.0}}}}"
```

---

## Launch Reference

All launch files across all packages are documented in **[LAUNCH.md](LAUNCH.md)**, including every argument, its default value, and concrete `ros2 launch` examples.

Quick summary of the most-used files:

| Launch File | Package | What it starts |
|---|---|---|
| [`pick_place_mission.launch.py`](src/pick_place_orchestrator/launch/pick_place_mission.launch.py) | `pick_place_orchestrator` | 🚀 **Complete Mission** — brings up the full stack and auto-starts the pick-and-place sequence |
| [`validation.launch.py`](src/pick_place_orchestrator/launch/validation.launch.py) | `pick_place_orchestrator` | ⭐ **Full stack** — sim + Nav2 + MoveIt + optimizer + orchestrator (no auto-start) |
| [`orchestrator.launch.py`](src/pick_place_orchestrator/launch/orchestrator.launch.py) | `pick_place_orchestrator` | Orchestrator node only (assumes stack is up) |
| [`optimizer_demo.launch.py`](src/base_placement_optimizer/launch/optimizer_demo.launch.py) | `base_placement_optimizer` | Sim + Nav2 + MoveIt + optimizer (no orchestrator) |
| [`simulation.launch.py`](src/mobile_manipulator_gazebo/launch/simulation.launch.py) | `mobile_manipulator_gazebo` | Gazebo + robot spawn + all bridges + controllers |
| [`nav_bringup.launch.py`](src/mobile_manipulator_nav/launch/nav_bringup.launch.py) | `mobile_manipulator_nav` | Sim + full Nav2 stack (map server, AMCL, planners) |
| [`navigation.launch.py`](src/mobile_manipulator_nav/launch/navigation.launch.py) | `mobile_manipulator_nav` | Nav2 stack only (no Gazebo) |
| [`demo.launch.py`](src/mobile_manipulator_moveit_config/launch/demo.launch.py) | `mobile_manipulator_moveit_config` | Sim + MoveIt move_group + RViz MotionPlanning panel |
| [`move_group.launch.py`](src/mobile_manipulator_moveit_config/launch/move_group.launch.py) | `mobile_manipulator_moveit_config` | MoveIt move_group only |
| [`slam_mapping.launch.py`](src/mobile_manipulator_slam/launch/slam_mapping.launch.py) | `mobile_manipulator_slam` | Sim + slam_toolbox (map building session) |
| [`display.launch.py`](src/mobile_manipulator_description/launch/display.launch.py) | `mobile_manipulator_description` | RViz URDF viewer + joint sliders |

See **[LAUNCH.md](LAUNCH.md)** for the remaining files (`gazebo.launch.py`, `optimizer.launch.py`, `rviz.launch.py`, `spawn_controllers.launch.py`, `nav2_no_collision_monitor.launch.py`, `mir_description/display.launch.py`, and the UR5e standalone demos).

---

## Architecture

### TF Tree

```
map
 └── odom
      └── base_link  (MiR 100)
           └── surface
                ├── ur5e_mount_link
                │    └── ur5e_base_link → … → ur5e_tool0
                │         ├── finger_left
                │         └── finger_right
                └── safety_collision_volume
```

### Topic Graph (key interfaces)

| Topic | Type | Direction |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo → ROS |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo → ROS |
| `/camera/points` | `sensor_msgs/PointCloud2` | Gazebo → ROS |
| `/optimize_placement` | `OptimizePlacement` action | ROS action |
| `/pick_place_mission` | `PickPlaceMission` action | ROS action |

### Controller Layout

| Controller | Joints |
|---|---|
| `joint_state_broadcaster` | All joints (read-only) |
| `joint_trajectory_controller` | UR5e arm joints |
| `gripper_controller` | `finger_left_joint`, `finger_right_joint` |

---

## Prerequisites

| Requirement | Version |
|---|---|
| ROS 2 | Jazzy |
| Gazebo | Harmonic |
| MoveIt 2 | Jazzy release |
| Nav2 | Jazzy release |
| BehaviorTree.CPP | 4.x |
| slam_toolbox | Jazzy release |
| ros2_control | Jazzy release |
| Python | 3.10+ |
| CMake | 3.22+ |

---

## License

Apache-2.0 — see individual `package.xml` files for per-package declarations.
