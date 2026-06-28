# Pick and Place Mission Launch

## Overview

This package provides a complete, autonomous pick-and-place mission launcher for the mobile manipulator. The system executes a table-to-table object transfer task using behavior trees, navigation, motion planning, and base placement optimization.

## System Architecture

The complete system includes:

1. **Gazebo Harmonic Simulation** - Physics simulation with `nav_workspace.sdf` world
2. **Nav2 Navigation Stack** - AMCL localization and path planning
3. **MoveIt 2** - Arm motion planning with collision avoidance
4. **Base Placement Optimizer** - Optimal mobile base positioning for manipulation
5. **Pick & Place Orchestrator** - Behavior tree-based mission execution
6. **Mission Trigger Node** - Automated action client to start the mission

## Quick Start

### Launch the Complete Mission

```bash
# Terminal 1: Launch the entire system and auto-start the mission
ros2 launch pick_place_orchestrator pick_place_mission.launch.py
```

This single command will:
- Start Gazebo with the warehouse world
- Launch Nav2 for navigation
- Start MoveIt for motion planning
- Bring up the optimizer and orchestrator
- Automatically trigger the pick-and-place mission after 25 seconds

### Monitor Progress

Watch the terminal output for mission feedback:
```
[mission_trigger_node]: Feedback - Phase: PICK_PHASE | Step: NAVIGATE_TO_PICK | Index: 3
[mission_trigger_node]: Feedback - Phase: PICK_PHASE | Step: EXECUTE_PICK | Index: 4
[mission_trigger_node]: Feedback - Phase: PLACE_PHASE | Step: NAVIGATE_TO_PLACE | Index: 9
...
[mission_trigger_node]: ✓ Mission SUCCEEDED! Object successfully moved from pick to place table.
```

## Launch Arguments

### Display Configuration

```bash
# Launch without Gazebo GUI (headless mode, faster)
ros2 launch pick_place_orchestrator pick_place_mission.launch.py headless:=true

# Launch without RViz
ros2 launch pick_place_orchestrator pick_place_mission.launch.py use_rviz:=false
```

### Mission Control

```bash
# Launch system but don't auto-start mission (manual trigger)
ros2 launch pick_place_orchestrator pick_place_mission.launch.py auto_start:=false

# Then manually trigger in another terminal:
ros2 action send_goal /pick_place_mission pick_place_orchestrator/action/PickPlaceMission \
  "{pick_pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 4.0, z: 0.88}, orientation: {w: 1.0}}}, \
    place_pose: {header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: -4.0, z: 0.88}, orientation: {w: 1.0}}}}"
```

### Custom Pick/Place Locations

```bash
# Custom pick and place coordinates
ros2 launch pick_place_orchestrator pick_place_mission.launch.py \
  pick_x:=5.0 pick_y:=4.0 pick_z:=0.88 \
  place_x:=5.0 place_y:=-4.0 place_z:=0.88
```

## World Configuration

The `nav_workspace.sdf` world contains:

- **Pick Table**: Position `(5.0, 4.0)`, Height `0.8m`, Color: Green
- **Place Table**: Position `(5.0, -4.0)`, Height `0.8m`, Color: Red
- **Workpiece**: Yellow cylinder, spawns at `(5.0, 4.0, 0.88)` on pick table
- **Robot Spawn**: Position `(-4.0, 0.0, 0.15)`

The world has a central dividing wall with two passages:
- **Wide door**: Y ∈ [2.5, 5.5] - 3.0m wide (preferred route to pick table)
- **Narrow passage**: Y ∈ [-2.5, -1.0] - 1.5m wide (high-cost route to place table)

## Mission Phases

The orchestrator executes a 12-step behavior tree:

### Pick Phase (Steps 1-6)
1. **Target Acquisition Pick** - Validate pick pose
2. **Optimize Pick Pose** - Calculate optimal base position
3. **Navigate to Pick** - Drive to optimized base pose
4. **Execute Pick** - Open gripper → Move arm → Close gripper
5. **Attach Payload** - Attach object to planning scene
6. **Stow Arm Pick** - Move arm to safe travel position

### Place Phase (Steps 7-12)
7. **Target Acquisition Place** - Validate place pose
8. **Optimize Place Pose** - Calculate optimal base position
9. **Navigate to Place** - Drive to optimized base pose
10. **Execute Place** - Move arm → Open gripper
11. **Detach Payload** - Remove object from planning scene
12. **Stow Arm Place** - Return arm to stowed position

## Troubleshooting

### Mission doesn't start automatically
- Increase startup delay if needed
- Check all nodes are running: `ros2 node list`
- Verify action server: `ros2 action list`

### Navigation fails
- Check AMCL localization in RViz
- Verify map server: `ros2 node info /map_server`
- Try manual Nav2 Goal in RViz first

### Motion planning fails
- Check MoveIt planning scene in RViz
- Verify target height is correct
- Ensure optimizer is running: `ros2 node info /base_placement_optimizer`

## Files

- **launch/pick_place_mission.launch.py** - Main mission launch file
- **src/mission_trigger_node.cpp** - Action client node
- **action/PickPlaceMission.action** - Mission action definition
- **behavior_trees/pick_place_mission.xml** - 12-step behavior tree
