#!/bin/bash
# Quick Start Script for Pick and Place Mission
# This script demonstrates how to launch the complete system

echo "=========================================="
echo "Mobile Manipulator Pick and Place Mission"
echo "=========================================="
echo ""
echo "This will launch:"
echo "  - Gazebo Harmonic (nav_workspace world)"
echo "  - Nav2 Navigation Stack"
echo "  - MoveIt 2 Motion Planning"
echo "  - Base Placement Optimizer"
echo "  - Pick & Place Orchestrator"
echo "  - Mission Trigger Node (auto-start)"
echo ""
echo "Tables:"
echo "  Pick:  (5.0, 4.0, 0.8m)  [GREEN]"
echo "  Place: (5.0, -4.0, 0.8m) [RED]"
echo ""
echo "The mission will auto-start in ~25 seconds"
echo "=========================================="
echo ""

# Source the workspace
source install/setup.bash

# Launch the mission
ros2 launch pick_place_orchestrator pick_place_mission.launch.py
