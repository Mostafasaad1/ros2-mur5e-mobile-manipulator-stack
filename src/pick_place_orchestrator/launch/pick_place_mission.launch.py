#!/usr/bin/env python3
"""Complete Pick and Place Mission Launch File."""

# System Components:
# - Gazebo Harmonic simulation with nav_workspace world
# - Nav2 navigation stack with pre-built map
# - MoveIt 2 motion planning
# - Base Placement Optimizer
# - Pick & Place Orchestrator with Behavior Tree
# - Mission Trigger Node (auto-starts the mission)
#
# Tables:
# - Pick table:  (5.0, 4.0, 0.8m height)
# - Place table: (5.0, -4.0, 0.8m height)
# - Workpiece spawns at (5.0, 4.0, 0.88m) on pick table
#
# Usage:
#   ros2 launch pick_place_orchestrator pick_place_mission.launch.py
#
# Optional Arguments:
#   headless:=true|false    - Run Gazebo without GUI (default: false for visualization)
#   use_rviz:=true|false    - Launch RViz2 (default: true)
#   auto_start:=true|false  - Automatically trigger mission (default: true)
#   pick_x:=5.0             - Pick pose X coordinate (default: 5.0)
#   pick_y:=4.0             - Pick pose Y coordinate (default: 4.0)
#   pick_z:=0.88            - Pick pose Z coordinate (default: 0.88)
#   place_x:=4.5            - Place pose X coordinate (default: 4.5)
#   place_y:=-4.0           - Place pose Y coordinate (default: -4.0)
#   place_z:=0.75           - Place pose Z coordinate (default: 0.75)

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch configurations
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_start = LaunchConfiguration('auto_start')
    pick_x = LaunchConfiguration('pick_x')
    pick_y = LaunchConfiguration('pick_y')
    pick_z = LaunchConfiguration('pick_z')
    place_x = LaunchConfiguration('place_x')
    place_y = LaunchConfiguration('place_y')
    place_z = LaunchConfiguration('place_z')

    # Get package directory
    orchestrator_dir = get_package_share_directory('pick_place_orchestrator')

    # 1. Include the full system stack (Gazebo, Nav2, MoveIt, Optimizer, Orchestrator)
    validation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orchestrator_dir, 'launch', 'validation.launch.py')
        ),
        launch_arguments={
            'headless': headless,
            'use_rviz': use_rviz,
        }.items()
    )

    # 2. Mission Trigger Node (with delay to allow system initialization)
    # This node will send the pick & place action goal
    mission_trigger_node = Node(
        package='pick_place_orchestrator',
        executable='mission_trigger_node',
        name='mission_trigger_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pick_x': pick_x,
            'pick_y': pick_y,
            'pick_z': pick_z,
            'place_x': place_x,
            'place_y': place_y,
            'place_z': place_z,
        }],
        condition=IfCondition(auto_start)
    )

    # Delay mission trigger to allow full system startup
    # Wait 70 seconds after orchestrator starts (which itself waits 10s after optimizer_demo)
    delayed_mission_trigger = TimerAction(
        period=80.0,
        actions=[mission_trigger_node]
    )

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo in headless mode (no GUI)'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),
        DeclareLaunchArgument(
            'auto_start',
            default_value='true',
            description='Automatically trigger the pick and place mission'
        ),
        DeclareLaunchArgument(
            'pick_x',
            default_value='4.5',
            description='Pick pose X coordinate (world frame)'
        ),
        DeclareLaunchArgument(
            'pick_y',
            default_value='4.0',
            description='Pick pose Y coordinate (world frame)'
        ),
        DeclareLaunchArgument(
            'pick_z',
            default_value='0.68',
            description='Pick pose Z coordinate (world frame, table height + object height/2)'
        ),
        DeclareLaunchArgument(
            'place_x',
            default_value='4.5',
            description='Place pose X coordinate (world frame)'
        ),
        DeclareLaunchArgument(
            'place_y',
            default_value='-4.0',
            description='Place pose Y coordinate (world frame)'
        ),
        DeclareLaunchArgument(
            'place_z',
            default_value='0.75',
            description='Place pose Z coordinate (above container walls)'
        ),

        # Launch the full system
        validation_launch,

        # Trigger the mission after delay
        delayed_mission_trigger,
    ])
