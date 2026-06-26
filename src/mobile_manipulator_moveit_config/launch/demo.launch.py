# Copyright 2026 mox
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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configurations
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    # Gazebo simulation (robot spawn + bridges + controllers)
    gazebo_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobile_manipulator_gazebo'),
                'launch',
                'simulation.launch.py',
            )
        ),
        launch_arguments={
            'headless': headless,
            'use_sim_time': 'true',
        }.items(),
    )

    # Move Group
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'launch',
                'move_group.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'ros2_control_hardware_type': 'gz',
        }.items(),
    )

    # RViz (optional)
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'launch',
                'rviz.launch.py',
            )
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={
            'use_sim_time': 'true',
            'ros2_control_hardware_type': 'gz',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run Gazebo without its GUI (server-only mode)',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 with the MoveIt MotionPlanning panel',
        ),
        gazebo_simulation,
        move_group,
        rviz,
    ])
