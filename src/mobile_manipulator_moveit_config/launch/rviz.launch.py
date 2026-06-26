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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt configuration
    moveit_configs = (
        MoveItConfigsBuilder('mobile_manipulator', package_name='mobile_manipulator_moveit_config')
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'config',
                'mobile_manipulator.urdf.xacro'
            ),
            mappings={
                'ros2_control_hardware_type': LaunchConfiguration(
                    'ros2_control_hardware_type', default='mock'
                )
            }
        )
        .to_moveit_configs()
    )

    # Path to rviz config
    rviz_config_file = os.path.join(
        get_package_share_directory('mobile_manipulator_moveit_config'),
        'config',
        'moveit.rviz'
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_configs.robot_description,
            moveit_configs.robot_description_semantic,
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'ros2_control_hardware_type',
            default_value='mock',
            description='Type of ros2_control hardware: mock, gz, etc.',
        ),
        rviz_node,
    ])
