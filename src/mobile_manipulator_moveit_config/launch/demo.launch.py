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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Build MoveIt configuration using MoveItConfigsBuilder pointing to the wrapper URDF
    moveit_configs = (
        MoveItConfigsBuilder('mobile_manipulator', package_name='mobile_manipulator_moveit_config')
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'config',
                'mobile_manipulator.urdf.xacro'
            ),
            mappings={
                'ros2_control_hardware_type': 'mock'
            }
        )
        .to_moveit_configs()
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            moveit_configs.robot_description,
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')},
        ],
    )

    # ros2_control node running the mock hardware interface
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_configs.robot_description,
            os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'config',
                'ros2_controllers.yaml'
            ),
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}
        ],
        output='screen',
    )

    # Spawn standard controllers (broadcaster and trajectory controller)
    spawn_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'launch',
                'spawn_controllers.launch.py'
            )
        )
    )

    # Move Group Launch (configured for mock hardware execution)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'launch',
                'move_group.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'ros2_control_hardware_type': 'mock',
        }.items(),
    )

    # RViz Launch
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'launch',
                'rviz.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'ros2_control_hardware_type': 'mock',
        }.items(),
    )

    # Static transform publisher to supply base_link transforms if odom isn't present
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'odom', 'base_link'],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        spawn_controllers,
        move_group,
        rviz,
    ])
