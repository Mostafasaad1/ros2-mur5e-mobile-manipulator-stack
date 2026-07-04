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
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    ros2_control_hardware_type_arg = DeclareLaunchArgument(
        'ros2_control_hardware_type',
        default_value='mock',
        description='Type of ros2_control hardware: mock, gz, etc.',
    )

    # Build MoveIt configuration using MoveItConfigsBuilder
    moveit_configs = (
        MoveItConfigsBuilder('mobile_manipulator', package_name='mobile_manipulator_moveit_config')
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('mobile_manipulator_moveit_config'),
                'config',
                'mobile_manipulator.urdf.xacro'
            ),
            mappings={
                'ros2_control_hardware_type': LaunchConfiguration('ros2_control_hardware_type')
            }
        )
        .planning_pipelines(
            pipelines=['ompl', 'pilz_industrial_motion_planner']
        )
        .sensors_3d('config/sensors_3d.yaml')
        .to_moveit_configs()
    )

    # Move Group Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'trajectory_execution.allowed_execution_duration_scaling': 10.0,
                'trajectory_execution.allowed_goal_duration_margin': 10.0,
                'trajectory_execution.execution_duration_monitoring': False,
                'start_state_max_bounds_error': 0.2,
                'check_start_state_bounds.start_state_max_bounds_error': 0.2,
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        ros2_control_hardware_type_arg,
        move_group_node,
    ])
