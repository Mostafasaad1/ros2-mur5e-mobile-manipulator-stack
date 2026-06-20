import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch configurations
    headless = LaunchConfiguration('headless', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Get package directories
    nav_dir = get_package_share_directory('mobile_manipulator_nav')
    moveit_config_dir = get_package_share_directory('mobile_manipulator_moveit_config')

    # 1. Include Nav Bringup (Simulation + Nav2 + RViz)
    nav_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'nav_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'headless': headless,
            'use_rviz': use_rviz,
        }.items(),
    )

    # 2. Include Move Group (MoveIt 2 planning)
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'ros2_control_hardware_type': 'gz',
        }.items(),
    )

    # 3. Build MoveIt configuration for the optimizer node
    moveit_configs = (
        MoveItConfigsBuilder('mobile_manipulator', package_name='mobile_manipulator_moveit_config')
        .robot_description(
            file_path=os.path.join(
                moveit_config_dir,
                'config',
                'mobile_manipulator.urdf.xacro'
            ),
            mappings={
                'ros2_control_hardware_type': 'gz'
            }
        )
        .to_moveit_configs()
    )

    # 4. Optimizer Node
    optimizer_node = Node(
        package='base_placement_optimizer',
        executable='optimizer_node_exe',
        name='base_placement_optimizer',
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo in headless mode'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),
        nav_bringup,
        move_group,
        optimizer_node,
    ])
