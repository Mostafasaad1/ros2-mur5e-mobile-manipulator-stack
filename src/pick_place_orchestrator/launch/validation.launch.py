import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')

    # 1. Full simulation/nav/moveit/optimizer stack
    optimizer_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('base_placement_optimizer'),
                'launch',
                'optimizer_demo.launch.py'
            )
        ),
        launch_arguments={
            'headless': headless,
            'use_rviz': use_rviz,
        }.items()
    )

    # 2. Orchestrator node (started with a small delay to ensure MoveIt and Nav2 are up)
    orchestrator = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('pick_place_orchestrator'),
                        'launch',
                        'orchestrator.launch.py'
                    )
                )
            )
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
        optimizer_demo,
        orchestrator
    ])
