import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package directories
    nav_dir = get_package_share_directory('mobile_manipulator_nav')
    gazebo_dir = get_package_share_directory('mobile_manipulator_gazebo')
    slam_dir = get_package_share_directory('mobile_manipulator_slam')

    # Default paths
    default_map = os.path.join(slam_dir, 'maps', 'nav_workspace.yaml')
    default_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')

    # Declared launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless     = LaunchConfiguration('headless')
    use_rviz     = LaunchConfiguration('use_rviz')
    world        = LaunchConfiguration('world')
    map_yaml     = LaunchConfiguration('map')
    params_file  = LaunchConfiguration('params_file')
    x_pose       = LaunchConfiguration('x_pose')
    y_pose       = LaunchConfiguration('y_pose')
    z_pose       = LaunchConfiguration('z_pose')

    # 1. Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world':        world,
            'headless':     headless,
            'x_pose':       x_pose,
            'y_pose':       y_pose,
            'z_pose':       z_pose,
        }.items(),
    )

    # 2. Navigation launch (Map server, AMCL, Nav2 servers, Sensor bridge, RViz)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz':     use_rviz,
            'map':          map_yaml,
            'params_file':  params_file,
        }.items(),
    )

    # Delay the navigation launch by 12 seconds to ensure Gazebo controllers are spawned
    # and publishing transforms/odometry before AMCL attempts to configure.
    delayed_navigation_launch = TimerAction(
        period=12.0,
        actions=[navigation_launch]
    )

    return LaunchDescription([
        # Declare arguments with descriptions and defaults
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo in headless mode'
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz2'
        ),
        DeclareLaunchArgument(
            'world', default_value='nav_workspace.sdf',
            description='Name of the Gazebo world file to load'
        ),
        DeclareLaunchArgument(
            'map', default_value=default_map,
            description='Path to map yaml'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='Nav2 parameters file'
        ),
        DeclareLaunchArgument(
            'x_pose', default_value='-4.0',
            description='Initial robot X spawn pose'
        ),
        DeclareLaunchArgument(
            'y_pose', default_value='0.0',
            description='Initial robot Y spawn pose'
        ),
        DeclareLaunchArgument(
            'z_pose', default_value='0.15',
            description='Initial robot Z spawn pose'
        ),

        # Launch Gazebo simulation
        gazebo_launch,

        # Launch Navigation and RViz with delay
        delayed_navigation_launch,
    ])
