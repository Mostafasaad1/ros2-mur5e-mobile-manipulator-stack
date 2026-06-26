import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    nav_dir = get_package_share_directory('mobile_manipulator_nav')

    try:
        slam_dir = get_package_share_directory('mobile_manipulator_slam')
        default_map = os.path.join(slam_dir, 'maps', 'nav_workspace.yaml')
    except Exception:
        default_map = ''

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz     = LaunchConfiguration('use_rviz',     default='true')
    map_yaml     = LaunchConfiguration('map',          default=default_map)
    params_file  = LaunchConfiguration(
        'params_file',
        default=os.path.join(nav_dir, 'config', 'nav2_params.yaml'))

    # ── 1. Our custom navigation launch (no collision_monitor) ──────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'nav2_no_collision_monitor.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  params_file,
            'autostart':    'true',
        }.items(),
    )

    # ── 2. Map server (separate from nav2 bringup) ───────────────────────────
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
        ],
    )

    map_server_lc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    # ── 3. AMCL ──────────────────────────────────────────────────────────────
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {'use_sim_time': use_sim_time},
        ],
    )

    amcl_lc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']},
        ],
    )

    # ── 4. Sensor bridge (Gazebo → ROS 2) ────────────────────────────────────
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 5. Arm state monitor ──────────────────────────────────────────────────
    arm_state_monitor = Node(
        package='mobile_manipulator_nav',
        executable='arm_state_monitor.py',
        name='arm_state_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 6. RViz2 ─────────────────────────────────────────────────────────────
    rviz_cfg = os.path.join(nav_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        DeclareLaunchArgument('use_rviz',    default_value='true',
                              description='Launch RViz2'),
        DeclareLaunchArgument('map',         default_value=default_map,
                              description='Path to map yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
                              nav_dir, 'config', 'nav2_params.yaml'),
                              description='Nav2 params file'),

        sensor_bridge,
        map_server,
        map_server_lc,
        amcl,
        amcl_lc,
        nav2_launch,
        arm_state_monitor,
        rviz_node,
    ])
