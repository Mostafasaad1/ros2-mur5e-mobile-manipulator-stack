import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    try:
        mobile_manipulator_nav_dir = get_package_share_directory('mobile_manipulator_nav')
    except Exception:
        mobile_manipulator_nav_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    try:
        mobile_manipulator_slam_dir = get_package_share_directory('mobile_manipulator_slam')
        default_map_path = os.path.join(
            mobile_manipulator_slam_dir,
            'maps',
            'nav_workspace.yaml'
        )
    except Exception:
        default_map_path = ''

    try:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    except Exception:
        nav2_bringup_dir = ''

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    map_yaml_file = LaunchConfiguration('map', default=default_map_path)

    # Path to our customized nav2 params
    params_file_path = os.path.join(
        mobile_manipulator_nav_dir,
        'config',
        'nav2_params.yaml'
    )
    params_file = LaunchConfiguration('params_file', default=params_file_path)

    # 1. Include standard Nav2 bringup
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
        condition=IfCondition(PythonExpression(["'", nav2_bringup_dir, "' != ''"]))
    )

    # 2. Arm State Monitor Node
    arm_state_monitor = Node(
        package='mobile_manipulator_nav',
        executable='arm_state_monitor.py',
        name='arm_state_monitor',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Sensor Bridge Node (bridges laser scan and IMU from Gazebo to ROS 2)
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='sensor_bridge',
        output='screen',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. RViz2 display node
    rviz_config_file = os.path.join(
        mobile_manipulator_nav_dir,
        'rviz',
        'nav2_default_view.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(PythonExpression(["'", headless, "' != 'true'"]))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo in headless mode'
        ),
        DeclareLaunchArgument(
            'map', default_value=default_map_path,
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file', default_value=params_file_path,
            description='Full path to the ROS 2 parameters file to use for all launched nodes'
        ),

        nav2_bringup,
        sensor_bridge,
        arm_state_monitor,
        rviz_node,
    ])
