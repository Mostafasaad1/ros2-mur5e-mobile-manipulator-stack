import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Package directories
    mobile_manipulator_gazebo_dir = get_package_share_directory('mobile_manipulator_gazebo')
    mobile_manipulator_desc_dir = get_package_share_directory('mobile_manipulator_description')
    mir_description_dir = get_package_share_directory('mir_description')
    ur_description_dir = get_package_share_directory('ur_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Set GZ_SIM_RESOURCE_PATH to find meshes and worlds
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(mobile_manipulator_gazebo_dir, '..'),
            os.path.join(mobile_manipulator_desc_dir, '..'),
            os.path.join(mir_description_dir, '..'),
            os.path.join(ur_description_dir, '..'),
        ])
    )

    # Path to the world file (custom simple maze)
    world_path = os.path.join(mobile_manipulator_gazebo_dir, 'worlds', 'simple_maze.sdf')

    # Process Xacro -> robot_description
    xacro_path = os.path.join(
        mobile_manipulator_desc_dir,
        'urdf',
        'mobile_manipulator.gazebo.xacro'
    )
    robot_desc = ParameterValue(
        Command(['xacro ', xacro_path]), value_type=str
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # Gazebo Harmonic Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true',
        }.items()
    )

    # Spawn Mobile Manipulator Entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mobile_manipulator',
        output='screen',
        arguments=[
            '-name', 'mobile_manipulator',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.15',
        ]
    )

    # ROS-Gazebo Parameter Bridge for Clock, LiDAR scan, and IMU data
    parameter_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_controller',
            '--controller-ros-args',
            '-r ~/cmd_vel:=/cmd_vel',
            '--controller-ros-args',
            '-r ~/odom:=/odom'
        ],
        output='screen',
    )

    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen',
    )

    # Sequence Spawners:
    # 1. Wait for spawn_entity to finish -> start joint_state_broadcaster
    # 2. Wait for joint_state_broadcaster to finish -> start diff_drive and joint_trajectory
    load_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    load_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    load_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    # slam_toolbox LifecycleNode
    slam_params_file = os.path.join(
        mobile_manipulator_gazebo_dir,
        'config',
        'mapper_params_online_async.yaml'
    )
    slam_toolbox = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace='',
        parameters=[
            slam_params_file,
            {
                'use_lifecycle_manager': False,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # Lifecycle transition events for slam_toolbox auto-start
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_toolbox),
            transition_id=Transition.TRANSITION_CONFIGURE
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_toolbox,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(slam_toolbox),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        )
    )

    # RViz2 Node
    rviz_config_file = os.path.join(
        mobile_manipulator_gazebo_dir,
        'config',
        'slam.rviz'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),

        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        parameter_bridge,
        load_joint_state_broadcaster,
        load_diff_drive_controller,
        load_joint_trajectory_controller,
        slam_toolbox,
        configure_event,
        activate_event,
        rviz2,
    ])
