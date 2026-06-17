import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package directories
    mobile_manipulator_desc_dir = get_package_share_directory('mobile_manipulator_description')
    mir_description_dir = get_package_share_directory('mir_description')
    ur_description_dir = get_package_share_directory('ur_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')
    world_file = LaunchConfiguration('world', default='empty.sdf')

    # Set GZ_SIM_RESOURCE_PATH to find meshes in all three packages
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=':'.join([
            os.path.join(mobile_manipulator_desc_dir, '..'),
            os.path.join(mir_description_dir, '..'),
            os.path.join(ur_description_dir, '..'),
        ])
    )

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
            'gz_args': [
                PythonExpression(["'-r -s ' if '", headless, "' == 'true' else '-r '"]),
                world_file
            ],
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

    # Bridge topics between ROS 2 and Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/gazebo_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        remappings=[
            ('/gazebo_tf', '/tf')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo in headless mode if true'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'world', default_value='empty.sdf',
            description='Gazebo world file'
        ),

        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        TimerAction(period=2.0, actions=[spawn_entity]),
        bridge,
    ])
