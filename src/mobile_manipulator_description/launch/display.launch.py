import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ur_type = LaunchConfiguration('ur_type', default='ur5e')
    tf_prefix = LaunchConfiguration('tf_prefix', default='ur5e_')

    urdf = os.path.join(
        get_package_share_directory('mobile_manipulator_description'),
        'urdf',
        'mobile_manipulator.urdf.xacro'
    )

    robot_desc = ParameterValue(
        Command([
            'xacro ', urdf,
            ' ur_type:=', ur_type,
            ' tf_prefix:=', tf_prefix,
        ]),
        value_type=str
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('mobile_manipulator_description'),
        'rviz',
        'display.rviz'
    )

    # Static transform from odom to base_footprint if needed for basic rviz visualization
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'ur_type',
            default_value='ur5e',
            description='Type of Universal Robot (e.g., ur5, ur5e)'
        ),
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='ur5e_',
            description='TF prefix to prepend to arm links'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        static_tf_node,

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir]
        ),
    ])
