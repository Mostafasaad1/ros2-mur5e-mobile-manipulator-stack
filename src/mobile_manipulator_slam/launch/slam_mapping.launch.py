import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnStateTransition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Package directories
    mobile_manipulator_gazebo_dir = get_package_share_directory('mobile_manipulator_gazebo')
    mobile_manipulator_slam_dir = get_package_share_directory('mobile_manipulator_slam')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # 1. Include base simulation launch from mobile_manipulator_gazebo
    # We pass the custom maze world file as an argument
    base_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mobile_manipulator_gazebo_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': 'simple_maze.sdf'
        }.items()
    )

    # 2. ROS-Gazebo Parameter Bridge for Sensor Data
    # Bridge `/scan` (LiDAR scan messages) and `/imu` (IMU messages)
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

    # 3. slam_toolbox configuration file path
    slam_params_file = os.path.join(
        mobile_manipulator_slam_dir,
        'config',
        'mapper_params_online_async.yaml'
    )

    # 4. Asynchronous slam_toolbox LifecycleNode
    slam_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        namespace=''
    )

    # 5. Automated Lifecycle Transitions for slam_toolbox
    # When node is instantiated (unconfigured), transition to configured
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_relationship=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # When transitioning to configured, transition to active
    activate_event = RegisterEventHandler(
        RegisterEventHandler(
            event_handler=OnStateTransition(
                target_lifecycle_node=slam_node,
                start_state='unconfigured',
                goal_state='inactive',
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_relationship=matches_action(slam_node),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            )
        )
    )

    # 6. RViz2 display node loaded with customized workspace view
    rviz_config_file = os.path.join(
        mobile_manipulator_slam_dir,
        'config',
        'slam.rviz'
    )
    rviz_node = Node(
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

        # Base simulation world & robot spawn
        base_simulation,

        # ROS-Gazebo bridge
        sensor_bridge,

        # SLAM toolbox & automated lifecycle actions
        slam_node,
        configure_event,
        activate_event,

        # Visualization
        rviz_node,
    ])
