from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='base_placement_optimizer',
            executable='optimizer_node_exe',
            name='base_placement_optimizer',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    ])
