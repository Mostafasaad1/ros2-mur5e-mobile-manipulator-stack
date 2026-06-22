import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Get package directories
    moveit_config_dir = get_package_share_directory('mobile_manipulator_moveit_config')
    orchestrator_dir = get_package_share_directory('pick_place_orchestrator')

    # Build MoveIt configuration for the node
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

    # Path to Behavior Tree XML definition
    bt_xml_path = os.path.join(orchestrator_dir, 'behavior_trees', 'pick_place_mission.xml')

    # Orchestrator Node
    orchestrator_node = Node(
        package='pick_place_orchestrator',
        executable='orchestrator_node',
        name='pick_place_orchestrator_node',
        output='screen',
        parameters=[
            moveit_configs.to_dict(),
            {
                'bt_xml_path': bt_xml_path,
                'use_sim_time': True
            }
        ]
    )

    return LaunchDescription([
        orchestrator_node
    ])
