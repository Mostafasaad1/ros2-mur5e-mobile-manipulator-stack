import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def load_file(file_path):
    try:
        with open(file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def degrees_constructor(loader, node):
    import math
    value = float(loader.construct_scalar(node))
    return value * math.pi / 180.0

yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

def load_yaml(file_path):
    try:
        with open(file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def launch_setup(context, *args, **kwargs):
    ur_type = context.perform_substitution(LaunchConfiguration("ur_type"))
    
    pkg_share = FindPackageShare("ur_description").find("ur_description")
    
    # Run xacro to get robot_description
    import subprocess
    xacro_path = os.path.join(pkg_share, "urdf", "ur_mocked.urdf.xacro")
    robot_description_content = subprocess.check_output([
        "xacro", xacro_path, f"ur_type:={ur_type}", "name:=ur"
    ]).decode("utf-8")
    
    robot_description = {"robot_description": robot_description_content}
    
    # Load SRDF — selected by ur_type (e.g. ur5.srdf or ur5e.srdf)
    srdf_path = os.path.join(pkg_share, "config", "moveit", f"{ur_type}.srdf")
    srdf_content = load_file(srdf_path)
    robot_description_semantic = {"robot_description_semantic": srdf_content}
    
    # Load Kinematics
    kinematics_yaml = load_yaml(os.path.join(pkg_share, "config", "moveit", "kinematics.yaml"))
    
    # Load Joint Limits
    joint_limits_yaml = load_yaml(os.path.join(pkg_share, "config", ur_type, "joint_limits.yaml"))
    
    # Load OMPL Planning
    ompl_yaml = load_yaml(os.path.join(pkg_share, "config", "moveit", "ompl_planning.yaml"))
    
    # Load MoveIt Controllers
    moveit_controllers = load_yaml(os.path.join(pkg_share, "config", "moveit", "moveit_controllers.yaml"))
    
    # Load ROS 2 Controllers configuration
    ros2_controllers_path = os.path.join(pkg_share, "config", "moveit", "ros2_controllers.yaml")

    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # Node: ros2_control_node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )
    
    # Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    ur_manipulator_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_manipulator_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Node: move_group
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            ompl_yaml,
            moveit_controllers,
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
        ],
    )
    
    launch_rviz = context.perform_substitution(LaunchConfiguration("launch_rviz")).lower() == "true"
    
    # Node: RViz
    rviz_node = None
    if launch_rviz:
        rviz_config_path = os.path.join(pkg_share, "rviz", "pick_place_demo.rviz")
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_path],
            parameters=[
                robot_description,
                robot_description_semantic,
                {"robot_description_kinematics": kinematics_yaml},
            ],
        )
    
    # Our Demo node (pick_place_demo)
    pick_place_demo_node = Node(
        package="ur_description",
        executable="pick_place_demo",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
        ],
    )

    nodes = [
        robot_state_publisher_node,
        ros2_control_node,
        # Sequence controller spawners to start only AFTER ros2_control_node is up
        # Eliminates the race condition where spawners call /controller_manager before it exists
        RegisterEventHandler(
            OnProcessStart(
                target_action=ros2_control_node,
                on_start=[
                    joint_state_broadcaster_spawner,
                    ur_manipulator_controller_spawner,
                ],
            )
        ),
        move_group_node,
        pick_place_demo_node,
    ]
    if rviz_node:
        nodes.append(rviz_node)
    return nodes

def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
            choices=["ur5", "ur5e"],
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 config interface.",
            choices=["true", "false"],
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
