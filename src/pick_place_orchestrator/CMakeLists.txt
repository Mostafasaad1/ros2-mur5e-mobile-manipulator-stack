cmake_minimum_required(VERSION 3.8)
project(pick_place_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(base_placement_optimizer REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate Action Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PickPlaceMission.action"
  DEPENDENCIES geometry_msgs action_msgs
)

# Build Library
add_library(pick_place_orchestrator_lib SHARED
  src/behavior_tree_engine.cpp
  src/behavior_tree_nodes/target_acquisition.cpp
  src/behavior_tree_nodes/optimize_pose_action.cpp
  src/behavior_tree_nodes/navigate_to_pose_action.cpp
  src/behavior_tree_nodes/move_arm_action.cpp
  src/behavior_tree_nodes/payload_actions.cpp
  src/behavior_tree_nodes/gripper_control_action.cpp
  src/behavior_tree_nodes/check_pose_condition.cpp
  src/behavior_tree_nodes/visual_servo_action.cpp
  src/orchestrator_node.cpp
)

target_include_directories(pick_place_orchestrator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(pick_place_orchestrator_lib PUBLIC cxx_std_17)

# We define BT_PLUGIN_EXPORT so the factory registration works
target_compile_definitions(pick_place_orchestrator_lib PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(pick_place_orchestrator_lib
  rclcpp
  rclcpp_action
  rclcpp_components
  behaviortree_cpp
  geometry_msgs
  action_msgs
  moveit_ros_planning_interface
  base_placement_optimizer
  nav2_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

# Link against our generated interfaces
rosidl_target_interfaces(pick_place_orchestrator_lib ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Register OrchestratorNode as a component and build the executable
rclcpp_components_register_node(pick_place_orchestrator_lib
  PLUGIN "pick_place_orchestrator::OrchestratorNode"
  EXECUTABLE orchestrator_node
)

# Build Mission Trigger Node executable
add_executable(mission_trigger_node src/mission_trigger_node.cpp)
target_compile_features(mission_trigger_node PUBLIC cxx_std_17)
ament_target_dependencies(mission_trigger_node
  rclcpp
  rclcpp_action
  geometry_msgs
)
rosidl_target_interfaces(mission_trigger_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install targets
install(TARGETS pick_place_orchestrator_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install mission_trigger_node to lib/<package_name> for ros2 launch
install(TARGETS mission_trigger_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY behavior_trees launch
  DESTINATION share/${PROJECT_NAME}
)

# Testing Infrastructure
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Skip copyright and cpplint checks for simplicity, let the other linters run
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # gtest target: test_end_to_end_mission
  ament_add_gtest(test_end_to_end_mission test/test_end_to_end_mission.cpp)
  target_include_directories(test_end_to_end_mission PRIVATE include)
  target_compile_features(test_end_to_end_mission PUBLIC cxx_std_17)
  ament_target_dependencies(test_end_to_end_mission
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_end_to_end_mission pick_place_orchestrator_lib)
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_end_to_end_mission "${cpp_typesupport_target}")

  # gtest target: test_payload_attachment
  ament_add_gtest(test_payload_attachment test/test_payload_attachment.cpp)
  target_include_directories(test_payload_attachment PRIVATE include)
  target_compile_features(test_payload_attachment PUBLIC cxx_std_17)
  ament_target_dependencies(test_payload_attachment
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_payload_attachment pick_place_orchestrator_lib)
  target_link_libraries(test_payload_attachment "${cpp_typesupport_target}")

  # gtest target: test_pick_isolation
  ament_add_gtest(test_pick_isolation test/test_pick_isolation.cpp)
  target_include_directories(test_pick_isolation PRIVATE include)
  target_compile_features(test_pick_isolation PUBLIC cxx_std_17)
  ament_target_dependencies(test_pick_isolation
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_pick_isolation pick_place_orchestrator_lib)
  target_link_libraries(test_pick_isolation "${cpp_typesupport_target}")

  # gtest target: test_failure_recovery
  ament_add_gtest(test_failure_recovery test/test_failure_recovery.cpp)
  target_include_directories(test_failure_recovery PRIVATE include)
  target_compile_features(test_failure_recovery PUBLIC cxx_std_17)
  ament_target_dependencies(test_failure_recovery
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_failure_recovery pick_place_orchestrator_lib)
  target_link_libraries(test_failure_recovery "${cpp_typesupport_target}")

  # gtest target: test_deterministic_joint_space
  ament_add_gtest(test_deterministic_joint_space test/test_deterministic_joint_space.cpp)
  target_include_directories(test_deterministic_joint_space PRIVATE include)
  target_compile_features(test_deterministic_joint_space PUBLIC cxx_std_17)
  ament_target_dependencies(test_deterministic_joint_space
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_deterministic_joint_space pick_place_orchestrator_lib)
  target_link_libraries(test_deterministic_joint_space "${cpp_typesupport_target}")

  # gtest target: test_cartesian_constraints
  ament_add_gtest(test_cartesian_constraints test/test_cartesian_constraints.cpp)
  target_include_directories(test_cartesian_constraints PRIVATE include)
  target_compile_features(test_cartesian_constraints PUBLIC cxx_std_17)
  ament_target_dependencies(test_cartesian_constraints
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_cartesian_constraints pick_place_orchestrator_lib)
  target_link_libraries(test_cartesian_constraints "${cpp_typesupport_target}")

  # gtest target: test_planning_fallback
  ament_add_gtest(test_planning_fallback test/test_planning_fallback.cpp)
  target_include_directories(test_planning_fallback PRIVATE include)
  target_compile_features(test_planning_fallback PUBLIC cxx_std_17)
  ament_target_dependencies(test_planning_fallback
    rclcpp
    rclcpp_action
    behaviortree_cpp
    geometry_msgs
    moveit_ros_planning_interface
    base_placement_optimizer
    nav2_msgs
  )
  target_link_libraries(test_planning_fallback pick_place_orchestrator_lib)
  target_link_libraries(test_planning_fallback "${cpp_typesupport_target}")
endif()

ament_export_include_directories(include)
ament_export_libraries(pick_place_orchestrator_lib)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  behaviortree_cpp
  geometry_msgs
  moveit_ros_planning_interface
  base_placement_optimizer
  nav2_msgs
)

# Finalize ament package setup
ament_package()
