cmake_minimum_required(VERSION 3.8)
project(base_placement_optimizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/OptimizePlacement.action"
  DEPENDENCIES geometry_msgs action_msgs
)

add_library(optimizer_node SHARED src/optimizer_node.cpp)
target_include_directories(optimizer_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(optimizer_node PUBLIC cxx_std_17)
ament_target_dependencies(optimizer_node
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  tf2_ros
  tf2_eigen)
# Use target_link_libraries for rosidl_target_interfaces when using ament_cmake
rosidl_target_interfaces(optimizer_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

rclcpp_components_register_node(optimizer_node
  PLUGIN "base_placement_optimizer::BasePlacementOptimizerNode"
  EXECUTABLE optimizer_node_exe)

install(TARGETS optimizer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

install(PROGRAMS scripts/run_task.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_optimizer test/test_optimizer.cpp)
  target_include_directories(test_optimizer PRIVATE include)
  target_compile_features(test_optimizer PUBLIC cxx_std_17)
  ament_target_dependencies(test_optimizer
    rclcpp
    rclcpp_action
    rclcpp_components
    nav_msgs
    geometry_msgs
    moveit_core
    moveit_ros_planning
    tf2_ros
    tf2_eigen)
  target_link_libraries(test_optimizer optimizer_node)

  # For action interfaces
  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
  target_link_libraries(test_optimizer "${cpp_typesupport_target}")
endif()

ament_export_include_directories(include)
ament_export_libraries(optimizer_node)
ament_export_dependencies(
  rclcpp
  rclcpp_action
  rclcpp_components
  nav_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  tf2_ros
  tf2_eigen)

ament_package()
