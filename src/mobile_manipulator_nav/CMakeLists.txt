cmake_minimum_required(VERSION 3.8)
project(mobile_manipulator_nav)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)

add_library(is_arm_stowed_condition_bt_node SHARED
  src/is_arm_stowed_condition.cpp
)
target_include_directories(is_arm_stowed_condition_bt_node PRIVATE
  include
)
target_compile_definitions(is_arm_stowed_condition_bt_node PRIVATE
  BT_PLUGIN_EXPORT
)
ament_target_dependencies(is_arm_stowed_condition_bt_node
  rclcpp
  std_msgs
  behaviortree_cpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_navigation_launch.py)
endif()

# Install targets
install(
  TARGETS is_arm_stowed_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install scripts
install(
  PROGRAMS scripts/arm_state_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
