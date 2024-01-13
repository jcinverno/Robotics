cmake_minimum_required(VERSION 3.8)
project(my_bt_navigator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependenciesS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# Set target dependencies
set(TARGET_DEPENDS
  rclcpp
  behaviortree_cpp_v3
  yaml-cpp
  rclcpp_action
  nav2_msgs
  tf2
  tf2_geometry_msgs
  rclcpp_lifecycle
  builtin_interfaces
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  std_msgs
  std_srvs
  nav2_util
  pluginlib
  nav2_behavior_tree
)

# Include necessary directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)

# Create the plugin library
add_library(my_bt_navigator_condition_bt_node SHARED /home/trsa2024/turtlebot3_ws/src/my_bt_navigator/plugins/detect_cone.cpp)

# List of libraries to link against
list(APPEND plugin_libs
    my_bt_navigator_condition_bt_node
    ${rclcpp_LIBRARIES}
    ${behaviortree_cpp_v3_LIBRARIES}
)

# Link against the necessary libraries
target_link_libraries(${plugin_libs})

# Specify dependencies for the plugin library
foreach(bt_plugin my_bt_navigator_condition_bt_node)
    ament_target_dependencies(${bt_plugin} ${TARGET_DEPENDS})
    target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()


# Install the plugin library
install(TARGETS my_bt_navigator_condition_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
