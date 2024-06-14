cmake_minimum_required(VERSION 3.8)
project(my_launch)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(nav2_map_server REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(OGRE REQUIRED)
find_package(custom_path REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_util REQUIRED)

# Enable automatic MOC
set(CMAKE_AUTOMOC ON)

# Define RViz plugin
set(rviz_plugin_sources ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/src/myCustomDisplay.cpp)
set(rviz_plugin_headers ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/include/myCustomDisplay.hpp)

add_library(my_launch_plugin SHARED ${rviz_plugin_sources})
ament_target_dependencies(my_launch_plugin rclcpp rviz_common rviz_rendering rviz_default_plugins geometry_msgs)

target_include_directories(my_launch_plugin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/include)
target_link_libraries(my_launch_plugin ${OGRE_LIBRARIES} rviz_common::rviz_common)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS my_launch_plugin ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/rviz_plugin/include/ DESTINATION include/${PROJECT_NAME})

# Define Costmap plugin
set(costmap_plugin_sources ${CMAKE_CURRENT_SOURCE_DIR}/costmap_plugin/src/MyCostmapPlugin.cpp)
set(costmap_plugin_headers ${CMAKE_CURRENT_SOURCE_DIR}/costmap_plugin/include/MyCostmapPlugin.hpp)

add_library(my_costmap_plugin SHARED ${costmap_plugin_sources})
ament_target_dependencies(my_costmap_plugin rclcpp nav2_costmap_2d geometry_msgs pluginlib)

target_include_directories(my_costmap_plugin PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/costmap_plugin/include)
target_link_libraries(my_costmap_plugin ${nav2_costmap_2d_LIBRARIES})
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugin_description.xml)

install(TARGETS my_costmap_plugin ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/costmap_plugin/include/ DESTINATION include/${PROJECT_NAME})

# Executables
add_executable(init_pos_pub src/init_pos_pub.cpp)
add_executable(create_path src/create_path.cpp)
add_executable(robotController src/robotController.cpp)
add_executable(work_area_monitor src/work_area_monitor.cpp)
add_executable(controller_monitor src/controller_monitor.cpp)
add_executable(main_controller src/main_controller.cpp)
add_executable(demo src/demo.cpp)
add_executable(kidnap src/kidnap.cpp)
add_executable(costmap_monitor src/costmap_monitor.cpp)
add_executable(my_costmap_node src/my_costmap_node.cpp)

ament_target_dependencies(init_pos_pub rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(create_path rclcpp geometry_msgs nav_msgs std_msgs custom_path)
ament_target_dependencies(robotController rclcpp geometry_msgs nav_msgs tf2 std_msgs)
ament_target_dependencies(controller_monitor rclcpp geometry_msgs nav_msgs tf2 tf2_ros std_msgs custom_path)
ament_target_dependencies(work_area_monitor rclcpp geometry_msgs std_msgs)
ament_target_dependencies(main_controller rclcpp geometry_msgs nav_msgs tf2 std_msgs)
ament_target_dependencies(demo rclcpp geometry_msgs nav_msgs tf2 tf2_ros std_msgs)
ament_target_dependencies(kidnap rclcpp geometry_msgs)
ament_target_dependencies(costmap_monitor rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs std_msgs)
ament_target_dependencies(my_costmap_node rclcpp nav2_costmap_2d nav2_util)

install(TARGETS init_pos_pub create_path robotController work_area_monitor controller_monitor main_controller demo kidnap costmap_monitor my_costmap_node DESTINATION lib/${PROJECT_NAME})

# Installation of other resources
install(DIRECTORY launch params map rviz DESTINATION share/${PROJECT_NAME})

# Finalize package setup
ament_package()