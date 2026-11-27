cmake_minimum_required(VERSION 3.5)
project(acker_diff_controller)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)

#find_package(ament_cmake_python REQUIRED)

# ROS-free core: kinematics, steering loop, limiting and odometry.
# Must not depend on any ROS package, so it builds identically on every distro.
add_library(acker_diff_core STATIC
  src/acker_diff_core.cpp
  src/odometry.cpp
  src/speed_limiter.cpp
  lib/minimal-PID-controller/pid.cpp
)
set_target_properties(acker_diff_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(acker_diff_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib/minimal-PID-controller>
)

add_library(acker_diff_controller SHARED
  src/acker_diff_controller.cpp
)
target_link_libraries(acker_diff_controller acker_diff_core)
ament_target_dependencies(acker_diff_controller
  builtin_interfaces
  controller_interface
  geometry_msgs
  ackermann_msgs
  hardware_interface
  nav_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  tf2
  tf2_msgs
)
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(acker_diff_controller PRIVATE "acker_diff_controller_BUILDING_DLL")
# prevent pluginlib from using boost
target_compile_definitions(acker_diff_controller PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
pluginlib_export_plugin_description_file(controller_interface acker_diff_plugin.xml)

install(DIRECTORY include/ lib/minimal-PID-controller
  DESTINATION include
)

install(TARGETS acker_diff_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_dependencies(
  ackermann_msgs
  controller_interface
  geometry_msgs
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  tf2
  tf2_msgs
)
ament_export_include_directories(
  include
  include/minimal-PID-controller
)

install(PROGRAMS
  scripts/cmd_vel_to_ackermann.py
  DESTINATION lib/${PROJECT_NAME}
)
#ament_python_install_package(scripts) #conflicts with certain gazebo includes

ament_export_libraries(
  acker_diff_controller
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rosgraph_msgs REQUIRED)

  ament_add_gtest(test_characterisation test/test_characterisation.cpp)
  target_link_libraries(test_characterisation acker_diff_core)

  ament_add_gtest(test_acker_diff_core test/test_acker_diff_core.cpp)
  target_link_libraries(test_acker_diff_core acker_diff_core)

  ament_add_gtest(test_rear_axle_centre test/test_rear_axle_centre.cpp)
  target_link_libraries(test_rear_axle_centre acker_diff_core)

  ament_add_gtest(test_activation test/test_activation.cpp)
  target_include_directories(test_activation PRIVATE include)
  target_link_libraries(test_activation acker_diff_controller)
  ament_target_dependencies(test_activation
    rclcpp controller_interface hardware_interface rosgraph_msgs)
endif()

ament_package()
