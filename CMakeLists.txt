cmake_minimum_required(VERSION 3.8)
project(wrench_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(include)

add_library(wrench_filter_component SHARED src/wrench_filter_node.cpp)
ament_target_dependencies(wrench_filter_component rclcpp rclcpp_components geometry_msgs std_srvs)
rclcpp_components_register_node(wrench_filter_component
  PLUGIN "wrench_filter::WrenchFilter"
  EXECUTABLE wrench_filter_node)

add_library(wrench_watchdog_component SHARED src/wrench_watchdog_node.cpp)
ament_target_dependencies(wrench_watchdog_component
  rclcpp rclcpp_components geometry_msgs controller_manager_msgs std_srvs)
rclcpp_components_register_node(wrench_watchdog_component
  PLUGIN "wrench_filter::WrenchWatchdog"
  EXECUTABLE wrench_watchdog_node)

install(TARGETS wrench_filter_component wrench_watchdog_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_iir_filter test/test_iir_filter.cpp)
  ament_add_gtest(test_watchdog_monitor test/test_watchdog_monitor.cpp)
endif()

ament_export_include_directories(include)
ament_package()
