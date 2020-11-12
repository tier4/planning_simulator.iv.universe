cmake_minimum_required(VERSION 3.5)
project(scenario_api_simulator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(autoware_lanelet2_msgs REQUIRED)
find_package(autoware_perception_msgs REQUIRED)
find_package(dummy_perception_publisher REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lanelet2_extension REQUIRED)
find_package(npc_simulator REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scenario_api_utils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

add_library(scenario_api_simulator SHARED
  src/scenario_api_simulator.cpp
  src/npc_route_manager.cpp
)
target_include_directories(scenario_api_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(scenario_api_simulator autoware_perception_msgs dummy_perception_publisher geometry_msgs lanelet2_extension npc_simulator rclcpp scenario_api_utils std_msgs tf2 unique_identifier_msgs)

ament_export_targets(export_scenario_api_simulator HAS_LIBRARY_TARGET)
ament_export_dependencies(autoware_perception_msgs geometry_msgs npc_simulator rclcpp std_msgs unique_identifier_msgs)

install(TARGETS scenario_api_simulator
  EXPORT export_scenario_api_simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
