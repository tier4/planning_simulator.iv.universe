cmake_minimum_required(VERSION 3.5)
project(npc_simulator)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# message generation

set(msg_files
  msg/LaneChangeDir.msg
  msg/LaneFollowMode.msg
  msg/Object.msg
)
set(service_files
    srv/GetObject.srv
)
set(msg_dependencies
  autoware_perception_msgs
  dummy_perception_publisher
  geometry_msgs
  std_msgs
  unique_identifier_msgs
  )
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${service_files}
  DEPENDENCIES ${msg_dependencies}
  )
ament_export_dependencies(rosidl_default_runtime)

ament_auto_add_library(${PROJECT_NAME}_node STATIC src/node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE "${Boost_INCLUDE_DIR}")
target_link_libraries(${PROJECT_NAME}_node "${Boost_LIBRARY_DIR}")
ament_target_dependencies(${PROJECT_NAME}_node Boost)

ament_auto_add_executable(${PROJECT_NAME}_main
  src/main.cpp
)
# to allow using interfaces created in this package
rosidl_target_interfaces(${PROJECT_NAME}_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# export information to downstream packages
ament_auto_package(
  INSTALL_TO_SHARE launch
)
