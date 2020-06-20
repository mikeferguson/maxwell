cmake_minimum_required(VERSION 3.5)
project(maxwell)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(etherbotix REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_controllers_interface REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(include)

add_library(torso_controller SHARED
  src/torso_controller.cpp
)
ament_target_dependencies(torso_controller
  etherbotix
  geometry_msgs
  pluginlib
  rclcpp
  robot_controllers_interface
  std_srvs
)

pluginlib_export_plugin_description_file(robot_controllers_interface torso_controller.xml)

if (BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(torso_controller_tests test/torso_controller_tests.cpp)
  target_link_libraries(torso_controller_tests
    torso_controller
  )
endif()

install(
  TARGETS
    torso_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
