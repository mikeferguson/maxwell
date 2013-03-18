cmake_minimum_required(VERSION 2.8.3)
project(maxwell_interactive_markers)

find_package(Boost REQUIRED)
find_package(catkin REQUIRED roscpp visualization_msgs interactive_markers arbotix_msgs simple_arm_server actionlib)
catkin_package()

include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
#add_executable(arm_marker_server src/arm_marker_server.cpp)
#target_link_libraries(arm_marker_server ${Boost_LIBRARIES}
#                                        ${catkin_LIBRARIES} )

#install(TARGETS arm_marker_server
#        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
