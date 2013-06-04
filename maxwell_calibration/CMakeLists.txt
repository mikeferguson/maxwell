cmake_minimum_required(VERSION 2.8.3)
project(maxwell_calibration)

find_package(catkin)
catkin_package()

install(DIRECTORY capture_data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY estimate_params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY view_results
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#rosbuild_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_maxwell_fk.launch)
