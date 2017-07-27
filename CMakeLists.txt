cmake_minimum_required(VERSION 2.8.3)
project(nav_pcontroller)

find_package(catkin REQUIRED COMPONENTS roscpp roslib tf visualization_msgs sensor_msgs
  geometry_msgs move_base_msgs actionlib)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS roscpp roslib tf visualization_msgs sensor_msgs geometry_msgs move_base_msgs actionlib
)

add_executable(${PROJECT_NAME} src/nav_pcontroller.cc src/BaseDistance.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(speed_filter src/speed_filter.cc src/BaseDistance.cc)
target_link_libraries(speed_filter ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} speed_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES launch/pr2_nav_pcontroller.launch
              launch/boxy_nav_pcontroller.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

