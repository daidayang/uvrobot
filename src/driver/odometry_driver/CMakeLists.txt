cmake_minimum_required(VERSION 2.8.3)
project(odometry_driver)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  tf
  serial
  tf2
  tf2_eigen
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  #include
  ${catkin_INCLUDE_DIRS}
  #${vehicle_control_INCLUDE_DIRS}
)



catkin_package(
  #INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs
)
 

add_executable(odometry_driver src/odometry_driver.hpp src/odometry_driver.cpp src/odometry_driver_node.cpp)
target_link_libraries(odometry_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(odometry_driver ${catkin_EXPORTED_TARGETS})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)