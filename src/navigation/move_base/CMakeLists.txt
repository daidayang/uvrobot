cmake_minimum_required(VERSION 2.8.3)
project(move_base)

add_definitions(-std=c++11)
find_package(catkin REQUIRED
    COMPONENTS
        actionlib
        base_local_planner
        clear_costmap_recovery
        cmake_modules
        costmap_2d
        dynamic_reconfigure
        geometry_msgs
        message_generation
        move_base_msgs
        nav_core
        nav_msgs
        navfn
        pluginlib
        roscpp
        rospy
        rotate_recovery
        std_srvs
        tf2_geometry_msgs
        tf2_ros
)
find_package(Eigen3 REQUIRED)
add_definitions(${EIGEN3_DEFINITIONS})

add_message_files(
  DIRECTORY 
  msg
  FILES
    TaskData.msg
 )

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/MoveBase.cfg
)

catkin_package(
    CATKIN_DEPENDS
        dynamic_reconfigure
        geometry_msgs
        move_base_msgs
        nav_msgs
        roscpp
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

# move_base
add_library(move_base
  src/move_base.cpp
)
target_link_libraries(move_base
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
    )
add_dependencies(move_base ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_base_node
  src/move_base_node.cpp
)
add_dependencies(move_base_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base_node move_base)
set_target_properties(move_base_node PROPERTIES OUTPUT_NAME move_base)

#path_marker_publisher
add_executable(path_marker_publisher src/path_marker_publisher.cpp)

add_dependencies(path_marker_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_marker_publisher ${Boost_LIBRARIES} ${catkin_LIBRARIES})


#filter_scan_data.cpp
add_executable(filter_scan_data src/filter_scan_data.cpp)
add_dependencies(filter_scan_data ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(filter_scan_data ${Boost_LIBRARIES} ${catkin_LIBRARIES})



add_executable(display_coordinate_system src/display_coordinate_system.cpp)
add_dependencies(display_coordinate_system ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(display_coordinate_system ${Boost_LIBRARIES} ${catkin_LIBRARIES})



install(
    TARGETS
        move_base_node
        display_coordinate_system
        path_marker_publisher
        filter_scan_data
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    TARGETS
        move_base
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(
    TARGETS
        filter_scan_data
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
    TARGETS
        path_marker_publisher
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


install(
    TARGETS
        display_coordinate_system
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
