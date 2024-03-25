cmake_minimum_required(VERSION 3.10.2)
project(bag_handle_estimator)

## Compile as C++14, supported in ROS Noetic and newer
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    pcl_ros
    pcl_conversions
    tf2
    tf2_ros
    tf2_geometry_msgs

)

add_service_files(
  FILES
  execute_ctrl.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  tf2
  tf2_ros
  tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bag_handle_estimator src/handle_estimator.cpp)
target_link_libraries(bag_handle_estimator ${catkin_LIBRARIES})
add_dependencies(bag_handle_estimator ${PROJECT_NAME}_generate_messages_cpp)
