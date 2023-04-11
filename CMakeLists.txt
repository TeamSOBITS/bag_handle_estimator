cmake_minimum_required(VERSION 3.0.2)
project(bag_handle_estimator)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    image_transport
    pcl_conversions
    pcl_ros
    message_generation
    tf
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

catkin_package(
  CATKIN_DEPENDS
  std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bag_handle_estimator src/handle_estimator.cpp)
target_link_libraries(bag_handle_estimator ${catkin_LIBRARIES})
add_dependencies(bag_handle_estimator ${PROJECT_NAME}_generate_messages_cpp)
