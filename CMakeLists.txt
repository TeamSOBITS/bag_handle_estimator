cmake_minimum_required(VERSION 3.8)
# cmake_minimum_required(VERSION 3.10.2)
project(bag_handle_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
# ## Compile as C++14, supported in ROS Noetic and newer
# add_compile_options(-std=c++14)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(sobits_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# ## Find catkin macros and libraries
# ## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# ## is used, also find other catkin packages
# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     rospy
#     sensor_msgs
#     geometry_msgs
#     std_msgs
#     pcl_ros
#     pcl_conversions
#     tf2
#     tf2_ros
#     tf2_geometry_msgs
#     sobits_msgs
# )

# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
# catkin_package(
#   LIBRARIES ${PROJECT_NAME}
#   CATKIN_DEPENDS
#   roscpp
#   rospy
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   pcl_ros
#   pcl_conversions
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   sobits_msgs
# )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#--- for C++
include_directories(include)
add_executable(handle_estimator src/handle_estimator.cpp)
ament_target_dependencies(handle_estimator rclcpp std_msgs geometry_msgs sensor_msgs pcl_conversions pcl_ros tf2_ros sobits_msgs rosidl_default_runtime)

install(TARGETS
handle_estimator
  DESTINATION lib/${PROJECT_NAME}
)
#--- end C++

#-- for Launch
install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/launch)
#--- end Launch

# include_directories(
#   include
#   # ${catkin_INCLUDE_DIRS}
# )
# add_executable(bag_handle_estimator src/handle_estimator.cpp)
# target_link_libraries(bag_handle_estimator ${catkin_LIBRARIES})
# add_dependencies(bag_handle_estimator ${PROJECT_NAME}_generate_messages_cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
