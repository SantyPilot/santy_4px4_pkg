cmake_minimum_required(VERSION 2.8.3)
project(santy_4px4_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  mavros_msgs
  std_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(ctrl_node src/ctrl_node.cpp
                         src/PX4Controller.cpp 
                         src/TargetGenerator.cpp
                         src/utils.cpp)
add_dependencies(ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPORTED_TARGETS})
target_link_libraries(ctrl_node ${catkin_LIBRARIES})

# joy node
add_executable(js_node src/joy_node.cpp)
target_link_libraries(js_node ${catkin_LIBRARIES})
