cmake_minimum_required(VERSION 2.8.3)
project(quad_control)

add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  roscpp
  tf
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES quad_controller
  CATKIN_DEPENDS cmake_modules geometry_msgs
  roscpp tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include  ${catkin_INCLUDE_DIRS}
)

add_library(quad_controller
  src/library/quad_controller.cpp
)


target_link_libraries(quad_controller ${catkin_LIBRARIES})
add_dependencies(quad_controller ${catkin_EXPORTED_TARGETS})

add_executable(waypoint_publisher_node src/nodes/waypoint_publisher_node.cpp)
add_dependencies(waypoint_publisher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_publisher_node quad_controller ${catkin_LIBRARIES})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_node quad_controller ${catkin_LIBRARIES})
