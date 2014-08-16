cmake_minimum_required(VERSION 2.8.3)
project(dynamic_models)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  geometry_msgs
  nodelet
  roscpp
  std_msgs
  tf
)

#add catkin sourced packages:
catkin_package(
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp nodelet std_msgs geometry_msgs tf gazebo_plugins gazebo_ros
)

#find and add gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${PROTOBUF_INCLUDE_DIR})


#set the default path for built executables to the "bin" directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)


#Controller Executables:


#Plugin Libraries:
add_library(flip_door_plugin src/plugins/flip_door_plugin.cc)
target_link_libraries(flip_door_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${PROTOBUF_LIBRARY})


