cmake_minimum_required(VERSION 2.8.3)
project(kuka_seed_commands)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp tf ar_track_alvar)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES commandManager
  CATKIN_DEPENDS
    roscpp
    roslib
    rospy
    std_msgs
    tf
    geometry_msgs
    ar_track_alvar
    ar_track_alvar_msgs
  DEPENDS
    eigen
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_library(commandManager src/commandInterpreter.cpp src/objectsList.cpp)
target_link_libraries(commandManager -ltinyxml  ${catkin_LIBRARIES})

# Ros nodes need an executable with the same name to compile. 
# This is an ObjectSimulator compiled with kuka_seed_commands name
add_executable(kuka_seed_commands src/objectsSimulator.cpp src/objectsList.cpp)
target_link_libraries(kuka_seed_commands -ltinyxml  ${catkin_LIBRARIES})
