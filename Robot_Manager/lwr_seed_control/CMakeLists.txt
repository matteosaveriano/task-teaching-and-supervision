cmake_minimum_required(VERSION 2.8.3)
project(lwr_seed_control)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf kuka_seed_commands message_generation)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl)
find_package(kdl_parser)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# MESSAGES
add_message_files(FILES DMPData.msg  DMPPoint.msg  DMPTraj.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES robotBehaviourManager
  CATKIN_DEPENDS
    roscpp
    roslib
    rospy
    std_msgs
    kuka_seed_commands
    kdl_parser
    tf
	message_runtime
  DEPENDS
    EIGEN3
)

set(sources_ds
    src/GenerateDSTrajectory.cpp
    src/DMP/dmp.cpp
    src/DMP/fourier_approx.cpp
    src/DMP/radial_approx.cpp
    src/DMP/linear_approx.cpp
)

# LIBRARIES
add_library(robotBehaviourManager src/robotBehaviourManager.cpp  src/robotControl.cpp ${sources_ds})
target_link_libraries(robotBehaviourManager commandManager -lboost_serialization ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES}  ${orocos_kdl_LIBRARIES})

# EXECUTABLES
add_executable(learnTask src/learnTask.cpp)
target_link_libraries(learnTask robotBehaviourManager)

add_executable(executeTask src/executeTask.cpp)
target_link_libraries(executeTask robotBehaviourManager)

# Dummy executable with same name of the project
add_executable(lwr_seed_control src/executeTask.cpp)
target_link_libraries(lwr_seed_control robotBehaviourManager)
