cmake_minimum_required(VERSION 2.8.3)
project(seed_segment)

find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib geometry_msgs nav_msgs tf message_generation)
find_package(Eigen3 REQUIRED)
find_package(OpenCV)

set(ECLIPSE_SEED_DIR ${PROJECT_SOURCE_DIR}/eclipseclp )
set(ECLIPSE_HEADER_PATH ${PROJECT_SOURCE_DIR}/eclipseclp/include/x86_64_linux )

include_directories(${catkin_INCLUDE_DIRS} ${ECLIPSE_HEADER_PATH} ${POMDP_HEADER_PATH} ${EIGEN3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# MESSAGES
add_message_files(FILES AllObject.msg  Object.msg  ResultField.msg  Result.msg)
generate_messages(DEPENDENCIES geometry_msgs nav_msgs)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    roscpp
    roslib
    rospy
    geometry_msgs
    nav_msgs
    tf
	message_runtime
  DEPENDS
    EIGEN3
)

## Declare a cpp executable
set( SEED_SRC
	 src/seed_wakeUp.cpp
     src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/POMDP.cpp
	 src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/POMDPState.cpp
     src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/Observ.cpp
     src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/DialogFlow.cpp
     src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/FlowState.cpp
     src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/tinyxml2.cpp
)


add_definitions( -D__LINUX__ )

add_executable(seed_segment src/seed.cpp ${SEED_SRC})
add_dependencies(seed_segment ${${PROJECT_NAME}_EXPORTED_TARGETS}) # Tell catkin to compile the messages first
target_link_libraries( seed_segment  	
					   ${catkin_LIBRARIES} 
	                   ${OpenCV_LIBRARIES}
    	               ${PROJECT_SOURCE_DIR}/eclipseclp/lib/x86_64_linux/libeclipse.so
    	               gmp
					   -lgvc
                       -lcgraph
                       -lcdt
)

# COMMENT/UNCOMMENT to remove/add GRAPHVIZ (show(_) behaviour)
# Use the package PkgConfig to detect GRAPHVIZ headers/library files
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GTK3 REQUIRED gtk+-3.0)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(GTK3 REQUIRED libgvc)
# Setup CMake to use GRAPHVIZ, tell the compiler where to look for headers
# and to the linker where to look for libraries
#include_directories(${GTK3_INCLUDE_DIRS})
#link_directories(${GTK3_LIBRARY_DIRS})
# Add other flags to the compiler
#add_definitions(${GTK3_CFLAGS_OTHER})
# Link the target to the GRAPHVIZ libraries
#target_link_libraries( seed_segment -lgvc -lcgraph -lcdt)
