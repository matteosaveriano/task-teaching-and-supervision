cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()


# -g gives information for the sigsegv-backtrace
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g" )


##---Header
#Set location of the .h files for eclipse prolog, player/stage, Open-prs and Pompd
#SET( ECLIPSE_HEADER_PATH $ENV{HOME}/devel/src/eclipseclp/include/x86_64_linux )
set( ECLIPSE_SEED_DIR ${PROJECT_SOURCE_DIR}/eclipseclp )
set( ECLIPSE_HEADER_PATH ${PROJECT_SOURCE_DIR}/eclipseclp/include/x86_64_linux )
#SET( OPRS_HEADER_PATH		 $ENV{ROBOTPKG_BASE}/include/openprs )
#---

include_directories( ${ECLIPSE_HEADER_PATH} )
include_directories( ${POMDP_HEADER_PATH} )
#include_directories( ${OPRS_HEADER_PATH} )


set( EXTERNAL_LIBS
	eclipse
	gmp
	#ExtMP
	opencv_core 
	opencv_highgui

)

## Declare a cpp library
# add_library(seed_2_0
#   src/${PROJECT_NAME}/seed_2_0.cpp
# )

## Declare a cpp executable
set( SEED_SRC
	src/seed_wakeUp.cpp
)


add_definitions( -D__LINUX__ )

rosbuild_add_executable(seed_segment 
        src/seed.cpp
        src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/POMDP.cpp
	src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/POMDPState.cpp
        src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/Observ.cpp
        src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/DialogFlow.cpp
        src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/FlowState.cpp
        src/behaviour/SAPHARI/multimodal_hri_framework/AlfredDialogueManager/tinyxml2.cpp
        ${SEED_SRC})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(seed_2_0_node seed_2_0_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(seed_segment  #${EXTERNAL_LIBS} #${catkin_LIBRARIES} # ${EXTERNAL_LIBS}
	#$ENV{ROBOTPKG_BASE}/lib/libExtMP.a 	
	opencv_core 
	opencv_highgui
    	#$ENV{HOME}/devel/src/eclipseclp/lib/x86_64_linux/libeclipse.so
    	${PROJECT_SOURCE_DIR}/eclipseclp/lib/x86_64_linux/libeclipse.so
    	gmp)


find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})


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
target_link_libraries( seed_segment -lgvc -lcgraph -lcdt)
