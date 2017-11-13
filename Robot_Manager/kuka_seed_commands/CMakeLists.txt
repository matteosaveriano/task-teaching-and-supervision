cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Find Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

rosbuild_add_library(commandManager src/commandInterpreter.cpp src/objectsList.cpp)
target_link_libraries(commandManager -ltinyxml)

rosbuild_add_executable(objectsSimulator src/objectsSimulator.cpp src/objectsList.cpp)
target_link_libraries(objectsSimulator -ltinyxml)
