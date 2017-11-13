cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# Set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
# Set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()

set(sources_ds
    src/GenerateDSTrajectory.cpp
    src/DMP/dmp.cpp
    src/DMP/fourier_approx.cpp
    src/DMP/radial_approx.cpp
    src/DMP/linear_approx.cpp
)

# Find Eigen library
find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

# LIBRARIES
rosbuild_add_library(robotBehaviourManager src/robotBehaviourManager.cpp  src/robotControl.cpp ${sources_ds})
target_link_libraries(robotBehaviourManager commandManager -lboost_serialization)

# EXECUTABLES
rosbuild_add_executable(learnTask src/learnTask.cpp)
target_link_libraries(learnTask robotBehaviourManager)

rosbuild_add_executable(executeTask src/executeTask.cpp)
target_link_libraries(executeTask robotBehaviourManager)
