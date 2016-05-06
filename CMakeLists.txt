cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

#Taken from the other example, could also be Release
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Also taken from the other thing, don't know what it does yet
rosbuild_add_boost_directories()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#Taken from example project

# Library
rosbuild_add_library(assertive_behaviour_lib src/assertive_behaviour.cpp)
set_target_properties(assertive_behaviour_lib
                      PROPERTIES OUTPUT_NAME "assertive_behaviour")
rosbuild_link_boost(assertive_behaviour_lib thread)

# Explore
rosbuild_add_executable(assertive_behaviour src/main.cpp)
target_link_libraries(assertive_behaviour assertive_behaviour_lib)
#target_link_libraries(explore skeleplanner)
#target_link_libraries(explore topomap)
