set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/cmake/")

# TODO is there some way to conditionally define things from the command line?

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_manipulation_shell)

find_package(PCL 1.2 REQUIRED)
find_package(Readline REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${Readline_INCLUDE_DIR})
include_directories(${INCLUDE_DIRECTORIES} /Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/headers)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_manipulation_shell pcl_manipulation_shell.cpp /Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/src/stringmanip.cpp)

target_link_libraries(pcl_manipulation_shell ${PCL_LIBRARIES} ${Readline_LIBRARY})
