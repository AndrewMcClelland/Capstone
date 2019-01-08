cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(pcl_read)
 
find_package(PCL 1.2 REQUIRED)
aux_source_directory(. SRC_LIST)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Set some damn debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Wall")

# set(CMAKE_C_FLAGS "-O2")
 
add_executable (pcl_read pcl_read.cpp)
target_link_libraries (pcl_read ${PCL_LIBRARIES})
