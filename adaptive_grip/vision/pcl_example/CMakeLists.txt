cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(openni_grabber)
 
find_package(VTK REQUIRED)
find_package(PCL 1.2 REQUIRED)
aux_source_directory(. SRC_LIST)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Set some damn debug flags
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")

# set(CMAKE_C_FLAGS "-O2")
 
add_executable (openni_grabber openni_viewer_simple.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES} ${VTK_LIBRARIES})
