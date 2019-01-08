cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "$ENV{FOURXX}/cmake/" "$ENV{HOME}/Qt5.5.0/5.5/gcc_64") 
#set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH} /opt/Qt5.7.0/5.7/gcc_64/lib/cmake/Qt5Widgets")
set(Qt5Widgets_DIR "/opt/Qt5.7.0/5.7/gcc_64/lib/cmake/Qt5Widgets")
project(pcl_qt_cmake)

#set(PCL_DIR "~/Documents/ELEC490/libraries/pcl/build")
find_package(Qt5Widgets REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(VTK 7.1 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

link_directories("/usr/local/lib")

# This doesn't work
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-inconsistent-missing-override -g")
#Touch
set(FOURXX "/home/robot/Documents/Group3/adaptive_grip")
set(FOURXX_DIR "${FOURXX}")
set(VTK_LIBRARIES vtkRenderingQt vtkViewsQt vtkGUISupportQt)

include_directories(${INCLUDE_DIRECTORIES} "/home/robot/Documents/Group3/original/headers")
include_directories(${INCLUDE_DIRECTORIES} ${PCL_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRECTORIES} ${Readline_INCLUDE_DIR})
include_directories("${INCLUDE_DIRECTORIES} ${FOURXX_DIR}/headers")
include_directories("${INCLUDE_DIRECTORIES} /opt/Qt5.7.0/5.7/gcc_64/include")

# GripperBase Library
#include_directories(${INCLUDE_DIRECTORIES} "/Users/amnicholas/Documents/ELEC490/adaptive_grip_recent/libraries/GantryLib/")

add_subdirectory(${FOURXX_DIR}/src  ${CMAKE_CURRENT_BINARY_DIR}fourxx)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${LINK_DIRECTORIES} "${FOURXX}/libraries/GripperBase")
link_directories(${LINK_DIRECTORIES} "/opt/Qt5.7.0/5.7/gcc_64/lib")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")
#set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

add_definitions(${PCL_DEFINTIONS})

qt5_wrap_ui(UIS_HDRS
    pclviewer2.ui
    gantrywindow.ui
    calibrationwindow.ui)

aux_source_directory(. SRC_LIST)
#link_directories(${LINK_DIRECTORIES} "/home/robot/Documents/Group3/adaptive_grip/hack")

set (SOURCES mainwindow.cpp itemdisplaywindow.cpp chartview.cpp gantrywindow.cpp calibrationwindow.cpp engthread.cpp liveviewer.cpp)
add_executable(${PROJECT_NAME} main.cpp ${SOURCES} ${UIS_HDRS} mainwindow.cpp)
add_executable(calibrate ${SOURCES} ${UIS_HDRS} calibrate.cpp)

target_link_libraries(pcl_qt_cmake ${PCL_LIBRARIES} ${Readline_LIBRARY} ${VTK_LIBRARIES} Qt5::Core Qt5::Widgets Qt5Charts GripperBase boost_thread Qt5Charts Utils )
target_link_libraries(calibrate ${PCL_LIBRARIES} ${Readline_LIBRARY} ${VTK_LIBRARIES} Utils GripperBase boost_thread Qt5Charts Utils )


# qt5_use_modules(${PROJECT_NAME} Widgets)
