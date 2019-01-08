## Name        :  src/CMakeLists.txt
## Author      :  Aaron
## Purpose     :  CMake input file.

# Tell CMake to compile these files into a library called "Utils"
#find_package(Qt5Widgets REQUIRED)
set(Qt5Core_DIR "/opt/Qt5.7.0/5.7/gcc_64/lib/cmake/Qt5Core")

find_package(PCL 1.2 REQUIRED)
find_package(Qt5Core REQUIRED)
#find_package(Readline REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Qt5Core_INCLUDE_DIRS})
include_directories("${INCLUDE_DIRECTORIES} /home/robot/Documents/Group3/libraries/GeometricTools/GTEngine/Include")
include_directories("${INCLUDE_DIRECTORIES} /opt/Qt5.7.0/5.7/gcc_64/include")

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

#link_directories("/usr/local/lib")
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fPIC")

qt5_wrap_cpp(Engine2_SRC ../headers/Engine2.h Engine2.cpp)


# Removed Shell.cpp
add_library(Utils 
   RobotExt.cpp stringmanip.cpp Logger.cpp  Engine.cpp PlaneFilter.cpp TwoPlaneFilter.cpp Camera.cpp 
   CameraFile.cpp PCLUtils.cpp ClusterFinder.cpp WSObject.cpp plane_clipper_3D.cpp pcl_visualizer_instantiations.cpp
   Calibration.cpp Engine2.cpp ${Engine2_SRC} QLogger.cpp Clawduino.cpp Coordinate.cpp SurfaceMap.cpp)
#add_library(GteLib SurfaceMap.cpp)
link_directories(${LINK_DIRECTORIES} "/opt/Qt5.7.0/5.7/gcc_64/lib")


target_link_libraries(Utils Qt5::Core)

