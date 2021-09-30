cmake_minimum_required(VERSION 2.8)

#project info
project(myProject)

# cmake c++11
set(CMAKE_CXX_STANDARD 11)

# compiler
set(CMAKE_C_COMPILER "g++")

# header file, package
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


add_executable(myProject main.cpp yeStereoCamera.hpp yeStereoCamera.cpp stereo.hpp stereo.cpp)
target_link_libraries(myProject ${OpenCV_LIBS})
