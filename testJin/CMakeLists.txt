cmake_minimum_required(VERSION 3.8)

#project info
project(myProject)

# cmake c++11
set(CMAKE_CXX_STANDARD 11)

# compiler
set(CMAKE_C_COMPILER "g++")

# header file, package
find_package(OpenCV REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${CMAKE_SOURCE_DIR}/darknet/include
	../src
	./
) 

link_libraries(
	${OpenCV_LIBS}
	${CMAKE_SOURCE_DIR}/darknet/libdarknet.so
)

 add_executable(main main.cpp yeStereoCamera.cpp)
