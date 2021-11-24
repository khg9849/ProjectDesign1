cmake_minimum_required(VERSION 3.2)

project(projKSH)
find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/../darknet/include
	../src
	./
)

link_libraries(
	${OpenCV_LIBS}
	${CMAKE_SOURCE_DIR}/../darknet/libdarknet.so
)

##ADD_EXECUTABLE(caliDetectAndMatch caliDetectAndMatch.cpp CallYolo.cpp)
ADD_EXECUTABLE(yeStereoCamera main.cpp yeStereoCamera.cpp)
