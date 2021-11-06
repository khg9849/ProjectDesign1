cmake_minimum_required(VERSION 3.10)

project(
	hProject
	LANGUAGES CXX
	)

ADD_EXECUTABLE(
	hStereo
	featureDetection.cpp
	)

target_include_directories(
	hStereo
PUBLIC
	${CMAKE_SOURCE_DIR}/includes
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/darknet/include
	)

target_link_libraries(
	hStereo
	${OPENCV_LIBRARIES}
	${CMAKE_SOURCE_DIR}/darknet
	)
