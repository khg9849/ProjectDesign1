cmake_minimum_required(VERSION 3.10)

project(
	hProject
	LANGUAGES CXX
	)

ADD_EXECUTABLE(
	yeStereo
	main.cpp
	yeStereoCamera.cpp
	)

target_include_directories(
	yeStereo
PUBLIC
	${CMAKE_SOURCE_DIR}/includes
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/darknet/include
	)

target_link_libraries(
	yeStereo
	${OPENCV_LIBRARIES}
	${CMAKE_SOURCE_DIR}/darknet
	)
