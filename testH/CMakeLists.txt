cmake_minimum_required(VERSION 3.10)

project(
	hProject
	LANGUAGES CXX
	)

ADD_EXECUTABLE(
	project
	main.cpp
	yeStereoCamera.cpp
	)

ADD_EXECUTABLE(
	calib
	main2.cpp
	calibration.cpp
	)

target_include_directories(
	project
PUBLIC
	${CMAKE_SOURCE_DIR}/includes
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/darknet/include
	)
target_include_directories(
	calib
PUBLIC
	${CMAKE_SOURCE_DIR}/includes
	${OPENCV_INCLUDE_DIRS}
	${CMAKE_SOURCE_DIR}/darknet/include
	)

target_link_libraries(
	project
	${OPENCV_LIBRARIES}
	${CMAKE_SOURCE_DIR}/darknet/libdarknet.so
	)
target_link_libraries(
	calib
	${OPENCV_LIBRARIES}
	${CMAKE_SOURCE_DIR}/darknet/libdarknet.so
	)
