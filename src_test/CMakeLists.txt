cmake_minimum_required(VERSION 3.10)

project(
	hProject
	LANGUAGES CXX
	)

	
ADD_EXECUTABLE(
	hStereo
	main.cpp
	)
	

target_include_directories(
	hStereo
PUBLIC
	${CMAKE_SOURCE_DIR}/includes
	${CMAKE_SOURCE_DIR}/../../../usr/local/include/opencv4
	#${OPENCV_INCLUDE_DIR}
	${CMAKE_SOURCE_DIR}/darknet/include
	)
	
target_link_libraries(
	hStereo
	${OPENCV_LIBRARIES}
	${CMAKE_SOURCE_DIR}/darknet/libdarknet.so
	)
	
