project(
	cameraCalibration
	CXX
	)

ADD_EXECUTABLE(
	stereoCamera_N
	stereoCamera_N.cpp
	)

ADD_EXECUTABLE(
	stereoCamera_90
	stereoCamera_90.cpp
	)

ADD_EXECUTABLE(
	stereoCamera_ver2
	stereoCamera_ver2.cpp
	)

find_package(
	PkgConfig
	REQUIRED
	)

pkg_check_modules(
	OPENCV
	opencv4
	REQUIRED
	)

include_directories(
	${OPENCV_INCLUDE_DIRS}
	)

target_link_libraries(
	stereoCamera_N
	${OPENCV_LIBRARIES}
	)

target_link_libraries(
	stereoCamera_90
	${OPENCV_LIBRARIES}
	)

target_link_libraries(
	stereoCamera_ver2
	${OPENCV_LIBRARIES}
	)
