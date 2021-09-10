project(
	cameraCalibration
	CXX
	)

ADD_EXECUTABLE(
	oneCamera
	oneCamera.cpp
	)

ADD_EXECUTABLE(
	stereoCamera
	stereoCamera.cpp
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
	oneCamera
	${OPENCV_LIBRARIES}
	)

target_link_libraries(
	stereoCamera
	${OPENCV_LIBRARIES}
	)
