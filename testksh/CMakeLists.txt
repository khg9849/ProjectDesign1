cmake_minimum_required(VERSION 3.2)
project(projKSH)
find_package(OpenCV REQUIRED)

include_directories(
	${OpenCV_INCLUDE_DIRS}
	../src
	./
)

LIST(APPEND LINK_LIBS
	${OpenCV_LIBS}
)

#file(GLOB SOURCEFILES
#	../src/*.cpp
#)

#ADD_LIBRARY(libName SHARED ${SOURCEFILES})
#TARGET_LINK_LIBRARIES(libName ${LINK_LIBS})

ADD_EXECUTABLE(caliParam caliParam.cpp)
ADD_EXECUTABLE(caliPic caliPic.cpp)
ADD_EXECUTABLE(caliDetectAndMatch caliDetectAndMatch.cpp)
ADD_EXECUTABLE(testKSH testKSH.cpp yeStereoCamera.cpp)

target_link_libraries(caliParam ${OpenCV_LIBS})
target_link_libraries(caliPic ${OpenCV_LIBS})
target_link_libraries(caliDetectAndMatch ${OpenCV_LIBS})
target_link_libraries(testKSH ${OpenCV_LIBS})
