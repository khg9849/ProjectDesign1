cmake_minimum_required(VERSION 3.2)
project(projSeo)
find_package(OpenCV REQUIRED)

INCLUDE_DIRECTORIES(
	${OpenCV_INCLUDE_DIRS}
#   ${jsoncpp_INCLUDE_DIR}
	../src
   ./
	../../darknet/include
)

LINK_DIRECTORIES(
	../../darknet
)

LIST(APPEND LINK_LIBS
	${OpenCV_LIBS}
	darknet
#   ${CMAKE_THREAD_LIBS_INIT}
#   ${Jsoncpp_LIBRARY}
#    mysqlclient
#    jsoncpp
#    modbus
)


file(GLOB SOURCEFILES
	../src/*.cpp
)

# Create library
ADD_LIBRARY(libName SHARED ${SOURCEFILES})
TARGET_LINK_LIBRARIES(libName ${LINK_LIBS})

add_executable(${CMAKE_PROJECT_NAME} testSeo.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} libName)

