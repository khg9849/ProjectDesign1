
project(testCali)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})



ADD_EXECUTABLE(caliParam caliParam.cpp)
ADD_EXECUTABLE(caliPic caliPic.cpp)

target_link_libraries(caliParam ${OpenCV_LIBS})
target_link_libraries(caliPic ${OpenCV_LIBS})
