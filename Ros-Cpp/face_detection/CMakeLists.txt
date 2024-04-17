cmake_minimum_required(VERSION 3.0.2)

project(objectdetection)

FIND_PACKAGE(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/opencv4/opencv2)


add_executable(objectdetection objectDetection.cpp)

target_link_libraries(objectdetection PRIVATE ${OpenCV_LIBRARIES})

