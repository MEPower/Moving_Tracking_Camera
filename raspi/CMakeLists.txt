cmake_minimum_required(VERSION 2.9)
project(ObjectRecognitionServer)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(ObjectRecognitionServer main.cpp)
target_link_libraries(ObjectRecognitionServer ${OpenCV_LIBS})