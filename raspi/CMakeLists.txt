cmake_minimum_required(VERSION 3.1)
project(ObjectRecognitionServer)

# Compiler Config
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Executable
add_executable(ObjectRecognitionServer main.cpp)

# Libraries
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(ObjectRecognitionServer ${OpenCV_LIBS} serial)
