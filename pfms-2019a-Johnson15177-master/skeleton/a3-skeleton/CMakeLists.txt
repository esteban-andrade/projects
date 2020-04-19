cmake_minimum_required(VERSION 2.8.3)
project(a3_simulator)

# Importing the necessary external packages
find_package( Threads REQUIRED )
find_package( OpenCV REQUIRED )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/lib
)


#For linking against it
message ( STATUS "Looking for libray in ${CMAKE_SOURCE_DIR}/lib" )
link_directories(${CMAKE_SOURCE_DIR}/lib)

# Declare a C++ executable
add_executable(${PROJECT_NAME}_example main.cpp)

# Link the library
target_link_libraries(${PROJECT_NAME}_example ${OpenCV_LIBS} simulator)
