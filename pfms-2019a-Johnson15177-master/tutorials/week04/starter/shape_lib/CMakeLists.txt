cmake_minimum_required(VERSION 2.8.3)
project(shape_library)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Build the library 
add_library(yyyyyyy STATIC xxxxxxxx)

#For linking against it
message ( STATUS "Looking for libray in ${CMAKE_BINARY_DIR}" )
link_directories(${CMAKE_BINARY_DIR})
