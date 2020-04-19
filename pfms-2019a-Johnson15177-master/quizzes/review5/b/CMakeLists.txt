# The minimum version of CMake Required
cmake_minimum_required (VERSION 2.8)

# Any project name will suffice, this has connotaions when using advanced CMake Features
set(PROJECT_NAME quiz5b)
project (${PROJECT_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# Add the library you want to test
add_library(complex_lib complex.cpp)

##### Testing Setup
add_subdirectory(lib/gtest-1.6.0)
enable_testing()

# Include the gtest library, the gtest_SOURCE_DIR is from 'project(gtest)'
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

# Add the unit tests
add_executable(q5bTests test_complex.cpp)

# Standard linking to gtest
target_link_libraries(q5bTests gtest gtest_main)
target_link_libraries(q5bTests complex_lib)
