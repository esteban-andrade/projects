cmake_minimum_required(VERSION 2.8)

# Make PROJECT_SOURCE_DIR, PROJECT_BINARY_DIR, and PROJECT_NAME ava ilable.
project(myproject)

# Define our library
add_library(project1_lib project1.cpp)

add_subdirectory(lib/gtest-1.6.0)
include_directories(${gtest_SOURCE_DIR}/include)

# Create the test executable
add_executable(runUnitTests test/test_project1.cpp)

# Link gtest stuff.
target_link_libraries(runUnitTests gtest gtest_main)
# Link our library
target_link_libraries(runUnitTests project1_lib)
