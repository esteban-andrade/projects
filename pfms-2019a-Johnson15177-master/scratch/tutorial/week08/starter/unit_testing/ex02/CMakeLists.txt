# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Add project name
project (ex02) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# CMake will now create the library
add_library(sensor_lib laser.cpp radar.cpp ranger.cpp rangerinterface.cpp) 

# CMake files will create executable 
add_executable(ex02 main.cpp)

target_link_libraries(ex02 sensor_lib)


# The Following section is for unit testing that we design on the library we have created
# Steps 1-3 do not change

# 1. In case we do not have gtest installed on our system we need to add the below line (which conveys the gtest executable location)
add_subdirectory(lib/gtest-1.6.0)
# 2. Let's unable testing
enable_testing()
# 3. include the gtest library, the gtest_SOURCE_DIR is from 'project(gtest)'
include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
# 4. Add the unit test executable aagainst the source code of the test
add_executable(rawTests test/utest_rawdata.cpp)
# 5, Linking to gtest
target_link_libraries(rawTests gtest gtest_main)
# 6. Also link the the library
target_link_libraries(rawTests sensor_lib)
