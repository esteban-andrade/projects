# The minimum version of CMake Required 
cmake_minimum_required (VERSION 2.6)

# Add project name
project(assignment2) 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# add a target to generate API documentation with Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
add_custom_target(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
)
endif(DOXYGEN_FOUND)

# CMake will now create the library, add the files needed
add_library(ranger_lib laser.cpp radar.cpp ranger.cpp rangerInterface.cpp rangerfusion.cpp rangerfusioninterface.cpp) 

# Cmake files will create executable add the files needed
add_executable(assignment2 main.cpp)


target_link_libraries(assignment2 ranger_lib)
