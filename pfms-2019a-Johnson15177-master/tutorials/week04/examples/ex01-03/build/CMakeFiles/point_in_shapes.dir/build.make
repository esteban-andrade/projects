# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build

# Include any dependencies generated for this target.
include CMakeFiles/point_in_shapes.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/point_in_shapes.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/point_in_shapes.dir/flags.make

CMakeFiles/point_in_shapes.dir/main.cpp.o: CMakeFiles/point_in_shapes.dir/flags.make
CMakeFiles/point_in_shapes.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/point_in_shapes.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/point_in_shapes.dir/main.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/main.cpp

CMakeFiles/point_in_shapes.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/point_in_shapes.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/main.cpp > CMakeFiles/point_in_shapes.dir/main.cpp.i

CMakeFiles/point_in_shapes.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/point_in_shapes.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/main.cpp -o CMakeFiles/point_in_shapes.dir/main.cpp.s

CMakeFiles/point_in_shapes.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/point_in_shapes.dir/main.cpp.o.requires

CMakeFiles/point_in_shapes.dir/main.cpp.o.provides: CMakeFiles/point_in_shapes.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/point_in_shapes.dir/build.make CMakeFiles/point_in_shapes.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/point_in_shapes.dir/main.cpp.o.provides

CMakeFiles/point_in_shapes.dir/main.cpp.o.provides.build: CMakeFiles/point_in_shapes.dir/main.cpp.o


# Object files for target point_in_shapes
point_in_shapes_OBJECTS = \
"CMakeFiles/point_in_shapes.dir/main.cpp.o"

# External object files for target point_in_shapes
point_in_shapes_EXTERNAL_OBJECTS =

point_in_shapes: CMakeFiles/point_in_shapes.dir/main.cpp.o
point_in_shapes: CMakeFiles/point_in_shapes.dir/build.make
point_in_shapes: libshapes.a
point_in_shapes: CMakeFiles/point_in_shapes.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable point_in_shapes"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/point_in_shapes.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/point_in_shapes.dir/build: point_in_shapes

.PHONY : CMakeFiles/point_in_shapes.dir/build

CMakeFiles/point_in_shapes.dir/requires: CMakeFiles/point_in_shapes.dir/main.cpp.o.requires

.PHONY : CMakeFiles/point_in_shapes.dir/requires

CMakeFiles/point_in_shapes.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/point_in_shapes.dir/cmake_clean.cmake
.PHONY : CMakeFiles/point_in_shapes.dir/clean

CMakeFiles/point_in_shapes.dir/depend:
	cd /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03 /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03 /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build /home/user/git/pfms-2019a-Johnson15177/tutorials/week04/examples/ex01-03/build/CMakeFiles/point_in_shapes.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/point_in_shapes.dir/depend

