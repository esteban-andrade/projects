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
CMAKE_SOURCE_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build

# Include any dependencies generated for this target.
include CMakeFiles/shapes.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/shapes.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/shapes.dir/flags.make

CMakeFiles/shapes.dir/main.cpp.o: CMakeFiles/shapes.dir/flags.make
CMakeFiles/shapes.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/shapes.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shapes.dir/main.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/main.cpp

CMakeFiles/shapes.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shapes.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/main.cpp > CMakeFiles/shapes.dir/main.cpp.i

CMakeFiles/shapes.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shapes.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/main.cpp -o CMakeFiles/shapes.dir/main.cpp.s

CMakeFiles/shapes.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/shapes.dir/main.cpp.o.requires

CMakeFiles/shapes.dir/main.cpp.o.provides: CMakeFiles/shapes.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/shapes.dir/build.make CMakeFiles/shapes.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/shapes.dir/main.cpp.o.provides

CMakeFiles/shapes.dir/main.cpp.o.provides.build: CMakeFiles/shapes.dir/main.cpp.o


CMakeFiles/shapes.dir/shape.cpp.o: CMakeFiles/shapes.dir/flags.make
CMakeFiles/shapes.dir/shape.cpp.o: ../shape.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/shapes.dir/shape.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shapes.dir/shape.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/shape.cpp

CMakeFiles/shapes.dir/shape.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shapes.dir/shape.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/shape.cpp > CMakeFiles/shapes.dir/shape.cpp.i

CMakeFiles/shapes.dir/shape.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shapes.dir/shape.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/shape.cpp -o CMakeFiles/shapes.dir/shape.cpp.s

CMakeFiles/shapes.dir/shape.cpp.o.requires:

.PHONY : CMakeFiles/shapes.dir/shape.cpp.o.requires

CMakeFiles/shapes.dir/shape.cpp.o.provides: CMakeFiles/shapes.dir/shape.cpp.o.requires
	$(MAKE) -f CMakeFiles/shapes.dir/build.make CMakeFiles/shapes.dir/shape.cpp.o.provides.build
.PHONY : CMakeFiles/shapes.dir/shape.cpp.o.provides

CMakeFiles/shapes.dir/shape.cpp.o.provides.build: CMakeFiles/shapes.dir/shape.cpp.o


CMakeFiles/shapes.dir/rectangle.cpp.o: CMakeFiles/shapes.dir/flags.make
CMakeFiles/shapes.dir/rectangle.cpp.o: ../rectangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/shapes.dir/rectangle.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shapes.dir/rectangle.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/rectangle.cpp

CMakeFiles/shapes.dir/rectangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shapes.dir/rectangle.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/rectangle.cpp > CMakeFiles/shapes.dir/rectangle.cpp.i

CMakeFiles/shapes.dir/rectangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shapes.dir/rectangle.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/rectangle.cpp -o CMakeFiles/shapes.dir/rectangle.cpp.s

CMakeFiles/shapes.dir/rectangle.cpp.o.requires:

.PHONY : CMakeFiles/shapes.dir/rectangle.cpp.o.requires

CMakeFiles/shapes.dir/rectangle.cpp.o.provides: CMakeFiles/shapes.dir/rectangle.cpp.o.requires
	$(MAKE) -f CMakeFiles/shapes.dir/build.make CMakeFiles/shapes.dir/rectangle.cpp.o.provides.build
.PHONY : CMakeFiles/shapes.dir/rectangle.cpp.o.provides

CMakeFiles/shapes.dir/rectangle.cpp.o.provides.build: CMakeFiles/shapes.dir/rectangle.cpp.o


CMakeFiles/shapes.dir/triangle.cpp.o: CMakeFiles/shapes.dir/flags.make
CMakeFiles/shapes.dir/triangle.cpp.o: ../triangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/shapes.dir/triangle.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/shapes.dir/triangle.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/triangle.cpp

CMakeFiles/shapes.dir/triangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/shapes.dir/triangle.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/triangle.cpp > CMakeFiles/shapes.dir/triangle.cpp.i

CMakeFiles/shapes.dir/triangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/shapes.dir/triangle.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/triangle.cpp -o CMakeFiles/shapes.dir/triangle.cpp.s

CMakeFiles/shapes.dir/triangle.cpp.o.requires:

.PHONY : CMakeFiles/shapes.dir/triangle.cpp.o.requires

CMakeFiles/shapes.dir/triangle.cpp.o.provides: CMakeFiles/shapes.dir/triangle.cpp.o.requires
	$(MAKE) -f CMakeFiles/shapes.dir/build.make CMakeFiles/shapes.dir/triangle.cpp.o.provides.build
.PHONY : CMakeFiles/shapes.dir/triangle.cpp.o.provides

CMakeFiles/shapes.dir/triangle.cpp.o.provides.build: CMakeFiles/shapes.dir/triangle.cpp.o


# Object files for target shapes
shapes_OBJECTS = \
"CMakeFiles/shapes.dir/main.cpp.o" \
"CMakeFiles/shapes.dir/shape.cpp.o" \
"CMakeFiles/shapes.dir/rectangle.cpp.o" \
"CMakeFiles/shapes.dir/triangle.cpp.o"

# External object files for target shapes
shapes_EXTERNAL_OBJECTS =

shapes: CMakeFiles/shapes.dir/main.cpp.o
shapes: CMakeFiles/shapes.dir/shape.cpp.o
shapes: CMakeFiles/shapes.dir/rectangle.cpp.o
shapes: CMakeFiles/shapes.dir/triangle.cpp.o
shapes: CMakeFiles/shapes.dir/build.make
shapes: CMakeFiles/shapes.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking CXX executable shapes"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/shapes.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/shapes.dir/build: shapes

.PHONY : CMakeFiles/shapes.dir/build

CMakeFiles/shapes.dir/requires: CMakeFiles/shapes.dir/main.cpp.o.requires
CMakeFiles/shapes.dir/requires: CMakeFiles/shapes.dir/shape.cpp.o.requires
CMakeFiles/shapes.dir/requires: CMakeFiles/shapes.dir/rectangle.cpp.o.requires
CMakeFiles/shapes.dir/requires: CMakeFiles/shapes.dir/triangle.cpp.o.requires

.PHONY : CMakeFiles/shapes.dir/requires

CMakeFiles/shapes.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/shapes.dir/cmake_clean.cmake
.PHONY : CMakeFiles/shapes.dir/clean

CMakeFiles/shapes.dir/depend:
	cd /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3 /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3 /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week3/build/CMakeFiles/shapes.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/shapes.dir/depend

