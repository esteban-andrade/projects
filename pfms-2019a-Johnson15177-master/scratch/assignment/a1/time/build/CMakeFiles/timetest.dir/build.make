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
CMAKE_SOURCE_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build

# Include any dependencies generated for this target.
include CMakeFiles/timetest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/timetest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/timetest.dir/flags.make

CMakeFiles/timetest.dir/time.cpp.o: CMakeFiles/timetest.dir/flags.make
CMakeFiles/timetest.dir/time.cpp.o: ../time.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/timetest.dir/time.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/timetest.dir/time.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/time.cpp

CMakeFiles/timetest.dir/time.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/timetest.dir/time.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/time.cpp > CMakeFiles/timetest.dir/time.cpp.i

CMakeFiles/timetest.dir/time.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/timetest.dir/time.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/time.cpp -o CMakeFiles/timetest.dir/time.cpp.s

CMakeFiles/timetest.dir/time.cpp.o.requires:

.PHONY : CMakeFiles/timetest.dir/time.cpp.o.requires

CMakeFiles/timetest.dir/time.cpp.o.provides: CMakeFiles/timetest.dir/time.cpp.o.requires
	$(MAKE) -f CMakeFiles/timetest.dir/build.make CMakeFiles/timetest.dir/time.cpp.o.provides.build
.PHONY : CMakeFiles/timetest.dir/time.cpp.o.provides

CMakeFiles/timetest.dir/time.cpp.o.provides.build: CMakeFiles/timetest.dir/time.cpp.o


# Object files for target timetest
timetest_OBJECTS = \
"CMakeFiles/timetest.dir/time.cpp.o"

# External object files for target timetest
timetest_EXTERNAL_OBJECTS =

timetest: CMakeFiles/timetest.dir/time.cpp.o
timetest: CMakeFiles/timetest.dir/build.make
timetest: CMakeFiles/timetest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable timetest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/timetest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/timetest.dir/build: timetest

.PHONY : CMakeFiles/timetest.dir/build

CMakeFiles/timetest.dir/requires: CMakeFiles/timetest.dir/time.cpp.o.requires

.PHONY : CMakeFiles/timetest.dir/requires

CMakeFiles/timetest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/timetest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/timetest.dir/clean

CMakeFiles/timetest.dir/depend:
	cd /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/time/build/CMakeFiles/timetest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/timetest.dir/depend
