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
CMAKE_SOURCE_DIR = "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build"

# Include any dependencies generated for this target.
include CMakeFiles/monochromecamera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/monochromecamera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/monochromecamera.dir/flags.make

CMakeFiles/monochromecamera.dir/main.cpp.o: CMakeFiles/monochromecamera.dir/flags.make
CMakeFiles/monochromecamera.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/monochromecamera.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/monochromecamera.dir/main.cpp.o -c "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/main.cpp"

CMakeFiles/monochromecamera.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/monochromecamera.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/main.cpp" > CMakeFiles/monochromecamera.dir/main.cpp.i

CMakeFiles/monochromecamera.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/monochromecamera.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/main.cpp" -o CMakeFiles/monochromecamera.dir/main.cpp.s

CMakeFiles/monochromecamera.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/monochromecamera.dir/main.cpp.o.requires

CMakeFiles/monochromecamera.dir/main.cpp.o.provides: CMakeFiles/monochromecamera.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/monochromecamera.dir/build.make CMakeFiles/monochromecamera.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/monochromecamera.dir/main.cpp.o.provides

CMakeFiles/monochromecamera.dir/main.cpp.o.provides.build: CMakeFiles/monochromecamera.dir/main.cpp.o


CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o: CMakeFiles/monochromecamera.dir/flags.make
CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o: ../monochromecamera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o -c "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/monochromecamera.cpp"

CMakeFiles/monochromecamera.dir/monochromecamera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/monochromecamera.dir/monochromecamera.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/monochromecamera.cpp" > CMakeFiles/monochromecamera.dir/monochromecamera.cpp.i

CMakeFiles/monochromecamera.dir/monochromecamera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/monochromecamera.dir/monochromecamera.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/monochromecamera.cpp" -o CMakeFiles/monochromecamera.dir/monochromecamera.cpp.s

CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.requires:

.PHONY : CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.requires

CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.provides: CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.requires
	$(MAKE) -f CMakeFiles/monochromecamera.dir/build.make CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.provides.build
.PHONY : CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.provides

CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.provides.build: CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o


# Object files for target monochromecamera
monochromecamera_OBJECTS = \
"CMakeFiles/monochromecamera.dir/main.cpp.o" \
"CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o"

# External object files for target monochromecamera
monochromecamera_EXTERNAL_OBJECTS =

monochromecamera: CMakeFiles/monochromecamera.dir/main.cpp.o
monochromecamera: CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o
monochromecamera: CMakeFiles/monochromecamera.dir/build.make
monochromecamera: CMakeFiles/monochromecamera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable monochromecamera"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/monochromecamera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/monochromecamera.dir/build: monochromecamera

.PHONY : CMakeFiles/monochromecamera.dir/build

CMakeFiles/monochromecamera.dir/requires: CMakeFiles/monochromecamera.dir/main.cpp.o.requires
CMakeFiles/monochromecamera.dir/requires: CMakeFiles/monochromecamera.dir/monochromecamera.cpp.o.requires

.PHONY : CMakeFiles/monochromecamera.dir/requires

CMakeFiles/monochromecamera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/monochromecamera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/monochromecamera.dir/clean

CMakeFiles/monochromecamera.dir/depend:
	cd "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class" "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class" "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build" "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build" "/home/user/git/pfms-2019a-Johnson15177/scratch/review/12544300 - Sensor Class/build/CMakeFiles/monochromecamera.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/monochromecamera.dir/depend

