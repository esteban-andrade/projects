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
CMAKE_SOURCE_DIR = /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build

# Include any dependencies generated for this target.
include CMakeFiles/arrays.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/arrays.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/arrays.dir/flags.make

CMakeFiles/arrays.dir/arrays.cpp.o: CMakeFiles/arrays.dir/flags.make
CMakeFiles/arrays.dir/arrays.cpp.o: ../arrays.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/arrays.dir/arrays.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/arrays.dir/arrays.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/arrays.cpp

CMakeFiles/arrays.dir/arrays.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/arrays.dir/arrays.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/arrays.cpp > CMakeFiles/arrays.dir/arrays.cpp.i

CMakeFiles/arrays.dir/arrays.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/arrays.dir/arrays.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/arrays.cpp -o CMakeFiles/arrays.dir/arrays.cpp.s

CMakeFiles/arrays.dir/arrays.cpp.o.requires:

.PHONY : CMakeFiles/arrays.dir/arrays.cpp.o.requires

CMakeFiles/arrays.dir/arrays.cpp.o.provides: CMakeFiles/arrays.dir/arrays.cpp.o.requires
	$(MAKE) -f CMakeFiles/arrays.dir/build.make CMakeFiles/arrays.dir/arrays.cpp.o.provides.build
.PHONY : CMakeFiles/arrays.dir/arrays.cpp.o.provides

CMakeFiles/arrays.dir/arrays.cpp.o.provides.build: CMakeFiles/arrays.dir/arrays.cpp.o


# Object files for target arrays
arrays_OBJECTS = \
"CMakeFiles/arrays.dir/arrays.cpp.o"

# External object files for target arrays
arrays_EXTERNAL_OBJECTS =

arrays: CMakeFiles/arrays.dir/arrays.cpp.o
arrays: CMakeFiles/arrays.dir/build.make
arrays: CMakeFiles/arrays.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable arrays"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/arrays.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/arrays.dir/build: arrays

.PHONY : CMakeFiles/arrays.dir/build

CMakeFiles/arrays.dir/requires: CMakeFiles/arrays.dir/arrays.cpp.o.requires

.PHONY : CMakeFiles/arrays.dir/requires

CMakeFiles/arrays.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/arrays.dir/cmake_clean.cmake
.PHONY : CMakeFiles/arrays.dir/clean

CMakeFiles/arrays.dir/depend:
	cd /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build /home/user/git/pfms-2019a-Johnson15177/quizzes/quiz0/a/Build/CMakeFiles/arrays.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/arrays.dir/depend

