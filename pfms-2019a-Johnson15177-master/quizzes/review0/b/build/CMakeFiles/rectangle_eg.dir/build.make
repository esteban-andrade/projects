# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build

# Include any dependencies generated for this target.
include CMakeFiles/rectangle_eg.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/rectangle_eg.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/rectangle_eg.dir/flags.make

CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o: CMakeFiles/rectangle_eg.dir/flags.make
CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o: ../rectangle_main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o -c /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/rectangle_main.cpp

CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/rectangle_main.cpp > CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.i

CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/rectangle_main.cpp -o CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.s

CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.requires:

.PHONY : CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.requires

CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.provides: CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.requires
	$(MAKE) -f CMakeFiles/rectangle_eg.dir/build.make CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.provides.build
.PHONY : CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.provides

CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.provides.build: CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o


CMakeFiles/rectangle_eg.dir/rectangle.cpp.o: CMakeFiles/rectangle_eg.dir/flags.make
CMakeFiles/rectangle_eg.dir/rectangle.cpp.o: ../rectangle.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/rectangle_eg.dir/rectangle.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/rectangle_eg.dir/rectangle.cpp.o -c /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/rectangle.cpp

CMakeFiles/rectangle_eg.dir/rectangle.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/rectangle_eg.dir/rectangle.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/rectangle.cpp > CMakeFiles/rectangle_eg.dir/rectangle.cpp.i

CMakeFiles/rectangle_eg.dir/rectangle.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/rectangle_eg.dir/rectangle.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/rectangle.cpp -o CMakeFiles/rectangle_eg.dir/rectangle.cpp.s

CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.requires:

.PHONY : CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.requires

CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.provides: CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.requires
	$(MAKE) -f CMakeFiles/rectangle_eg.dir/build.make CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.provides.build
.PHONY : CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.provides

CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.provides.build: CMakeFiles/rectangle_eg.dir/rectangle.cpp.o


# Object files for target rectangle_eg
rectangle_eg_OBJECTS = \
"CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o" \
"CMakeFiles/rectangle_eg.dir/rectangle.cpp.o"

# External object files for target rectangle_eg
rectangle_eg_EXTERNAL_OBJECTS =

rectangle_eg: CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o
rectangle_eg: CMakeFiles/rectangle_eg.dir/rectangle.cpp.o
rectangle_eg: CMakeFiles/rectangle_eg.dir/build.make
rectangle_eg: CMakeFiles/rectangle_eg.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable rectangle_eg"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/rectangle_eg.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/rectangle_eg.dir/build: rectangle_eg

.PHONY : CMakeFiles/rectangle_eg.dir/build

CMakeFiles/rectangle_eg.dir/requires: CMakeFiles/rectangle_eg.dir/rectangle_main.cpp.o.requires
CMakeFiles/rectangle_eg.dir/requires: CMakeFiles/rectangle_eg.dir/rectangle.cpp.o.requires

.PHONY : CMakeFiles/rectangle_eg.dir/requires

CMakeFiles/rectangle_eg.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/rectangle_eg.dir/cmake_clean.cmake
.PHONY : CMakeFiles/rectangle_eg.dir/clean

CMakeFiles/rectangle_eg.dir/depend:
	cd /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build /home/nate/git/pfms-2019a-chrisnguyen1403/quizzes/quiz0/b/build/CMakeFiles/rectangle_eg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/rectangle_eg.dir/depend

