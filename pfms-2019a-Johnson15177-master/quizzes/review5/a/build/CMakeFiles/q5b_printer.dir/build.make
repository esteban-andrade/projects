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
CMAKE_SOURCE_DIR = /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build

# Include any dependencies generated for this target.
include CMakeFiles/q5b_printer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/q5b_printer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/q5b_printer.dir/flags.make

CMakeFiles/q5b_printer.dir/printer.cpp.o: CMakeFiles/q5b_printer.dir/flags.make
CMakeFiles/q5b_printer.dir/printer.cpp.o: ../printer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/q5b_printer.dir/printer.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/q5b_printer.dir/printer.cpp.o -c /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/printer.cpp

CMakeFiles/q5b_printer.dir/printer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/q5b_printer.dir/printer.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/printer.cpp > CMakeFiles/q5b_printer.dir/printer.cpp.i

CMakeFiles/q5b_printer.dir/printer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/q5b_printer.dir/printer.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/printer.cpp -o CMakeFiles/q5b_printer.dir/printer.cpp.s

CMakeFiles/q5b_printer.dir/printer.cpp.o.requires:

.PHONY : CMakeFiles/q5b_printer.dir/printer.cpp.o.requires

CMakeFiles/q5b_printer.dir/printer.cpp.o.provides: CMakeFiles/q5b_printer.dir/printer.cpp.o.requires
	$(MAKE) -f CMakeFiles/q5b_printer.dir/build.make CMakeFiles/q5b_printer.dir/printer.cpp.o.provides.build
.PHONY : CMakeFiles/q5b_printer.dir/printer.cpp.o.provides

CMakeFiles/q5b_printer.dir/printer.cpp.o.provides.build: CMakeFiles/q5b_printer.dir/printer.cpp.o


# Object files for target q5b_printer
q5b_printer_OBJECTS = \
"CMakeFiles/q5b_printer.dir/printer.cpp.o"

# External object files for target q5b_printer
q5b_printer_EXTERNAL_OBJECTS =

q5b_printer: CMakeFiles/q5b_printer.dir/printer.cpp.o
q5b_printer: CMakeFiles/q5b_printer.dir/build.make
q5b_printer: CMakeFiles/q5b_printer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable q5b_printer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/q5b_printer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/q5b_printer.dir/build: q5b_printer

.PHONY : CMakeFiles/q5b_printer.dir/build

CMakeFiles/q5b_printer.dir/requires: CMakeFiles/q5b_printer.dir/printer.cpp.o.requires

.PHONY : CMakeFiles/q5b_printer.dir/requires

CMakeFiles/q5b_printer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/q5b_printer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/q5b_printer.dir/clean

CMakeFiles/q5b_printer.dir/depend:
	cd /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build /home/user/PMS/pfms-2019a-asherk03/quizzes/quiz5/a/build/CMakeFiles/q5b_printer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/q5b_printer.dir/depend

