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
CMAKE_SOURCE_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build

# Include any dependencies generated for this target.
include CMakeFiles/fusion_lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/fusion_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/fusion_lib.dir/flags.make

CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o: CMakeFiles/fusion_lib.dir/flags.make
CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o: ../rangerfusioninterface.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/rangerfusioninterface.cpp

CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/rangerfusioninterface.cpp > CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.i

CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/rangerfusioninterface.cpp -o CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.s

CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.requires:

.PHONY : CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.requires

CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.provides: CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.requires
	$(MAKE) -f CMakeFiles/fusion_lib.dir/build.make CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.provides.build
.PHONY : CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.provides

CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.provides.build: CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o


CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o: CMakeFiles/fusion_lib.dir/flags.make
CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o: ../rangerfusion.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/rangerfusion.cpp

CMakeFiles/fusion_lib.dir/rangerfusion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fusion_lib.dir/rangerfusion.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/rangerfusion.cpp > CMakeFiles/fusion_lib.dir/rangerfusion.cpp.i

CMakeFiles/fusion_lib.dir/rangerfusion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fusion_lib.dir/rangerfusion.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/rangerfusion.cpp -o CMakeFiles/fusion_lib.dir/rangerfusion.cpp.s

CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.requires:

.PHONY : CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.requires

CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.provides: CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.requires
	$(MAKE) -f CMakeFiles/fusion_lib.dir/build.make CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.provides.build
.PHONY : CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.provides

CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.provides.build: CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o


# Object files for target fusion_lib
fusion_lib_OBJECTS = \
"CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o" \
"CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o"

# External object files for target fusion_lib
fusion_lib_EXTERNAL_OBJECTS =

libfusion_lib.a: CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o
libfusion_lib.a: CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o
libfusion_lib.a: CMakeFiles/fusion_lib.dir/build.make
libfusion_lib.a: CMakeFiles/fusion_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libfusion_lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/fusion_lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fusion_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/fusion_lib.dir/build: libfusion_lib.a

.PHONY : CMakeFiles/fusion_lib.dir/build

CMakeFiles/fusion_lib.dir/requires: CMakeFiles/fusion_lib.dir/rangerfusioninterface.cpp.o.requires
CMakeFiles/fusion_lib.dir/requires: CMakeFiles/fusion_lib.dir/rangerfusion.cpp.o.requires

.PHONY : CMakeFiles/fusion_lib.dir/requires

CMakeFiles/fusion_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fusion_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fusion_lib.dir/clean

CMakeFiles/fusion_lib.dir/depend:
	cd /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2 /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2 /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build /home/user/git/pfms-2019a-Johnson15177/scratch/assignment/a2attempt2/build/CMakeFiles/fusion_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fusion_lib.dir/depend

