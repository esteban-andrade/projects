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
CMAKE_SOURCE_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build

# Include any dependencies generated for this target.
include CMakeFiles/access_pixels.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/access_pixels.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/access_pixels.dir/flags.make

CMakeFiles/access_pixels.dir/access_pixels.cpp.o: CMakeFiles/access_pixels.dir/flags.make
CMakeFiles/access_pixels.dir/access_pixels.cpp.o: ../access_pixels.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/access_pixels.dir/access_pixels.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/access_pixels.dir/access_pixels.cpp.o -c /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/access_pixels.cpp

CMakeFiles/access_pixels.dir/access_pixels.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/access_pixels.dir/access_pixels.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/access_pixels.cpp > CMakeFiles/access_pixels.dir/access_pixels.cpp.i

CMakeFiles/access_pixels.dir/access_pixels.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/access_pixels.dir/access_pixels.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/access_pixels.cpp -o CMakeFiles/access_pixels.dir/access_pixels.cpp.s

CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires:

.PHONY : CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires

CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides: CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires
	$(MAKE) -f CMakeFiles/access_pixels.dir/build.make CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides.build
.PHONY : CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides

CMakeFiles/access_pixels.dir/access_pixels.cpp.o.provides.build: CMakeFiles/access_pixels.dir/access_pixels.cpp.o


# Object files for target access_pixels
access_pixels_OBJECTS = \
"CMakeFiles/access_pixels.dir/access_pixels.cpp.o"

# External object files for target access_pixels
access_pixels_EXTERNAL_OBJECTS =

access_pixels: CMakeFiles/access_pixels.dir/access_pixels.cpp.o
access_pixels: CMakeFiles/access_pixels.dir/build.make
access_pixels: /opt/ros/kinetic/lib/libopencv_stitching3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_superres3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_videostab3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_aruco3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_bgsegm3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_bioinspired3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_ccalib3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_cvv3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_dpm3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_face3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_fuzzy3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_hdf3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_img_hash3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_line_descriptor3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_optflow3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_reg3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_rgbd3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_saliency3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_stereo3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_structured_light3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_surface_matching3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_tracking3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_xfeatures2d3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_ximgproc3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_xobjdetect3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_xphoto3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_shape3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_photo3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_calib3d3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_viz3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_phase_unwrapping3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_video3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_datasets3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_plot3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_text3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_dnn3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_features2d3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_flann3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_highgui3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_ml3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_videoio3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_imgcodecs3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_objdetect3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_imgproc3.so.3.3.1
access_pixels: /opt/ros/kinetic/lib/libopencv_core3.so.3.3.1
access_pixels: CMakeFiles/access_pixels.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable access_pixels"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/access_pixels.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/access_pixels.dir/build: access_pixels

.PHONY : CMakeFiles/access_pixels.dir/build

CMakeFiles/access_pixels.dir/requires: CMakeFiles/access_pixels.dir/access_pixels.cpp.o.requires

.PHONY : CMakeFiles/access_pixels.dir/requires

CMakeFiles/access_pixels.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/access_pixels.dir/cmake_clean.cmake
.PHONY : CMakeFiles/access_pixels.dir/clean

CMakeFiles/access_pixels.dir/depend:
	cd /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build /home/user/git/pfms-2019a-Johnson15177/scratch/tutorial/week08/starter/opencv/access_pixels/build/CMakeFiles/access_pixels.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/access_pixels.dir/depend

