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
CMAKE_SOURCE_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch7

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch7/build

# Include any dependencies generated for this target.
include CMakeFiles/3d3d.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/3d3d.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/3d3d.dir/flags.make

CMakeFiles/3d3d.dir/3d3d.o: CMakeFiles/3d3d.dir/flags.make
CMakeFiles/3d3d.dir/3d3d.o: ../3d3d.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/3d3d.dir/3d3d.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/3d3d.dir/3d3d.o -c /home/jinjing/slam_14/jinjing_slambook2/ch7/3d3d.cpp

CMakeFiles/3d3d.dir/3d3d.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3d3d.dir/3d3d.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jinjing/slam_14/jinjing_slambook2/ch7/3d3d.cpp > CMakeFiles/3d3d.dir/3d3d.i

CMakeFiles/3d3d.dir/3d3d.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3d3d.dir/3d3d.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jinjing/slam_14/jinjing_slambook2/ch7/3d3d.cpp -o CMakeFiles/3d3d.dir/3d3d.s

CMakeFiles/3d3d.dir/3d3d.o.requires:

.PHONY : CMakeFiles/3d3d.dir/3d3d.o.requires

CMakeFiles/3d3d.dir/3d3d.o.provides: CMakeFiles/3d3d.dir/3d3d.o.requires
	$(MAKE) -f CMakeFiles/3d3d.dir/build.make CMakeFiles/3d3d.dir/3d3d.o.provides.build
.PHONY : CMakeFiles/3d3d.dir/3d3d.o.provides

CMakeFiles/3d3d.dir/3d3d.o.provides.build: CMakeFiles/3d3d.dir/3d3d.o


# Object files for target 3d3d
3d3d_OBJECTS = \
"CMakeFiles/3d3d.dir/3d3d.o"

# External object files for target 3d3d
3d3d_EXTERNAL_OBJECTS =

3d3d: CMakeFiles/3d3d.dir/3d3d.o
3d3d: CMakeFiles/3d3d.dir/build.make
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
3d3d: /home/jinjing/package/Sophus/build/libSophus.so
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
3d3d: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
3d3d: CMakeFiles/3d3d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch7/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable 3d3d"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/3d3d.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/3d3d.dir/build: 3d3d

.PHONY : CMakeFiles/3d3d.dir/build

CMakeFiles/3d3d.dir/requires: CMakeFiles/3d3d.dir/3d3d.o.requires

.PHONY : CMakeFiles/3d3d.dir/requires

CMakeFiles/3d3d.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/3d3d.dir/cmake_clean.cmake
.PHONY : CMakeFiles/3d3d.dir/clean

CMakeFiles/3d3d.dir/depend:
	cd /home/jinjing/slam_14/jinjing_slambook2/ch7/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jinjing/slam_14/jinjing_slambook2/ch7 /home/jinjing/slam_14/jinjing_slambook2/ch7 /home/jinjing/slam_14/jinjing_slambook2/ch7/build /home/jinjing/slam_14/jinjing_slambook2/ch7/build /home/jinjing/slam_14/jinjing_slambook2/ch7/build/CMakeFiles/3d3d.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/3d3d.dir/depend

