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
CMAKE_SOURCE_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch6/build

# Include any dependencies generated for this target.
include CMakeFiles/g2o_PROJ.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/g2o_PROJ.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/g2o_PROJ.dir/flags.make

CMakeFiles/g2o_PROJ.dir/g2o.cpp.o: CMakeFiles/g2o_PROJ.dir/flags.make
CMakeFiles/g2o_PROJ.dir/g2o.cpp.o: ../g2o.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/g2o_PROJ.dir/g2o.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/g2o_PROJ.dir/g2o.cpp.o -c /home/jinjing/slam_14/jinjing_slambook2/ch6/g2o.cpp

CMakeFiles/g2o_PROJ.dir/g2o.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/g2o_PROJ.dir/g2o.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jinjing/slam_14/jinjing_slambook2/ch6/g2o.cpp > CMakeFiles/g2o_PROJ.dir/g2o.cpp.i

CMakeFiles/g2o_PROJ.dir/g2o.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/g2o_PROJ.dir/g2o.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jinjing/slam_14/jinjing_slambook2/ch6/g2o.cpp -o CMakeFiles/g2o_PROJ.dir/g2o.cpp.s

CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.requires:

.PHONY : CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.requires

CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.provides: CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.requires
	$(MAKE) -f CMakeFiles/g2o_PROJ.dir/build.make CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.provides.build
.PHONY : CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.provides

CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.provides.build: CMakeFiles/g2o_PROJ.dir/g2o.cpp.o


# Object files for target g2o_PROJ
g2o_PROJ_OBJECTS = \
"CMakeFiles/g2o_PROJ.dir/g2o.cpp.o"

# External object files for target g2o_PROJ
g2o_PROJ_EXTERNAL_OBJECTS =

g2o_PROJ: CMakeFiles/g2o_PROJ.dir/g2o.cpp.o
g2o_PROJ: CMakeFiles/g2o_PROJ.dir/build.make
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
g2o_PROJ: /usr/local/lib/libg2o_core.so
g2o_PROJ: /usr/local/lib/libg2o_stuff.so
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
g2o_PROJ: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
g2o_PROJ: CMakeFiles/g2o_PROJ.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable g2o_PROJ"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/g2o_PROJ.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/g2o_PROJ.dir/build: g2o_PROJ

.PHONY : CMakeFiles/g2o_PROJ.dir/build

CMakeFiles/g2o_PROJ.dir/requires: CMakeFiles/g2o_PROJ.dir/g2o.cpp.o.requires

.PHONY : CMakeFiles/g2o_PROJ.dir/requires

CMakeFiles/g2o_PROJ.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/g2o_PROJ.dir/cmake_clean.cmake
.PHONY : CMakeFiles/g2o_PROJ.dir/clean

CMakeFiles/g2o_PROJ.dir/depend:
	cd /home/jinjing/slam_14/jinjing_slambook2/ch6/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jinjing/slam_14/jinjing_slambook2/ch6 /home/jinjing/slam_14/jinjing_slambook2/ch6 /home/jinjing/slam_14/jinjing_slambook2/ch6/build /home/jinjing/slam_14/jinjing_slambook2/ch6/build /home/jinjing/slam_14/jinjing_slambook2/ch6/build/CMakeFiles/g2o_PROJ.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/g2o_PROJ.dir/depend

