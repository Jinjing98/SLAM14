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
CMAKE_SOURCE_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch9

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch9/build

# Include any dependencies generated for this target.
include test/CMakeFiles/run_vo.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/run_vo.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/run_vo.dir/flags.make

test/CMakeFiles/run_vo.dir/run_vo.o: test/CMakeFiles/run_vo.dir/flags.make
test/CMakeFiles/run_vo.dir/run_vo.o: ../test/run_vo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/run_vo.dir/run_vo.o"
	cd /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_vo.dir/run_vo.o -c /home/jinjing/slam_14/jinjing_slambook2/ch9/test/run_vo.cpp

test/CMakeFiles/run_vo.dir/run_vo.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_vo.dir/run_vo.i"
	cd /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jinjing/slam_14/jinjing_slambook2/ch9/test/run_vo.cpp > CMakeFiles/run_vo.dir/run_vo.i

test/CMakeFiles/run_vo.dir/run_vo.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_vo.dir/run_vo.s"
	cd /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jinjing/slam_14/jinjing_slambook2/ch9/test/run_vo.cpp -o CMakeFiles/run_vo.dir/run_vo.s

test/CMakeFiles/run_vo.dir/run_vo.o.requires:

.PHONY : test/CMakeFiles/run_vo.dir/run_vo.o.requires

test/CMakeFiles/run_vo.dir/run_vo.o.provides: test/CMakeFiles/run_vo.dir/run_vo.o.requires
	$(MAKE) -f test/CMakeFiles/run_vo.dir/build.make test/CMakeFiles/run_vo.dir/run_vo.o.provides.build
.PHONY : test/CMakeFiles/run_vo.dir/run_vo.o.provides

test/CMakeFiles/run_vo.dir/run_vo.o.provides.build: test/CMakeFiles/run_vo.dir/run_vo.o


# Object files for target run_vo
run_vo_OBJECTS = \
"CMakeFiles/run_vo.dir/run_vo.o"

# External object files for target run_vo
run_vo_EXTERNAL_OBJECTS =

../bin/run_vo: test/CMakeFiles/run_vo.dir/run_vo.o
../bin/run_vo: test/CMakeFiles/run_vo.dir/build.make
../bin/run_vo: ../lib/libmyslam.so
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_face.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_video.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_text.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.3.2.0
../bin/run_vo: /usr/lib/x86_64-linux-gnu/libopencv_core.so.3.2.0
../bin/run_vo: /home/jinjing/package/Sophus/build/libSophus.so
../bin/run_vo: test/CMakeFiles/run_vo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch9/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/run_vo"
	cd /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_vo.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/run_vo.dir/build: ../bin/run_vo

.PHONY : test/CMakeFiles/run_vo.dir/build

test/CMakeFiles/run_vo.dir/requires: test/CMakeFiles/run_vo.dir/run_vo.o.requires

.PHONY : test/CMakeFiles/run_vo.dir/requires

test/CMakeFiles/run_vo.dir/clean:
	cd /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test && $(CMAKE_COMMAND) -P CMakeFiles/run_vo.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/run_vo.dir/clean

test/CMakeFiles/run_vo.dir/depend:
	cd /home/jinjing/slam_14/jinjing_slambook2/ch9/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jinjing/slam_14/jinjing_slambook2/ch9 /home/jinjing/slam_14/jinjing_slambook2/ch9/test /home/jinjing/slam_14/jinjing_slambook2/ch9/build /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test /home/jinjing/slam_14/jinjing_slambook2/ch9/build/test/CMakeFiles/run_vo.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/run_vo.dir/depend

