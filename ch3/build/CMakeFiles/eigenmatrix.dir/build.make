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
CMAKE_SOURCE_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch3

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jinjing/slam_14/jinjing_slambook2/ch3/build

# Include any dependencies generated for this target.
include CMakeFiles/eigenmatrix.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/eigenmatrix.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/eigenmatrix.dir/flags.make

CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o: CMakeFiles/eigenmatrix.dir/flags.make
CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o: ../useEigen/eigenMatrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o -c /home/jinjing/slam_14/jinjing_slambook2/ch3/useEigen/eigenMatrix.cpp

CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jinjing/slam_14/jinjing_slambook2/ch3/useEigen/eigenMatrix.cpp > CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.i

CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jinjing/slam_14/jinjing_slambook2/ch3/useEigen/eigenMatrix.cpp -o CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.s

CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.requires:

.PHONY : CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.requires

CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.provides: CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.requires
	$(MAKE) -f CMakeFiles/eigenmatrix.dir/build.make CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.provides.build
.PHONY : CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.provides

CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.provides.build: CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o


# Object files for target eigenmatrix
eigenmatrix_OBJECTS = \
"CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o"

# External object files for target eigenmatrix
eigenmatrix_EXTERNAL_OBJECTS =

eigenmatrix: CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o
eigenmatrix: CMakeFiles/eigenmatrix.dir/build.make
eigenmatrix: CMakeFiles/eigenmatrix.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jinjing/slam_14/jinjing_slambook2/ch3/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable eigenmatrix"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/eigenmatrix.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/eigenmatrix.dir/build: eigenmatrix

.PHONY : CMakeFiles/eigenmatrix.dir/build

CMakeFiles/eigenmatrix.dir/requires: CMakeFiles/eigenmatrix.dir/useEigen/eigenMatrix.o.requires

.PHONY : CMakeFiles/eigenmatrix.dir/requires

CMakeFiles/eigenmatrix.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/eigenmatrix.dir/cmake_clean.cmake
.PHONY : CMakeFiles/eigenmatrix.dir/clean

CMakeFiles/eigenmatrix.dir/depend:
	cd /home/jinjing/slam_14/jinjing_slambook2/ch3/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jinjing/slam_14/jinjing_slambook2/ch3 /home/jinjing/slam_14/jinjing_slambook2/ch3 /home/jinjing/slam_14/jinjing_slambook2/ch3/build /home/jinjing/slam_14/jinjing_slambook2/ch3/build /home/jinjing/slam_14/jinjing_slambook2/ch3/build/CMakeFiles/eigenmatrix.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/eigenmatrix.dir/depend

