# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/shyreckdc/project/pcl

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shyreckdc/project/pcl

# Include any dependencies generated for this target.
include CMakeFiles/statistical_removal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/statistical_removal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/statistical_removal.dir/flags.make

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o: CMakeFiles/statistical_removal.dir/flags.make
CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o: statistical_removal.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/shyreckdc/project/pcl/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o -c /home/shyreckdc/project/pcl/statistical_removal.cpp

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/shyreckdc/project/pcl/statistical_removal.cpp > CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/shyreckdc/project/pcl/statistical_removal.cpp -o CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.requires:
.PHONY : CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.requires

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.provides: CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.requires
	$(MAKE) -f CMakeFiles/statistical_removal.dir/build.make CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.provides.build
.PHONY : CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.provides

CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.provides.build: CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o

# Object files for target statistical_removal
statistical_removal_OBJECTS = \
"CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o"

# External object files for target statistical_removal
statistical_removal_EXTERNAL_OBJECTS =

statistical_removal: CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o
statistical_removal: CMakeFiles/statistical_removal.dir/build.make
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_system.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_thread.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpthread.so
statistical_removal: /usr/lib/libpcl_common.so
statistical_removal: /usr/lib/libpcl_octree.so
statistical_removal: /usr/lib/libOpenNI.so
statistical_removal: /usr/lib/libpcl_io.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
statistical_removal: /usr/lib/libpcl_kdtree.so
statistical_removal: /usr/lib/libpcl_search.so
statistical_removal: /usr/lib/libpcl_sample_consensus.so
statistical_removal: /usr/lib/libpcl_filters.so
statistical_removal: /usr/lib/libpcl_features.so
statistical_removal: /usr/lib/libpcl_keypoints.so
statistical_removal: /usr/lib/libpcl_segmentation.so
statistical_removal: /usr/lib/libpcl_visualization.so
statistical_removal: /usr/lib/libpcl_outofcore.so
statistical_removal: /usr/lib/libpcl_registration.so
statistical_removal: /usr/lib/libpcl_recognition.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libqhull.so
statistical_removal: /usr/lib/libpcl_surface.so
statistical_removal: /usr/lib/libpcl_people.so
statistical_removal: /usr/lib/libpcl_tracking.so
statistical_removal: /usr/lib/libpcl_apps.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_system.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_thread.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libpthread.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libqhull.so
statistical_removal: /usr/lib/libOpenNI.so
statistical_removal: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
statistical_removal: /usr/lib/libvtkCharts.so.5.8.0
statistical_removal: /usr/lib/libpcl_common.so
statistical_removal: /usr/lib/libpcl_octree.so
statistical_removal: /usr/lib/libpcl_io.so
statistical_removal: /usr/lib/libpcl_kdtree.so
statistical_removal: /usr/lib/libpcl_search.so
statistical_removal: /usr/lib/libpcl_sample_consensus.so
statistical_removal: /usr/lib/libpcl_filters.so
statistical_removal: /usr/lib/libpcl_features.so
statistical_removal: /usr/lib/libpcl_keypoints.so
statistical_removal: /usr/lib/libpcl_segmentation.so
statistical_removal: /usr/lib/libpcl_visualization.so
statistical_removal: /usr/lib/libpcl_outofcore.so
statistical_removal: /usr/lib/libpcl_registration.so
statistical_removal: /usr/lib/libpcl_recognition.so
statistical_removal: /usr/lib/libpcl_surface.so
statistical_removal: /usr/lib/libpcl_people.so
statistical_removal: /usr/lib/libpcl_tracking.so
statistical_removal: /usr/lib/libpcl_apps.so
statistical_removal: /usr/lib/libvtkViews.so.5.8.0
statistical_removal: /usr/lib/libvtkInfovis.so.5.8.0
statistical_removal: /usr/lib/libvtkWidgets.so.5.8.0
statistical_removal: /usr/lib/libvtkHybrid.so.5.8.0
statistical_removal: /usr/lib/libvtkParallel.so.5.8.0
statistical_removal: /usr/lib/libvtkVolumeRendering.so.5.8.0
statistical_removal: /usr/lib/libvtkRendering.so.5.8.0
statistical_removal: /usr/lib/libvtkGraphics.so.5.8.0
statistical_removal: /usr/lib/libvtkImaging.so.5.8.0
statistical_removal: /usr/lib/libvtkIO.so.5.8.0
statistical_removal: /usr/lib/libvtkFiltering.so.5.8.0
statistical_removal: /usr/lib/libvtkCommon.so.5.8.0
statistical_removal: /usr/lib/libvtksys.so.5.8.0
statistical_removal: CMakeFiles/statistical_removal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable statistical_removal"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/statistical_removal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/statistical_removal.dir/build: statistical_removal
.PHONY : CMakeFiles/statistical_removal.dir/build

CMakeFiles/statistical_removal.dir/requires: CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o.requires
.PHONY : CMakeFiles/statistical_removal.dir/requires

CMakeFiles/statistical_removal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/statistical_removal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/statistical_removal.dir/clean

CMakeFiles/statistical_removal.dir/depend:
	cd /home/shyreckdc/project/pcl && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shyreckdc/project/pcl /home/shyreckdc/project/pcl /home/shyreckdc/project/pcl /home/shyreckdc/project/pcl /home/shyreckdc/project/pcl/CMakeFiles/statistical_removal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/statistical_removal.dir/depend

