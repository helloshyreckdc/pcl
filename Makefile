# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:

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

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "No interactive CMake dialog available..."
	/usr/bin/cmake -E echo No\ interactive\ CMake\ dialog\ available.
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/shyreckdc/project/pcl/CMakeFiles /home/shyreckdc/project/pcl/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/shyreckdc/project/pcl/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named draw_model

# Build rule for target.
draw_model: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 draw_model
.PHONY : draw_model

# fast build rule for target.
draw_model/fast:
	$(MAKE) -f CMakeFiles/draw_model.dir/build.make CMakeFiles/draw_model.dir/build
.PHONY : draw_model/fast

#=============================================================================
# Target rules for targets named extract_indices

# Build rule for target.
extract_indices: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 extract_indices
.PHONY : extract_indices

# fast build rule for target.
extract_indices/fast:
	$(MAKE) -f CMakeFiles/extract_indices.dir/build.make CMakeFiles/extract_indices.dir/build
.PHONY : extract_indices/fast

#=============================================================================
# Target rules for targets named interactive_icp

# Build rule for target.
interactive_icp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 interactive_icp
.PHONY : interactive_icp

# fast build rule for target.
interactive_icp/fast:
	$(MAKE) -f CMakeFiles/interactive_icp.dir/build.make CMakeFiles/interactive_icp.dir/build
.PHONY : interactive_icp/fast

#=============================================================================
# Target rules for targets named statistical_removal

# Build rule for target.
statistical_removal: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 statistical_removal
.PHONY : statistical_removal

# fast build rule for target.
statistical_removal/fast:
	$(MAKE) -f CMakeFiles/statistical_removal.dir/build.make CMakeFiles/statistical_removal.dir/build
.PHONY : statistical_removal/fast

draw_model.o: draw_model.cpp.o
.PHONY : draw_model.o

# target to build an object file
draw_model.cpp.o:
	$(MAKE) -f CMakeFiles/draw_model.dir/build.make CMakeFiles/draw_model.dir/draw_model.cpp.o
.PHONY : draw_model.cpp.o

draw_model.i: draw_model.cpp.i
.PHONY : draw_model.i

# target to preprocess a source file
draw_model.cpp.i:
	$(MAKE) -f CMakeFiles/draw_model.dir/build.make CMakeFiles/draw_model.dir/draw_model.cpp.i
.PHONY : draw_model.cpp.i

draw_model.s: draw_model.cpp.s
.PHONY : draw_model.s

# target to generate assembly for a file
draw_model.cpp.s:
	$(MAKE) -f CMakeFiles/draw_model.dir/build.make CMakeFiles/draw_model.dir/draw_model.cpp.s
.PHONY : draw_model.cpp.s

extract_indices.o: extract_indices.cpp.o
.PHONY : extract_indices.o

# target to build an object file
extract_indices.cpp.o:
	$(MAKE) -f CMakeFiles/extract_indices.dir/build.make CMakeFiles/extract_indices.dir/extract_indices.cpp.o
.PHONY : extract_indices.cpp.o

extract_indices.i: extract_indices.cpp.i
.PHONY : extract_indices.i

# target to preprocess a source file
extract_indices.cpp.i:
	$(MAKE) -f CMakeFiles/extract_indices.dir/build.make CMakeFiles/extract_indices.dir/extract_indices.cpp.i
.PHONY : extract_indices.cpp.i

extract_indices.s: extract_indices.cpp.s
.PHONY : extract_indices.s

# target to generate assembly for a file
extract_indices.cpp.s:
	$(MAKE) -f CMakeFiles/extract_indices.dir/build.make CMakeFiles/extract_indices.dir/extract_indices.cpp.s
.PHONY : extract_indices.cpp.s

interactive_icp.o: interactive_icp.cpp.o
.PHONY : interactive_icp.o

# target to build an object file
interactive_icp.cpp.o:
	$(MAKE) -f CMakeFiles/interactive_icp.dir/build.make CMakeFiles/interactive_icp.dir/interactive_icp.cpp.o
.PHONY : interactive_icp.cpp.o

interactive_icp.i: interactive_icp.cpp.i
.PHONY : interactive_icp.i

# target to preprocess a source file
interactive_icp.cpp.i:
	$(MAKE) -f CMakeFiles/interactive_icp.dir/build.make CMakeFiles/interactive_icp.dir/interactive_icp.cpp.i
.PHONY : interactive_icp.cpp.i

interactive_icp.s: interactive_icp.cpp.s
.PHONY : interactive_icp.s

# target to generate assembly for a file
interactive_icp.cpp.s:
	$(MAKE) -f CMakeFiles/interactive_icp.dir/build.make CMakeFiles/interactive_icp.dir/interactive_icp.cpp.s
.PHONY : interactive_icp.cpp.s

statistical_removal.o: statistical_removal.cpp.o
.PHONY : statistical_removal.o

# target to build an object file
statistical_removal.cpp.o:
	$(MAKE) -f CMakeFiles/statistical_removal.dir/build.make CMakeFiles/statistical_removal.dir/statistical_removal.cpp.o
.PHONY : statistical_removal.cpp.o

statistical_removal.i: statistical_removal.cpp.i
.PHONY : statistical_removal.i

# target to preprocess a source file
statistical_removal.cpp.i:
	$(MAKE) -f CMakeFiles/statistical_removal.dir/build.make CMakeFiles/statistical_removal.dir/statistical_removal.cpp.i
.PHONY : statistical_removal.cpp.i

statistical_removal.s: statistical_removal.cpp.s
.PHONY : statistical_removal.s

# target to generate assembly for a file
statistical_removal.cpp.s:
	$(MAKE) -f CMakeFiles/statistical_removal.dir/build.make CMakeFiles/statistical_removal.dir/statistical_removal.cpp.s
.PHONY : statistical_removal.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... edit_cache"
	@echo "... rebuild_cache"
	@echo "... interactive_icp"
	@echo "... statistical_removal"
	@echo "... extract_indices"
	@echo "... draw_model"
	@echo "... draw_model.o"
	@echo "... draw_model.i"
	@echo "... draw_model.s"
	@echo "... extract_indices.o"
	@echo "... extract_indices.i"
	@echo "... extract_indices.s"
	@echo "... interactive_icp.o"
	@echo "... interactive_icp.i"
	@echo "... interactive_icp.s"
	@echo "... statistical_removal.o"
	@echo "... statistical_removal.i"
	@echo "... statistical_removal.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

