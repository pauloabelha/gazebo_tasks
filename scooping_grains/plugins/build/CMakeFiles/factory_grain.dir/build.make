# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/cmake-gui

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/build

# Include any dependencies generated for this target.
include CMakeFiles/factory_grain.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/factory_grain.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/factory_grain.dir/flags.make

CMakeFiles/factory_grain.dir/factory_grain.cc.o: CMakeFiles/factory_grain.dir/flags.make
CMakeFiles/factory_grain.dir/factory_grain.cc.o: ../factory_grain.cc
	$(CMAKE_COMMAND) -E cmake_progress_report /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/factory_grain.dir/factory_grain.cc.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/factory_grain.dir/factory_grain.cc.o -c /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/factory_grain.cc

CMakeFiles/factory_grain.dir/factory_grain.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/factory_grain.dir/factory_grain.cc.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/factory_grain.cc > CMakeFiles/factory_grain.dir/factory_grain.cc.i

CMakeFiles/factory_grain.dir/factory_grain.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/factory_grain.dir/factory_grain.cc.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/factory_grain.cc -o CMakeFiles/factory_grain.dir/factory_grain.cc.s

CMakeFiles/factory_grain.dir/factory_grain.cc.o.requires:
.PHONY : CMakeFiles/factory_grain.dir/factory_grain.cc.o.requires

CMakeFiles/factory_grain.dir/factory_grain.cc.o.provides: CMakeFiles/factory_grain.dir/factory_grain.cc.o.requires
	$(MAKE) -f CMakeFiles/factory_grain.dir/build.make CMakeFiles/factory_grain.dir/factory_grain.cc.o.provides.build
.PHONY : CMakeFiles/factory_grain.dir/factory_grain.cc.o.provides

CMakeFiles/factory_grain.dir/factory_grain.cc.o.provides.build: CMakeFiles/factory_grain.dir/factory_grain.cc.o

# Object files for target factory_grain
factory_grain_OBJECTS = \
"CMakeFiles/factory_grain.dir/factory_grain.cc.o"

# External object files for target factory_grain
factory_grain_EXTERNAL_OBJECTS =

libfactory_grain.so: CMakeFiles/factory_grain.dir/factory_grain.cc.o
libfactory_grain.so: CMakeFiles/factory_grain.dir/build.make
libfactory_grain.so: CMakeFiles/factory_grain.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libfactory_grain.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/factory_grain.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/factory_grain.dir/build: libfactory_grain.so
.PHONY : CMakeFiles/factory_grain.dir/build

CMakeFiles/factory_grain.dir/requires: CMakeFiles/factory_grain.dir/factory_grain.cc.o.requires
.PHONY : CMakeFiles/factory_grain.dir/requires

CMakeFiles/factory_grain.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/factory_grain.dir/cmake_clean.cmake
.PHONY : CMakeFiles/factory_grain.dir/clean

CMakeFiles/factory_grain.dir/depend:
	cd /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/build /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/build /home/paulo/.gazebo/gazebo_models/scooping_grains/plugins/build/CMakeFiles/factory_grain.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/factory_grain.dir/depend

