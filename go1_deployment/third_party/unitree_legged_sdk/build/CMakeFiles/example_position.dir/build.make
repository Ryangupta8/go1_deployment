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
CMAKE_SOURCE_DIR = /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/example_position.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example_position.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_position.dir/flags.make

CMakeFiles/example_position.dir/example/example_position.cpp.o: CMakeFiles/example_position.dir/flags.make
CMakeFiles/example_position.dir/example/example_position.cpp.o: ../example/example_position.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_position.dir/example/example_position.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example_position.dir/example/example_position.cpp.o -c /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/example/example_position.cpp

CMakeFiles/example_position.dir/example/example_position.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_position.dir/example/example_position.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/example/example_position.cpp > CMakeFiles/example_position.dir/example/example_position.cpp.i

CMakeFiles/example_position.dir/example/example_position.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_position.dir/example/example_position.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/example/example_position.cpp -o CMakeFiles/example_position.dir/example/example_position.cpp.s

CMakeFiles/example_position.dir/example/example_position.cpp.o.requires:

.PHONY : CMakeFiles/example_position.dir/example/example_position.cpp.o.requires

CMakeFiles/example_position.dir/example/example_position.cpp.o.provides: CMakeFiles/example_position.dir/example/example_position.cpp.o.requires
	$(MAKE) -f CMakeFiles/example_position.dir/build.make CMakeFiles/example_position.dir/example/example_position.cpp.o.provides.build
.PHONY : CMakeFiles/example_position.dir/example/example_position.cpp.o.provides

CMakeFiles/example_position.dir/example/example_position.cpp.o.provides.build: CMakeFiles/example_position.dir/example/example_position.cpp.o


# Object files for target example_position
example_position_OBJECTS = \
"CMakeFiles/example_position.dir/example/example_position.cpp.o"

# External object files for target example_position
example_position_EXTERNAL_OBJECTS =

example_position: CMakeFiles/example_position.dir/example/example_position.cpp.o
example_position: CMakeFiles/example_position.dir/build.make
example_position: CMakeFiles/example_position.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_position"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example_position.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_position.dir/build: example_position

.PHONY : CMakeFiles/example_position.dir/build

CMakeFiles/example_position.dir/requires: CMakeFiles/example_position.dir/example/example_position.cpp.o.requires

.PHONY : CMakeFiles/example_position.dir/requires

CMakeFiles/example_position.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example_position.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example_position.dir/clean

CMakeFiles/example_position.dir/depend:
	cd /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build /home/amrl/project/icra-2025/go1_deployment/third_party/unitree_legged_sdk/build/CMakeFiles/example_position.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example_position.dir/depend

