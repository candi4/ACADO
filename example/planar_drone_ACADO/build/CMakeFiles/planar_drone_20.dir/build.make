# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/hojun/project/ACADO/example/planar_drone_ACADO

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hojun/project/ACADO/example/planar_drone_ACADO/build

# Include any dependencies generated for this target.
include CMakeFiles/planar_drone_20.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/planar_drone_20.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/planar_drone_20.dir/flags.make

CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.o: CMakeFiles/planar_drone_20.dir/flags.make
CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.o: ../planar_drone_20.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hojun/project/ACADO/example/planar_drone_ACADO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.o -c /home/hojun/project/ACADO/example/planar_drone_ACADO/planar_drone_20.cpp

CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hojun/project/ACADO/example/planar_drone_ACADO/planar_drone_20.cpp > CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.i

CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hojun/project/ACADO/example/planar_drone_ACADO/planar_drone_20.cpp -o CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.s

# Object files for target planar_drone_20
planar_drone_20_OBJECTS = \
"CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.o"

# External object files for target planar_drone_20
planar_drone_20_EXTERNAL_OBJECTS =

planar_drone_20: CMakeFiles/planar_drone_20.dir/planar_drone_20.cpp.o
planar_drone_20: CMakeFiles/planar_drone_20.dir/build.make
planar_drone_20: /usr/lib/x86_64-linux-gnu/libpython3.8.so
planar_drone_20: libmpc_solver.a
planar_drone_20: CMakeFiles/planar_drone_20.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hojun/project/ACADO/example/planar_drone_ACADO/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable planar_drone_20"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/planar_drone_20.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/planar_drone_20.dir/build: planar_drone_20

.PHONY : CMakeFiles/planar_drone_20.dir/build

CMakeFiles/planar_drone_20.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/planar_drone_20.dir/cmake_clean.cmake
.PHONY : CMakeFiles/planar_drone_20.dir/clean

CMakeFiles/planar_drone_20.dir/depend:
	cd /home/hojun/project/ACADO/example/planar_drone_ACADO/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hojun/project/ACADO/example/planar_drone_ACADO /home/hojun/project/ACADO/example/planar_drone_ACADO /home/hojun/project/ACADO/example/planar_drone_ACADO/build /home/hojun/project/ACADO/example/planar_drone_ACADO/build /home/hojun/project/ACADO/example/planar_drone_ACADO/build/CMakeFiles/planar_drone_20.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/planar_drone_20.dir/depend

