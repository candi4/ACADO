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
CMAKE_SOURCE_DIR = /home/hojun/project/ACADO/example/acado_manual_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hojun/project/ACADO/example/acado_manual_test/build

# Include any dependencies generated for this target.
include CMakeFiles/dae_optimization.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/dae_optimization.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/dae_optimization.dir/flags.make

CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.o: CMakeFiles/dae_optimization.dir/flags.make
CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.o: ../src/chapter3/dae_optimization.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hojun/project/ACADO/example/acado_manual_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.o -c /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3/dae_optimization.cpp

CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3/dae_optimization.cpp > CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.i

CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3/dae_optimization.cpp -o CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.s

# Object files for target dae_optimization
dae_optimization_OBJECTS = \
"CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.o"

# External object files for target dae_optimization
dae_optimization_EXTERNAL_OBJECTS =

dae_optimization: CMakeFiles/dae_optimization.dir/src/chapter3/dae_optimization.cpp.o
dae_optimization: CMakeFiles/dae_optimization.dir/build.make
dae_optimization: /home/hojun/ACADOtoolkit/build/lib/libacado_toolkit_s.so
dae_optimization: /usr/lib/x86_64-linux-gnu/libpython3.8.so
dae_optimization: CMakeFiles/dae_optimization.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hojun/project/ACADO/example/acado_manual_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable dae_optimization"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dae_optimization.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Copying executable to /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3"
	/usr/bin/cmake -E copy /home/hojun/project/ACADO/example/acado_manual_test/build/dae_optimization /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3

# Rule to build all files generated by this target.
CMakeFiles/dae_optimization.dir/build: dae_optimization

.PHONY : CMakeFiles/dae_optimization.dir/build

CMakeFiles/dae_optimization.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dae_optimization.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dae_optimization.dir/clean

CMakeFiles/dae_optimization.dir/depend:
	cd /home/hojun/project/ACADO/example/acado_manual_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hojun/project/ACADO/example/acado_manual_test /home/hojun/project/ACADO/example/acado_manual_test /home/hojun/project/ACADO/example/acado_manual_test/build /home/hojun/project/ACADO/example/acado_manual_test/build /home/hojun/project/ACADO/example/acado_manual_test/build/CMakeFiles/dae_optimization.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dae_optimization.dir/depend
