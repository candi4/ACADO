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
include CMakeFiles/ocp_storing2datastructure.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ocp_storing2datastructure.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ocp_storing2datastructure.dir/flags.make

CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.o: CMakeFiles/ocp_storing2datastructure.dir/flags.make
CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.o: ../src/chapter3/ocp_storing2datastructure.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hojun/project/ACADO/example/acado_manual_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.o -c /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3/ocp_storing2datastructure.cpp

CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3/ocp_storing2datastructure.cpp > CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.i

CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3/ocp_storing2datastructure.cpp -o CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.s

# Object files for target ocp_storing2datastructure
ocp_storing2datastructure_OBJECTS = \
"CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.o"

# External object files for target ocp_storing2datastructure
ocp_storing2datastructure_EXTERNAL_OBJECTS =

ocp_storing2datastructure: CMakeFiles/ocp_storing2datastructure.dir/src/chapter3/ocp_storing2datastructure.cpp.o
ocp_storing2datastructure: CMakeFiles/ocp_storing2datastructure.dir/build.make
ocp_storing2datastructure: /home/hojun/ACADOtoolkit/build/lib/libacado_toolkit_s.so
ocp_storing2datastructure: /usr/lib/x86_64-linux-gnu/libpython3.8.so
ocp_storing2datastructure: CMakeFiles/ocp_storing2datastructure.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hojun/project/ACADO/example/acado_manual_test/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ocp_storing2datastructure"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ocp_storing2datastructure.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Copying executable to /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3"
	/usr/bin/cmake -E copy /home/hojun/project/ACADO/example/acado_manual_test/build/ocp_storing2datastructure /home/hojun/project/ACADO/example/acado_manual_test/src/chapter3

# Rule to build all files generated by this target.
CMakeFiles/ocp_storing2datastructure.dir/build: ocp_storing2datastructure

.PHONY : CMakeFiles/ocp_storing2datastructure.dir/build

CMakeFiles/ocp_storing2datastructure.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ocp_storing2datastructure.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ocp_storing2datastructure.dir/clean

CMakeFiles/ocp_storing2datastructure.dir/depend:
	cd /home/hojun/project/ACADO/example/acado_manual_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hojun/project/ACADO/example/acado_manual_test /home/hojun/project/ACADO/example/acado_manual_test /home/hojun/project/ACADO/example/acado_manual_test/build /home/hojun/project/ACADO/example/acado_manual_test/build /home/hojun/project/ACADO/example/acado_manual_test/build/CMakeFiles/ocp_storing2datastructure.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ocp_storing2datastructure.dir/depend
