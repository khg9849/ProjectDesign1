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
CMAKE_SOURCE_DIR = /home/ubuntu/ProjectDesign1/testH

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/ProjectDesign1/testH/build

# Include any dependencies generated for this target.
include CMakeFiles/stereoCamera_N.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereoCamera_N.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereoCamera_N.dir/flags.make

CMakeFiles/stereoCamera_N.dir/stereoCamera_N.o: CMakeFiles/stereoCamera_N.dir/flags.make
CMakeFiles/stereoCamera_N.dir/stereoCamera_N.o: ../stereoCamera_N.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ubuntu/ProjectDesign1/testH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereoCamera_N.dir/stereoCamera_N.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereoCamera_N.dir/stereoCamera_N.o -c /home/ubuntu/ProjectDesign1/testH/stereoCamera_N.cpp

CMakeFiles/stereoCamera_N.dir/stereoCamera_N.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereoCamera_N.dir/stereoCamera_N.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/ProjectDesign1/testH/stereoCamera_N.cpp > CMakeFiles/stereoCamera_N.dir/stereoCamera_N.i

CMakeFiles/stereoCamera_N.dir/stereoCamera_N.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereoCamera_N.dir/stereoCamera_N.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/ProjectDesign1/testH/stereoCamera_N.cpp -o CMakeFiles/stereoCamera_N.dir/stereoCamera_N.s

# Object files for target stereoCamera_N
stereoCamera_N_OBJECTS = \
"CMakeFiles/stereoCamera_N.dir/stereoCamera_N.o"

# External object files for target stereoCamera_N
stereoCamera_N_EXTERNAL_OBJECTS =

stereoCamera_N: CMakeFiles/stereoCamera_N.dir/stereoCamera_N.o
stereoCamera_N: CMakeFiles/stereoCamera_N.dir/build.make
stereoCamera_N: CMakeFiles/stereoCamera_N.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ubuntu/ProjectDesign1/testH/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereoCamera_N"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereoCamera_N.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereoCamera_N.dir/build: stereoCamera_N

.PHONY : CMakeFiles/stereoCamera_N.dir/build

CMakeFiles/stereoCamera_N.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereoCamera_N.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereoCamera_N.dir/clean

CMakeFiles/stereoCamera_N.dir/depend:
	cd /home/ubuntu/ProjectDesign1/testH/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/ProjectDesign1/testH /home/ubuntu/ProjectDesign1/testH /home/ubuntu/ProjectDesign1/testH/build /home/ubuntu/ProjectDesign1/testH/build /home/ubuntu/ProjectDesign1/testH/build/CMakeFiles/stereoCamera_N.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereoCamera_N.dir/depend

